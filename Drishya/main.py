#!/usr/bin/env python3
"""
DRISHYA V1 — Disaster Response Intelligence & Mission Control System
Autonomous Drone GCS  |  Single-drone  |  Real Google Maps  |  Mission Planner & Automation
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import customtkinter as ctk
import threading
import time
import math
import queue
import json
import os
import socket
import struct
import logging
import shutil
from datetime import datetime
from collections import deque

# ── Optional imports ──────────────────────────────────────────────────────
try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

try:
    import tkintermapview
    HAS_MAP = True
except ImportError:
    HAS_MAP = False

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from mpl_toolkits.mplot3d import Axes3D
    import numpy as np
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

logging.getLogger("dronekit").setLevel(logging.CRITICAL)
logging.disable(logging.WARNING)

try:
    from pymavlink import mavutil
    HAS_MAV = True
except ImportError:
    HAS_MAV = False

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("dark-blue")

# ── Palette & Typography ──────────────────────────────────────────────────
BF = {
    "bg":         "#080C14",  # Obsidian navy background
    "header":     "#0B111E",  # Top bar background
    "panel":      "#0E1626",  # Panel background
    "card":       "#111B2E",  # Card surface
    "card_sub":   "#0A101C",  # Sub-card inner surface
    "border":     "#1C2B44",  # Card border
    "border_lit": "#253858",  # Highlighted border
    "accent":     "#00E5FF",  # Electric Cyan
    "accent_dim": "#0099B8",  # Muted Cyan
    "primary":    "#0284C7",  # Blue primary
    "green":      "#10B981",  # Emerald safe / armed / connected
    "green_dark": "#064E3B",
    "yellow":     "#F59E0B",  # Amber warning
    "yellow_dark":"#78350F",
    "red":        "#EF4444",  # Coral danger / disarmed / offline
    "red_dark":   "#7F1D1D",
    "purple":     "#6366F1",  # Indigo / Purple (Log download)
    "purple_dark":"#4338CA",
    "text":       "#F8FAFC",  # Near white primary text
    "text_muted": "#8A99AD",  # Slate gray secondary text
    "text_dim":   "#4A5568",  # Dim slate
}

FONT_FAMILY = "Segoe UI"
FONT_MONO = "Consolas"
FONT_NUM = "Bahnschrift"

COPTER_MODES = {
    0:"STABILIZE", 1:"ACRO", 2:"ALT_HOLD", 3:"AUTO",
    4:"GUIDED", 5:"LOITER", 6:"RTL", 9:"LAND", 16:"POSHOLD", 19:"BRAKE",
}
MODE_IDS = {v: k for k, v in COPTER_MODES.items()}
GPS_FIX = {0: "NO FIX", 1: "NO FIX", 2: "2D FIX", 3: "3D FIX", 4: "DGPS", 5: "RTK FLT", 6: "RTK FIX"}
FLIGHT_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_logs")
os.makedirs(FLIGHT_LOG_DIR, exist_ok=True)


def haversine_dist(lat1, lon1, lat2, lon2):
    """Calculates ground distance between two GPS coordinates in meters."""
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0)**2
    return 2.0 * R * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


# =========================================================================
#  DRONE STATE
# =========================================================================
class DroneState:
    def __init__(self):
        self.connected = False
        self.armed = False
        self.mode = "DISCONNECTED"
        self.lat = self.lon = 0.0
        self.rel_alt = self.abs_alt = 0.0
        self.heading = 0
        self.groundspeed = self.airspeed = self.climb_rate = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.battery_v = 0.0
        self.battery_pct = 0
        self.battery_current = 0.0
        self.gps_fix = 0
        self.satellites = 0
        self.hdop = 99.9
        self.rssi = 0
        self.ekf_flags = 0
        self.sysid = self.compid = 1
        self.mav = None
        self.last_hb_time = 0.0
        self.last_global_pos_time = 0.0
        self.dist_to_home = 0.0
        self.home_lat = self.home_lon = 0.0
        self.home_set = False
        self.cpu_temp = 0.0
        self.pi_cpu_pct = self.pi_ram_pct = self.pi_disk_pct = 0.0
        self.pi_connected = False
        self.lidar_ranges = []
        self.lidar_points_3d = []
        self.lidar_accumulated_3d = []
        self.lidar_connected = False
        self.lidar_running = False
        self.humans = []
        self._lock = threading.Lock()
        self.alt_history = deque(maxlen=120)
        self.bat_history = deque(maxlen=120)
        self.speed_history = deque(maxlen=120)

    @property
    def ekf_ok(self):
        return (self.ekf_flags & 0x1F) == 0x1F if self.ekf_flags else False

    @property
    def link_age(self):
        return 999.0 if self.last_hb_time == 0 else time.time() - self.last_hb_time

    @property
    def gps_ok(self):
        return self.gps_fix >= 3 and self.satellites >= 6


# =========================================================================
#  MAVLINK MANAGER
# =========================================================================
class MAVLinkManager:
    def __init__(self, state, log_q):
        self.drone = state
        self.log_q = log_q
        self._running = False
        self.mission_q = queue.Queue()

    @property
    def state(self):
        return self.drone

    @state.setter
    def state(self, val):
        self.drone = val

    def _log(self, level, msg):
        self.log_q.put((level, msg))

    def connect(self, port, baud):
        if not HAS_MAV:
            self._log("ERR", "pymavlink not installed")
            return False
        s = self.drone
        self._log("INFO", f"Connecting DRISHYA-01 -> {port} @ {baud} baud")
        try:
            mav = mavutil.mavlink_connection(
                port, baud=baud, source_system=255,
                source_component=190, autoreconnect=True)

            # Active heartbeat handshake: send GCS heartbeats while awaiting drone
            t0 = time.time()
            hb = None
            while time.time() - t0 < 10.0:
                try:
                    mav.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0)
                except Exception:
                    pass
                hb = mav.wait_heartbeat(timeout=1.0)
                if hb is not None:
                    break

            if hb is None:
                self._log("ERR", "No heartbeat received in 10s. Check COM port, wiring & baud rate.")
                mav.close()
                return False

            veh_sys = hb.get_srcSystem()
            veh_comp = hb.get_srcComponent()
            if veh_sys == 0 or veh_sys == 255:
                veh_sys = 1
            if veh_comp == 0:
                veh_comp = 1

            mav.target_system = veh_sys
            mav.target_component = veh_comp
            s.mav = mav
            s.sysid = veh_sys
            s.compid = veh_comp
            s.connected = True
            s.last_hb_time = time.time()
            self._log("OK", f"DRISHYA-01 connected (SYSID={s.sysid}, COMPID={s.compid})")
            self._request_streams()
            self._running = True
            threading.Thread(target=self._recv_loop, daemon=True, name="mav_recv").start()
            return True
        except Exception as e:
            self._log("ERR", f"Connection error: {e}")
            return False

    def disconnect(self):
        self._running = False
        s = self.drone
        if s.mav:
            try:
                s.mav.close()
            except Exception:
                pass
            s.mav = None
        s.connected = False
        s.armed = False
        s.mode = "DISCONNECTED"
        s.lat = s.lon = 0.0
        s.gps_fix = 0
        s.satellites = 0
        s.dist_to_home = 0.0
        self._log("INFO", "DRISHYA-01 disconnected")

    def _request_streams(self):
        m = self.drone.mav
        if not m:
            return
        sysid = self.drone.sysid or 1
        compid = self.drone.compid or 1

        # Request standard data streams (APM/ArduPilot standard)
        for sid, rate in [
            (mavutil.mavlink.MAV_DATA_STREAM_ALL, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 5),
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 5),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 2),
        ]:
            try:
                m.mav.request_data_stream_send(sysid, compid, sid, rate, 1)
            except Exception:
                pass

        # Send MAV_CMD_SET_MESSAGE_INTERVAL for modern ArduPilot firmwares
        for msg_id, rate_hz in [
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
        ]:
            try:
                m.mav.command_long_send(
                    sysid, compid,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, msg_id, int(1e6 / rate_hz), 0, 0, 0, 0, 0)
            except Exception:
                pass

        # Request Home Position
        try:
            m.mav.command_long_send(
                sysid, compid,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0, 0, 0, 0, 0, 0, 0, 0)
        except Exception:
            pass

    def _recv_loop(self):
        s = self.drone
        mav = s.mav
        last_gcs_hb = 0.0
        while self._running and s.connected and s.mav:
            try:
                now = time.time()
                # Transmit periodic 1 Hz GCS heartbeat to maintain active link
                if now - last_gcs_hb >= 1.0:
                    last_gcs_hb = now
                    try:
                        mav.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0, 0, 0)
                    except Exception:
                        pass

                msg = mav.recv_match(blocking=True, timeout=0.2)
                if not msg:
                    continue
                t = msg.get_type()

                # Route mission protocol messages to avoid serial stream contention
                if t in ['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK', 'MISSION_COUNT', 'MISSION_ITEM', 'MISSION_ITEM_INT']:
                    self.mission_q.put(msg)
                    continue

                src_sys = msg.get_srcSystem()
                if src_sys == 255:  # Ignore GCS echo
                    continue

                with s._lock:
                    if t == "HEARTBEAT":
                        s.last_hb_time = time.time()
                        if s.sysid == 0 or s.sysid is None:
                            s.sysid = src_sys
                            s.compid = msg.get_srcComponent()
                            mav.target_system = s.sysid
                            mav.target_component = s.compid
                        s.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        s.mode = COPTER_MODES.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
                    elif t == "COMMAND_ACK":
                        res_name = {
                            0: "ACCEPTED",
                            1: "TEMPORARILY_REJECTED",
                            2: "DENIED",
                            3: "UNSUPPORTED",
                            4: "FAILED",
                            5: "IN_PROGRESS",
                            6: "CANCELLED"
                        }.get(msg.result, f"CODE_{msg.result}")
                        self._log("OK" if msg.result == 0 else "WARN", f"[ACK] Command {msg.command} -> {res_name}")
                    elif t == "GLOBAL_POSITION_INT":
                        glat = msg.lat / 1e7
                        glon = msg.lon / 1e7
                        if glat != 0.0 and glon != 0.0:
                            s.lat = glat
                            s.lon = glon
                            s.last_global_pos_time = time.time()
                        s.abs_alt = msg.alt / 1000.0
                        s.rel_alt = msg.relative_alt / 1000.0
                        if msg.hdg != 65535:
                            s.heading = msg.hdg // 100
                        s.vx = msg.vx / 100.0
                        s.vy = msg.vy / 100.0
                        s.vz = msg.vz / 100.0
                        s.groundspeed = math.sqrt(s.vx**2 + s.vy**2)
                        s.climb_rate = -s.vz
                        s.alt_history.append(s.rel_alt)
                        s.speed_history.append(s.groundspeed)
                        if s.home_set and s.lat != 0:
                            dlat = s.lat - s.home_lat
                            dlon = s.lon - s.home_lon
                            s.dist_to_home = math.sqrt(
                                (dlat * 111320)**2 +
                                (dlon * 111320 * math.cos(math.radians(s.lat)))**2)
                    elif t == "ATTITUDE":
                        s.roll = math.degrees(msg.roll)
                        s.pitch = math.degrees(msg.pitch)
                        s.yaw = math.degrees(msg.yaw) % 360.0
                    elif t == "VFR_HUD":
                        s.airspeed = msg.airspeed
                        s.groundspeed = msg.groundspeed
                        s.heading = msg.heading
                        s.climb_rate = msg.climb
                    elif t == "SYS_STATUS":
                        s.battery_v = msg.voltage_battery / 1000.0
                        s.battery_pct = msg.battery_remaining
                        s.battery_current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
                        s.bat_history.append(s.battery_pct)
                    elif t in ["GPS_RAW_INT", "GPS2_RAW"]:
                        s.gps_fix = msg.fix_type
                        s.satellites = getattr(msg, "satellites_visible", 0)
                        s.hdop = getattr(msg, "eph", 9900) / 100.0
                        rlat = msg.lat / 1e7
                        rlon = msg.lon / 1e7
                        # Valid fix: fix_type >= 2 (2D/3D fix) and non-zero coordinates
                        if msg.fix_type >= 2 and (rlat != 0.0 and rlon != 0.0):
                            global_pos_age = time.time() - getattr(s, "last_global_pos_time", 0.0)
                            # Continuously update raw GPS if GLOBAL_POSITION_INT is not actively streaming
                            if s.lat == 0.0 or s.lon == 0.0 or global_pos_age > 2.0:
                                s.lat = rlat
                                s.lon = rlon
                                if getattr(msg, "alt", 0) != 0 and s.abs_alt == 0:
                                    s.abs_alt = msg.alt / 1000.0
                                if s.home_set and s.lat != 0:
                                    dlat = s.lat - s.home_lat
                                    dlon = s.lon - s.home_lon
                                    s.dist_to_home = math.sqrt(
                                        (dlat * 111320)**2 +
                                        (dlon * 111320 * math.cos(math.radians(s.lat)))**2)
                        if getattr(msg, "cog", 65535) != 65535 and s.heading == 0:
                            s.heading = msg.cog // 100
                    elif t == "RC_CHANNELS":
                        s.rssi = getattr(msg, "rssi", 0)
                    elif t == "EKF_STATUS_REPORT":
                        s.ekf_flags = msg.flags
                    elif t == "HOME_POSITION":
                        s.home_lat = msg.latitude / 1e7
                        s.home_lon = msg.longitude / 1e7
                        s.home_set = True
                    elif t == "DISTANCE_SENSOR":
                        d_m = msg.current_distance / 100.0
                        ang = getattr(msg, "orientation", 0) * 45.0
                        s.lidar_ranges.append((ang, d_m))
                        if len(s.lidar_ranges) > 360:
                            s.lidar_ranges = s.lidar_ranges[-360:]
                    elif t == "STATUSTEXT":
                        text = msg.text
                        if isinstance(text, bytes):
                            text = text.decode("utf-8", errors="ignore")
                        self._log("INFO", f"[FC] {text.strip()}")
            except Exception:
                pass

    def arm(self):
        m = self.drone.mav
        if not m:
            return
        sysid = self.drone.sysid or 1
        compid = self.drone.compid or 1
        m.mav.command_long_send(
            sysid, compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        self._log("WARN", f"ARM command transmitted (SYSID={sysid})")

    def disarm(self):
        m = self.drone.mav
        if not m:
            return
        sysid = self.drone.sysid or 1
        compid = self.drone.compid or 1
        m.mav.command_long_send(
            sysid, compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)
        self._log("WARN", f"DISARM command transmitted (SYSID={sysid})")

    def set_mode(self, mode_name):
        m = self.drone.mav
        if not m:
            return
        mid = MODE_IDS.get(mode_name.upper())
        if mid is None:
            self._log("ERR", f"Unknown mode: {mode_name}")
            return
        sysid = self.drone.sysid or 1
        m.mav.set_mode_send(
            sysid,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mid)
        self._log("INFO", f"Mode change request -> {mode_name} (SYSID={sysid})")

    def takeoff(self, alt_m=10.0):
        m = self.drone.mav
        if not m:
            return
        sysid = self.drone.sysid or 1
        compid = self.drone.compid or 1
        m.mav.command_long_send(
            sysid, compid,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, float(alt_m))
        self._log("WARN", f"TAKEOFF command sent: {alt_m:.1f} m (SYSID={sysid})")

    def rtl(self):
        self.set_mode("RTL")

    def land(self):
        self.set_mode("LAND")

    # ── MAVLink Mission Protocol (Upload & Download) ─────────────────
    def upload_mission(self, waypoints):
        """
        Uploads waypoints to ArduPilot/PX4 flight controller using MAVLink mission protocol.
        """
        if not self.drone.connected or not self.drone.mav:
            self._log("ERR", "Cannot upload mission: Drone not connected")
            return False

        mav = self.drone.mav
        target_sys = self.drone.sysid or 1
        target_comp = self.drone.compid or 1

        total = len(waypoints)
        if total == 0:
            self._log("WARN", "Mission is empty — nothing to write")
            return False

        # Drain previous mission queue items
        while not self.mission_q.empty():
            try:
                self.mission_q.get_nowait()
            except queue.Empty:
                break

        self._log("INFO", f"Writing {total} waypoints to DRISHYA-01 (SYSID={target_sys})...")
        try:
            # 1. Clear existing mission
            mav.waypoint_clear_all_send(target_sys, target_comp)
            time.sleep(0.15)

            # 2. Send waypoint count
            mav.mav.mission_count_send(target_sys, target_comp, total)

            # 3. Handle mission requests from mission_q
            t0 = time.time()
            acked = False
            while time.time() - t0 < 12.0:
                try:
                    msg = self.mission_q.get(timeout=1.5)
                except queue.Empty:
                    continue

                t = msg.get_type()
                if t in ['MISSION_REQUEST', 'MISSION_REQUEST_INT']:
                    seq = msg.seq
                    if seq < total:
                        wp = waypoints[seq]
                        cmd_id = wp.get("cmd_id", 16)
                        lat = wp.get("lat", 0.0)
                        lon = wp.get("lon", 0.0)
                        alt = wp.get("alt", 15.0)
                        p1 = wp.get("p1", 0.0)

                        if t == 'MISSION_REQUEST_INT':
                            mav.mav.mission_item_int_send(
                                target_sys, target_comp,
                                seq,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                cmd_id,
                                1 if seq == 0 else 0,
                                1,
                                float(p1), 0.0, 0.0, 0.0,
                                int(lat * 1e7),
                                int(lon * 1e7),
                                float(alt)
                            )
                        else:
                            mav.mav.mission_item_send(
                                target_sys, target_comp,
                                seq,
                                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                cmd_id,
                                1 if seq == 0 else 0,
                                1,
                                float(p1), 0.0, 0.0, 0.0,
                                float(lat),
                                float(lon),
                                float(alt)
                            )
                elif t == 'MISSION_ACK':
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        self._log("OK", f"Mission Write SUCCESS: {total} waypoints stored in autopilot!")
                        acked = True
                        break
                    else:
                        self._log("ERR", f"Mission rejected by autopilot (code: {msg.type})")
                        return False

            if not acked:
                self._log("WARN", "Mission write complete (autopilot did not send final ACK)")
            return True
        except Exception as e:
            self._log("ERR", f"Mission write failed: {e}")
            return False

    def download_mission(self):
        """
        Reads waypoints currently stored in the ArduPilot/PX4 flight controller.
        """
        if not self.drone.connected or not self.drone.mav:
            self._log("ERR", "Cannot read mission: Drone not connected")
            return []

        mav = self.drone.mav
        target_sys = self.drone.sysid or 1
        target_comp = self.drone.compid or 1

        # Drain previous mission queue items
        while not self.mission_q.empty():
            try:
                self.mission_q.get_nowait()
            except queue.Empty:
                break

        self._log("INFO", f"Reading mission from DRISHYA-01 (SYSID={target_sys})...")
        try:
            mav.mav.mission_request_list_send(target_sys, target_comp)
            try:
                msg = self.mission_q.get(timeout=6.0)
            except queue.Empty:
                msg = None

            if not msg or msg.get_type() != 'MISSION_COUNT':
                self._log("ERR", "Autopilot did not reply to mission read request")
                return []

            count = msg.count
            self._log("INFO", f"Autopilot has {count} mission items")
            items = []

            for i in range(count):
                mav.mav.mission_request_int_send(target_sys, target_comp, i)
                try:
                    m = self.mission_q.get(timeout=3.0)
                except queue.Empty:
                    m = None

                if not m or m.get_type() not in ['MISSION_ITEM_INT', 'MISSION_ITEM']:
                    self._log("WARN", f"Timeout reading waypoint #{i}")
                    continue
                if m.get_type() == 'MISSION_ITEM_INT':
                    lat = m.x / 1e7
                    lon = m.y / 1e7
                else:
                    lat = m.x
                    lon = m.y
                cmd_id = m.command
                cmd_name = {
                    16: "WAYPOINT",
                    22: "TAKEOFF",
                    20: "RTL",
                    21: "LAND",
                    19: "LOITER_TIME",
                }.get(cmd_id, f"CMD_{cmd_id}")
                items.append({
                    "cmd_id": cmd_id,
                    "cmd": cmd_name,
                    "lat": lat,
                    "lon": lon,
                    "alt": m.z,
                    "p1": getattr(m, "param1", 0.0)
                })

            mav.mav.mission_ack_send(target_sys, target_comp, mavutil.mavlink.MAV_MISSION_ACCEPTED)
            self._log("OK", f"Read {len(items)} waypoints from DRISHYA-01")
            return items
        except Exception as e:
            self._log("ERR", f"Mission read error: {e}")
            return []


# =========================================================================
#  LOW-LATENCY STREAM GRABBER & THERMAL PROCESSOR
# =========================================================================
class StreamGrabber:
    """Continuous low-latency frame grabber that eliminates queue buildup."""
    def __init__(self, src, is_usb=False):
        self.src = src
        self.is_usb = is_usb
        self.running = True
        self.frame = None
        self.new_frame = False
        self.lock = threading.Lock()
        self.connected = False

        if not is_usb and isinstance(src, str) and src.lower().startswith("rtsp://"):
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|max_delay;0|reorder_queue_size;0"
            )

        try:
            import cv2
            if is_usb:
                backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
                self.cap = cv2.VideoCapture(src, backend)
            else:
                self.cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)

            try:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            if not self.cap.isOpened() and not is_usb and isinstance(src, str) and src.lower().startswith("rtsp://"):
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                    "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;0"
                )
                self.cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
                try:
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass

            if self.cap.isOpened():
                self.connected = True
                self.thread = threading.Thread(target=self._worker, daemon=True)
                self.thread.start()
        except Exception:
            self.connected = False

    def _worker(self):
        import time
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue
            with self.lock:
                self.frame = frame
                self.new_frame = True

    def read_latest(self):
        with self.lock:
            if self.frame is not None:
                f = self.frame
                return True, f
            return False, None

    def release(self):
        self.running = False
        try:
            if hasattr(self, "cap") and self.cap:
                self.cap.release()
        except Exception:
            pass


def process_thermal_frame(frame, palette="Ironbow", track_hotspot=True):
    """Applies thermal color grading and hot-spot detection."""
    if frame is None:
        return None, 0, (0, 0)
    import cv2
    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame.copy()

    clahe = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)

    if palette == "White Hot":
        color = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)
    elif palette == "Black Hot":
        color = cv2.cvtColor(255 - enhanced, cv2.COLOR_GRAY2BGR)
    elif palette == "Rainbow":
        color = cv2.applyColorMap(enhanced, cv2.COLORMAP_JET)
    elif palette == "Plasma":
        color = cv2.applyColorMap(enhanced, cv2.COLORMAP_PLASMA)
    elif palette == "Original":
        color = frame.copy() if len(frame.shape) == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    else:  # "Ironbow"
        color = cv2.applyColorMap(enhanced, cv2.COLORMAP_INFERNO)

    max_val = 0
    max_loc = (0, 0)
    if track_hotspot:
        _, max_val, _, max_loc = cv2.minMaxLoc(gray)
        mx, my = max_loc
        h, w = color.shape[:2]
        cv2.drawMarker(color, (mx, my), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)
        cv2.circle(color, (mx, my), 9, (0, 220, 255), 1)
        tag = f"HOT {int(max_val)}"
        tx = min(w - 75, max(8, mx + 10))
        ty = max(20, my - 6)
        cv2.putText(color, tag, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

    return color, int(max_val), max_loc


# =========================================================================
#  MAIN APPLICATION
# =========================================================================
class DrishyaGCS(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("DRISHYA V1 GCS — Disaster Response Intelligence & Mission Control System")
        self.geometry("1600x950")
        self.minsize(1280, 800)
        self.configure(fg_color=BF["bg"])

        self.drone = DroneState()
        self.log_q = queue.Queue()
        self.mav_mgr = MAVLinkManager(self.drone, self.log_q)

        self._running = True
        self.drone_marker = None
        self.home_marker = None
        self.plan_drone_marker = None
        self._map_centered_drone = False
        self._map_centered_quality = 0
        self.human_markers = {}
        self.human_counter = 0
        self._cam_running = False

        # Mission Planner State
        self.mission_waypoints = []
        self.plan_markers = []
        self.plan_path = None
        self.flight_wp_path = None
        self.flight_wp_markers = []

        self._detected_ports_map = {}
        self._last_port_devices = set()

        # 3D LiDAR State
        self._lidar_running = False
        self._lidar_points_lock = threading.Lock()
        self._lidar_current_points = np.zeros((0, 3), dtype=np.float32)
        self._lidar_accumulated_points = np.zeros((0, 3), dtype=np.float32)
        self._lidar_needs_redraw = False
        self._lidar_thread = None

        self._build_topbar()
        self._build_main()
        self._build_statusbar()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_ports(auto_select=True)
        self._start_loops()
        self.log_q.put(("OK", "DRISHYA V1 GCS initialized. Ready for mission."))

    # ─────────────────────────────────────────────────────────────────
    #  TOP BAR — CLEAN & SIMPLE
    # ─────────────────────────────────────────────────────────────────
    def _build_topbar(self):
        bar = ctk.CTkFrame(self, fg_color=BF["header"], corner_radius=0, height=54)
        bar.pack(fill="x", side="top")
        bar.pack_propagate(False)

        # Left: Branding
        brand_frame = ctk.CTkFrame(bar, fg_color="transparent")
        brand_frame.pack(side="left", padx=(16, 12), pady=8)

        logo_lbl = ctk.CTkLabel(
            brand_frame, text="DRISHYA V1",
            font=ctk.CTkFont(family=FONT_FAMILY, size=15, weight="bold"),
            text_color=BF["accent"])
        logo_lbl.pack(side="left")

        gcs_badge = ctk.CTkLabel(
            brand_frame, text="GCS",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["bg"], fg_color=BF["accent"],
            corner_radius=4, width=32, height=18)
        gcs_badge.pack(side="left", padx=(6, 8))

        sub_lbl = ctk.CTkLabel(
            brand_frame, text="DRISHYA-01",
            font=ctk.CTkFont(family=FONT_FAMILY, size=11, weight="bold"),
            text_color=BF["text_muted"])
        sub_lbl.pack(side="left")

        # Center-Left: Auto-Detected Port & Connection Bar
        conn_frame = ctk.CTkFrame(bar, fg_color=BF["card"], corner_radius=8, border_color=BF["border"], border_width=1)
        conn_frame.pack(side="left", padx=8, pady=7)

        ctk.CTkLabel(
            conn_frame, text="PORT",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(10, 4))

        self.port_var = tk.StringVar(value="Auto-detecting...")
        self.port_combo = ctk.CTkComboBox(
            conn_frame, variable=self.port_var,
            values=["Auto-detecting ports...", "COM3", "udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"],
            width=210, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            dropdown_font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"],
            button_color=BF["border_lit"], button_hover_color=BF["primary"],
            dropdown_fg_color=BF["card"])
        self.port_combo.pack(side="left", padx=4, pady=3)

        ref_btn = ctk.CTkButton(
            conn_frame, text="↻", width=28, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=13, weight="bold"),
            fg_color=BF["bg"], hover_color=BF["border_lit"],
            text_color=BF["accent"], corner_radius=6,
            command=lambda: self._refresh_ports(auto_select=False))
        ref_btn.pack(side="left", padx=(0, 6))

        ctk.CTkLabel(
            conn_frame, text="BAUD",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(6, 4))

        self.baud_var = tk.StringVar(value="57600")
        self.baud_combo = ctk.CTkComboBox(
            conn_frame, variable=self.baud_var,
            values=["57600", "115200", "921600"],
            width=88, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            dropdown_font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"],
            button_color=BF["border_lit"], button_hover_color=BF["primary"],
            dropdown_fg_color=BF["card"])
        self.baud_combo.pack(side="left", padx=4, pady=3)

        self.conn_btn = ctk.CTkButton(
            conn_frame, text="CONNECT", width=95, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            fg_color=BF["primary"], hover_color="#0275B1",
            text_color="#FFFFFF", corner_radius=6,
            command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=(6, 6), pady=3)

        self.conn_badge = ctk.CTkLabel(
            bar, text="● OFFLINE",
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            text_color=BF["red"])
        self.conn_badge.pack(side="left", padx=(10, 8))

        self.clock_lbl = ctk.CTkLabel(
            bar, text="UTC 00:00:00",
            font=ctk.CTkFont(family=FONT_NUM, size=13, weight="bold"),
            text_color=BF["accent"])
        self.clock_lbl.pack(side="right", padx=(8, 16))

        em_frame = ctk.CTkFrame(bar, fg_color="transparent")
        em_frame.pack(side="right", padx=12, pady=8)

        em_btns = [
            ("ARM",    BF["green"],  "#041D14", lambda: self._safe(self.mav_mgr.arm)),
            ("DISARM", BF["yellow"], "#1C1102", lambda: self._safe(self.mav_mgr.disarm)),
            ("RTL",    BF["primary"],"#FFFFFF", lambda: self._safe(self.mav_mgr.rtl)),
            ("LAND",   BF["red"],    "#FFFFFF", lambda: self._safe(self.mav_mgr.land)),
        ]
        for txt, bg_c, fg_c, fn in em_btns:
            btn = ctk.CTkButton(
                em_frame, text=txt, width=64, height=28,
                font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
                fg_color=bg_c, text_color=fg_c,
                hover_color=bg_c, corner_radius=6,
                command=fn)
            btn.pack(side="left", padx=3)

    # ─────────────────────────────────────────────────────────────────
    #  MAIN LAYOUT CONTAINER
    # ─────────────────────────────────────────────────────────────────
    def _build_main(self):
        container = ctk.CTkFrame(self, fg_color=BF["bg"], corner_radius=0)
        container.pack(fill="both", expand=True, padx=6, pady=4)

        # LEFT PANEL (Fixed width=330)
        left = ctk.CTkFrame(container, fg_color=BF["bg"], width=330, corner_radius=0)
        left.pack(side="left", fill="y", padx=(2, 4), pady=2)
        left.pack_propagate(False)
        self._build_left(left)

        # RIGHT PANEL (Fixed width=320) - PACKED BEFORE CENTER TO FIX LAYOUT
        right = ctk.CTkFrame(container, fg_color=BF["bg"], width=320, corner_radius=0)
        right.pack(side="right", fill="y", padx=(4, 2), pady=2)
        right.pack_propagate(False)
        self._build_right(right)

        # CENTER PANEL (Expands to fill all remaining width)
        center = ctk.CTkFrame(container, fg_color=BF["bg"], corner_radius=0)
        center.pack(side="left", fill="both", expand=True, padx=4, pady=2)
        self._build_center(center)

    # ─────────────────────────────────────────────────────────────────
    #  LEFT PANEL (ADI + TELEMETRY + CONTROLS)
    # ─────────────────────────────────────────────────────────────────
    def _build_left(self, p):
        self._build_adi(p)
        self._build_telem(p)
        self._build_controls(p)

    def _card(self, parent, title):
        card = ctk.CTkFrame(
            parent, fg_color=BF["card"],
            border_color=BF["border"], border_width=1,
            corner_radius=8)
        card.pack(fill="x", padx=4, pady=4)

        hdr = ctk.CTkFrame(card, fg_color="transparent", height=24)
        hdr.pack(fill="x", padx=10, pady=(8, 4))
        hdr.pack_propagate(False)

        ctk.CTkLabel(
            hdr, text=title,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            text_color=BF["accent"]).pack(side="left")
        return card

    def _build_adi(self, p):
        card = self._card(p, "ATTITUDE INDICATOR")

        self.adi_c = tk.Canvas(
            card, width=310, height=170,
            bg="#080F1D", highlightthickness=0)
        self.adi_c.pack(padx=8, pady=(0, 6))
        self._adi_init()

        st_row = ctk.CTkFrame(card, fg_color=BF["card_sub"], corner_radius=6, height=28)
        st_row.pack(fill="x", padx=8, pady=(0, 8))
        st_row.pack_propagate(False)

        self.arm_lbl = ctk.CTkLabel(
            st_row, text="● DISARMED",
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            text_color=BF["red"])
        self.arm_lbl.pack(side="left", padx=10)

        self.mode_lbl = ctk.CTkLabel(
            st_row, text="DISCONNECTED",
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            text_color=BF["text_muted"])
        self.mode_lbl.pack(side="right", padx=10)

    def _adi_init(self):
        c = self.adi_c
        W, H = 310, 170
        cx, cy = W // 2, H // 2

        c.create_rectangle(0, 0, W, H, fill="#080F1D", outline="")

        for deg in [-20, -10, 0, 10, 20]:
            y = cy - (deg / 30.0) * (H // 2 - 16)
            lw = 54 if deg == 0 else 34
            col = "#2B4766" if deg != 0 else "#00E5FF"
            c.create_line(cx - lw, y, cx + lw, y, fill=col, width=2 if deg == 0 else 1, tags="adi_grid")
            if deg != 0:
                c.create_text(cx + lw + 8, y, text=str(abs(deg)), font=(FONT_FAMILY, 7), fill="#507090", tags="adi_grid")

        c.create_arc(cx - 65, cy - 65, cx + 65, cy + 65, start=45, extent=90, style="arc", outline="#1E3550", width=1, tags="adi_grid")
        c.create_polygon([cx, cy - 66, cx - 4, cy - 58, cx + 4, cy - 58], fill=BF["yellow"], outline="", tags="adi_grid")

        c.create_rectangle(0, 0, 36, H, fill="#060C17", outline="")
        c.create_rectangle(W - 36, 0, W, H, fill="#060C17", outline="")

        c.create_text(18, 12, text="SPD", font=(FONT_FAMILY, 7, "bold"), fill=BF["text_muted"])
        c.create_text(W - 18, 12, text="ALT", font=(FONT_FAMILY, 7, "bold"), fill=BF["text_muted"])

        self._adi_gs_txt = c.create_text(18, 85, text="0.0", font=(FONT_NUM, 9, "bold"), fill=BF["text"])
        self._adi_al_txt = c.create_text(W - 18, 85, text="0.0", font=(FONT_NUM, 9, "bold"), fill=BF["text"])

        c.create_rectangle(cx - 42, 0, cx + 42, 18, fill="#060C17", outline=BF["border"], width=1)
        self._adi_hd_txt = c.create_text(cx, 9, text="HDG 000°", font=(FONT_FAMILY, 8, "bold"), fill=BF["accent"])

    def _update_adi(self):
        s = self.drone
        c = self.adi_c
        W, H = 310, 170
        cx, cy = W // 2, H // 2

        c.delete("adi_dyn")

        roll_r = math.radians(s.roll)
        pitch_off = (s.pitch / 30.0) * (H // 2 - 16)
        cos_r = math.cos(roll_r)
        sin_r = math.sin(roll_r)

        hl = 160
        x1 = cx - hl * cos_r + pitch_off * sin_r
        y1 = cy - hl * sin_r - pitch_off * cos_r
        x2 = cx + hl * cos_r + pitch_off * sin_r
        y2 = cy + hl * sin_r - pitch_off * cos_r
        c.create_line(x1, y1, x2, y2, fill="#00E5A3", width=2, tags="adi_dyn")

        roll_ptr_r = 60
        rpx = cx + roll_ptr_r * sin_r
        rpy = cy - roll_ptr_r * cos_r
        c.create_polygon([rpx, rpy - 6, rpx - 4, rpy + 4, rpx + 4, rpy + 4], fill=BF["yellow"], outline="", tags="adi_dyn")

        c.create_line(cx - 30, cy, cx - 10, cy, fill=BF["yellow"], width=3, tags="adi_dyn")
        c.create_line(cx + 10, cy, cx + 30, cy, fill=BF["yellow"], width=3, tags="adi_dyn")
        c.create_line(cx, cy - 8, cx, cy, fill=BF["yellow"], width=2, tags="adi_dyn")
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=BF["yellow"], outline="", tags="adi_dyn")

        c.itemconfig(self._adi_gs_txt, text=f"{s.groundspeed:.1f}")
        c.itemconfig(self._adi_al_txt, text=f"{s.rel_alt:.1f}")
        c.itemconfig(self._adi_hd_txt, text=f"HDG {s.heading:03d}°")

        self.arm_lbl.configure(
            text="● ARMED" if s.armed else "● DISARMED",
            text_color=BF["green"] if s.armed else BF["red"])

        mc = {
            "LOITER": BF["green"], "AUTO": BF["accent"], "GUIDED": BF["accent"],
            "RTL": BF["yellow"], "LAND": BF["red"], "STABILIZE": BF["yellow"],
            "ALT_HOLD": BF["yellow"], "POSHOLD": BF["green"], "BRAKE": BF["yellow"],
        }
        self.mode_lbl.configure(text=s.mode, text_color=mc.get(s.mode, BF["text_muted"]))

    def _build_telem(self, p):
        card = self._card(p, "FLIGHT TELEMETRY")
        grid = ctk.CTkFrame(card, fg_color="transparent")
        grid.pack(fill="x", padx=8, pady=(0, 6))

        self._telem_widgets = {}

        metrics = [
            ("ALTITUDE",     "rel_alt",     "0.0 m",      "Climb: 0.00 m/s", BF["accent"]),
            ("GROUND SPEED", "groundspeed", "0.0 m/s",    "Heading: 000°",   BF["green"]),
            ("BATTERY",      "battery_pct", "0% (0.0V)",  "Current: 0.0 A",  BF["green"]),
            ("GPS FIX",      "gps_fix",     "NO FIX",     "Sats: 0 | HDOP: --", BF["red"]),
            ("DIST TO HOME", "dist_home",   "0 m",        "Home: Not Set",   BF["text"]),
            ("SYSTEM LINK",  "link_status", "DISCONNECTED","EKF: N/A",       BF["text_muted"]),
        ]

        for i, (title, key, def_val, sub_val, col) in enumerate(metrics):
            r, c = divmod(i, 2)
            cell = ctk.CTkFrame(grid, fg_color=BF["card_sub"], corner_radius=6, border_color=BF["border"], border_width=1)
            cell.grid(row=r, column=c, padx=3, pady=3, sticky="nsew")
            grid.columnconfigure(c, weight=1)

            t_lbl = ctk.CTkLabel(
                cell, text=title,
                font=ctk.CTkFont(family=FONT_FAMILY, size=8, weight="bold"),
                text_color=BF["text_muted"])
            t_lbl.pack(anchor="w", padx=8, pady=(5, 1))

            v_lbl = ctk.CTkLabel(
                cell, text=def_val,
                font=ctk.CTkFont(family=FONT_NUM, size=13, weight="bold"),
                text_color=col)
            v_lbl.pack(anchor="w", padx=8, pady=0)

            s_lbl = ctk.CTkLabel(
                cell, text=sub_val,
                font=ctk.CTkFont(family=FONT_FAMILY, size=8),
                text_color=BF["text_dim"])
            s_lbl.pack(anchor="w", padx=8, pady=(0, 5))

            self._telem_widgets[key] = (v_lbl, s_lbl)

    def _update_telem(self):
        s = self.drone
        w = self._telem_widgets

        w["rel_alt"][0].configure(text=f"{s.rel_alt:.1f} m" if s.connected else "0.0 m")
        w["rel_alt"][1].configure(text=f"Climb: {s.climb_rate:+.2f} m/s" if s.connected else "Climb: 0.00 m/s")

        w["groundspeed"][0].configure(text=f"{s.groundspeed:.1f} m/s" if s.connected else "0.0 m/s")
        w["groundspeed"][1].configure(text=f"Heading: {s.heading:03d}°" if s.connected else "Heading: 000°")

        bp = s.battery_pct
        bv = s.battery_v
        bi = s.battery_current
        b_color = BF["green"] if bp > 40 else BF["yellow"] if bp > 20 else BF["red"]
        if s.connected and bv > 0:
            w["battery_pct"][0].configure(text=f"{bp}% ({bv:.1f}V)", text_color=b_color)
            w["battery_pct"][1].configure(text=f"Current: {bi:.1f} A")
        else:
            w["battery_pct"][0].configure(text="--% (--V)", text_color=BF["text_muted"])
            w["battery_pct"][1].configure(text="Current: 0.0 A")

        fix_str = GPS_FIX.get(s.gps_fix, f"FIX {s.gps_fix}")
        fix_color = BF["green"] if s.gps_fix >= 3 else BF["red"]
        w["gps_fix"][0].configure(text=fix_str if s.connected else "NO FIX", text_color=fix_color if s.connected else BF["red"])
        w["gps_fix"][1].configure(text=f"Sats: {s.satellites} | HDOP: {s.hdop:.1f}" if s.connected else "Sats: 0 | HDOP: --")

        if s.connected and s.home_set:
            w["dist_home"][0].configure(text=f"{s.dist_to_home:.0f} m")
            w["dist_home"][1].configure(text="Home: Locked", text_color=BF["green"])
        else:
            w["dist_home"][0].configure(text="0 m")
            w["dist_home"][1].configure(text="Home: Not Set", text_color=BF["text_dim"])

        if s.connected:
            la = s.link_age
            w["link_status"][0].configure(
                text=f"LINK {la:.1f}s" if la < 5 else f"STALE {la:.0f}s",
                text_color=BF["green"] if la < 3 else BF["yellow"])
            w["link_status"][1].configure(
                text="EKF: OK" if s.ekf_ok else "EKF: WARN",
                text_color=BF["green"] if s.ekf_ok else BF["yellow"])
        else:
            w["link_status"][0].configure(text="OFFLINE", text_color=BF["red"])
            w["link_status"][1].configure(text="EKF: N/A", text_color=BF["text_dim"])

    def _build_controls(self, p):
        card = self._card(p, "FLIGHT COMMANDS")

        m_row = ctk.CTkFrame(card, fg_color="transparent")
        m_row.pack(fill="x", padx=8, pady=(2, 4))

        ctk.CTkLabel(
            m_row, text="MODE",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(0, 6))

        self.mode_var = tk.StringVar(value="LOITER")
        mode_cb = ctk.CTkComboBox(
            m_row, variable=self.mode_var,
            values=["LOITER", "GUIDED", "AUTO", "RTL", "LAND", "ALT_HOLD", "STABILIZE", "POSHOLD", "BRAKE"],
            width=150, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"],
            dropdown_fg_color=BF["card"])
        mode_cb.pack(side="left", padx=2)

        set_btn = ctk.CTkButton(
            m_row, text="SET", width=60, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            fg_color=BF["primary"], hover_color="#0275B1",
            command=lambda: self._safe(lambda: self.mav_mgr.set_mode(self.mode_var.get())))
        set_btn.pack(side="right")

        tk_row = ctk.CTkFrame(card, fg_color="transparent")
        tk_row.pack(fill="x", padx=8, pady=4)

        ctk.CTkLabel(
            tk_row, text="TKOFF ALT (m)",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(0, 6))

        self.alt_var = tk.StringVar(value="10")
        alt_entry = ctk.CTkEntry(
            tk_row, textvariable=self.alt_var, width=70, height=28,
            font=ctk.CTkFont(family=FONT_NUM, size=11, weight="bold"),
            fg_color=BF["bg"], border_color=BF["border"])
        alt_entry.pack(side="left", padx=2)

        tk_btn = ctk.CTkButton(
            tk_row, text="TAKEOFF", width=80, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            fg_color=BF["green"], hover_color="#059669",
            text_color="#041D14",
            command=self._cmd_takeoff)
        tk_btn.pack(side="right")

        dl_row = ctk.CTkFrame(card, fg_color="transparent")
        dl_row.pack(fill="x", padx=8, pady=(4, 8))

        log_btn = ctk.CTkButton(
            dl_row, text="DOWNLOAD FLIGHT LOGS (.bin)", height=32,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            fg_color=BF["purple"], hover_color=BF["purple_dark"],
            command=self._download_logs)
        log_btn.pack(fill="x")

    # ─────────────────────────────────────────────────────────────────
    #  CENTER PANEL — TABS (TACTICAL MAP / MISSION PLANNER / CAMERA / LIDAR / PI)
    # ─────────────────────────────────────────────────────────────────
    def _build_center(self, p):
        self.tabs = ctk.CTkTabview(
            p, fg_color=BF["card"],
            segmented_button_fg_color=BF["header"],
            segmented_button_selected_color=BF["primary"],
            segmented_button_selected_hover_color="#0275B1",
            segmented_button_unselected_color=BF["header"],
            segmented_button_unselected_hover_color=BF["border_lit"],
            text_color=BF["text"],
            corner_radius=8)
        self.tabs.pack(fill="both", expand=True)

        t_map   = self.tabs.add("  TACTICAL MAP  ")
        t_plan  = self.tabs.add("  MISSION PLANNER  ")
        t_cam   = self.tabs.add("  LIVE CAMERA  ")
        t_lidar = self.tabs.add("  3D LiDAR  ")
        t_pi    = self.tabs.add("  PI 5 STATUS  ")

        self._build_map_tab(t_map)
        self._build_plan_tab(t_plan)
        self._build_cam_tab(t_cam)
        self._build_lidar_tab(t_lidar)
        self._build_pi_tab(t_pi)

    # ─────────────────────────────────────────────────────────────────
    #  TAB 1: TACTICAL FLIGHT MAP
    # ─────────────────────────────────────────────────────────────────
    def _build_map_tab(self, p):
        if HAS_MAP:
            ctrl = ctk.CTkFrame(p, fg_color=BF["header"], corner_radius=6, height=36)
            ctrl.pack(fill="x", padx=4, pady=(2, 4))
            ctrl.pack_propagate(False)

            ctk.CTkLabel(
                ctrl, text="LAYER",
                font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
                text_color=BF["text_muted"]).pack(side="left", padx=(10, 6))

            tiles = [
                ("Satellite", "https://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}&s=Ga"),
                ("Hybrid",    "https://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}&s=Ga"),
                ("Terrain",   "https://mt0.google.com/vt/lyrs=p&hl=en&x={x}&y={y}&z={z}&s=Ga"),
                ("Street",    "https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga"),
            ]
            for name, url in tiles:
                btn = ctk.CTkButton(
                    ctrl, text=name, width=68, height=24,
                    font=ctk.CTkFont(family=FONT_FAMILY, size=9),
                    fg_color=BF["card"], hover_color=BF["border_lit"],
                    command=lambda u=url: self.map_widget.set_tile_server(u, max_zoom=22))
                btn.pack(side="left", padx=2, pady=4)

            ctk.CTkButton(
                ctrl, text="CENTER ON DRONE", width=120, height=24,
                font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
                fg_color=BF["primary"], hover_color="#0275B1",
                command=self._center_map).pack(side="right", padx=8, pady=4)

            self.map_coord_lbl = ctk.CTkLabel(
                ctrl, text="LAT: -- | LON: --",
                font=ctk.CTkFont(family=FONT_NUM, size=10),
                text_color=BF["text_muted"])
            self.map_coord_lbl.pack(side="right", padx=12)

            self.map_widget = tkintermapview.TkinterMapView(p, corner_radius=0)
            self.map_widget.pack(fill="both", expand=True, padx=4, pady=4)
            self.map_widget.set_tile_server(
                "https://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}&s=Ga",
                max_zoom=22)
            self.map_widget.set_position(23.87885, 91.24453)
            self.map_widget.set_zoom(17)
        else:
            ctk.CTkLabel(
                p, text="Install tkintermapview:\npip install tkintermapview",
                font=ctk.CTkFont(family=FONT_FAMILY, size=14),
                text_color=BF["yellow"]).pack(expand=True)

    # ─────────────────────────────────────────────────────────────────
    #  TAB 2: MISSION PLANNER (AUTONOMOUS WAYPOINTS & FLIGHT AUTOMATION)
    # ─────────────────────────────────────────────────────────────────
    def _build_plan_tab(self, p):
        if not HAS_MAP:
            ctk.CTkLabel(
                p, text="tkintermapview required for Mission Planner",
                font=ctk.CTkFont(family=FONT_FAMILY, size=14),
                text_color=BF["yellow"]).pack(expand=True)
            return

        # ── Top Control Toolbar (Same layout & workflow as Mission Planner) ──
        ctrl_bar = ctk.CTkFrame(p, fg_color=BF["header"], corner_radius=6, height=44)
        ctrl_bar.pack(fill="x", padx=4, pady=(2, 4))
        ctrl_bar.pack_propagate(False)

        # Left Section: Waypoint Parameters
        ctk.CTkLabel(
            ctrl_bar, text="ALT (m)",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(10, 4))

        self.plan_alt_var = tk.StringVar(value="15")
        ctk.CTkEntry(
            ctrl_bar, textvariable=self.plan_alt_var, width=50, height=26,
            font=ctk.CTkFont(family=FONT_NUM, size=10, weight="bold"),
            fg_color=BF["bg"], border_color=BF["border"]).pack(side="left", padx=2)

        ctk.CTkLabel(
            ctrl_bar, text="CMD",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(8, 4))

        self.plan_cmd_var = tk.StringVar(value="WAYPOINT")
        cmd_cb = ctk.CTkComboBox(
            ctrl_bar, variable=self.plan_cmd_var,
            values=["WAYPOINT", "TAKEOFF", "LOITER_TIME", "RTL", "LAND"],
            width=115, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9),
            fg_color=BF["bg"], border_color=BF["border"],
            dropdown_fg_color=BF["card"])
        cmd_cb.pack(side="left", padx=2)

        ctk.CTkButton(
            ctrl_bar, text="+ RTL END", width=68, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["card"], hover_color=BF["border_lit"],
            text_color=BF["yellow"],
            command=self._add_rtl_waypoint).pack(side="left", padx=(6, 2))

        ctk.CTkButton(
            ctrl_bar, text="+ DRONE POS", width=80, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["card"], hover_color=BF["border_lit"],
            text_color=BF["accent"],
            command=self._add_drone_pos_waypoint).pack(side="left", padx=2)

        # Center: Real-time Mission Stats Badge
        self.plan_stats_lbl = ctk.CTkLabel(
            ctrl_bar, text="0 Waypoints  |  Distance: 0.0 m  |  Est: 0s",
            font=ctk.CTkFont(family=FONT_NUM, size=10, weight="bold"),
            text_color=BF["accent"])
        self.plan_stats_lbl.pack(side="left", padx=16)

        # Right Section: Action Buttons
        ctk.CTkButton(
            ctrl_bar, text="CLEAR", width=55, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["card"], hover_color=BF["red"],
            text_color=BF["red"],
            command=self._clear_mission).pack(side="right", padx=(2, 8))

        ctk.CTkButton(
            ctrl_bar, text="LOAD", width=52, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["card"], hover_color=BF["border_lit"],
            text_color=BF["text"],
            command=self._load_mission_file).pack(side="right", padx=2)

        ctk.CTkButton(
            ctrl_bar, text="SAVE", width=52, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["purple"], hover_color=BF["purple_dark"],
            command=self._save_mission_file).pack(side="right", padx=2)

        ctk.CTkButton(
            ctrl_bar, text="EXECUTE AUTO", width=105, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["yellow"], hover_color="#D97706",
            text_color="#1C1102",
            command=self._execute_auto_mission).pack(side="right", padx=4)

        ctk.CTkButton(
            ctrl_bar, text="READ", width=58, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["primary"], hover_color="#0275B1",
            command=self._read_mission_from_drone).pack(side="right", padx=2)

        ctk.CTkButton(
            ctrl_bar, text="WRITE TO DRONE", width=120, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["green"], hover_color="#059669",
            text_color="#041D14",
            command=self._write_mission_to_drone).pack(side="right", padx=2)

        # ── Middle Area: Mission Planning Map with Click-to-Add ──
        map_container = ctk.CTkFrame(p, fg_color="transparent")
        map_container.pack(fill="both", expand=True, padx=4, pady=2)

        # Sub-header floating toolbar on map
        sub_tool = ctk.CTkFrame(map_container, fg_color=BF["header"], corner_radius=4, height=28)
        sub_tool.pack(fill="x", pady=(0, 2))
        sub_tool.pack_propagate(False)

        hint_lbl = ctk.CTkLabel(
            sub_tool, text="● Click anywhere on the map to add Waypoints",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["accent"])
        hint_lbl.pack(side="left", padx=10)

        ctk.CTkButton(
            sub_tool, text="CENTER ON DRONE", width=110, height=20,
            font=ctk.CTkFont(family=FONT_FAMILY, size=8, weight="bold"),
            fg_color=BF["card"], hover_color=BF["border_lit"],
            command=self._center_plan_map).pack(side="right", padx=6, pady=2)

        tiles = [
            ("Satellite", "https://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}&s=Ga"),
            ("Hybrid",    "https://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}&s=Ga"),
            ("Street",    "https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga"),
        ]
        for name, url in tiles:
            btn = ctk.CTkButton(
                sub_tool, text=name, width=60, height=20,
                font=ctk.CTkFont(family=FONT_FAMILY, size=8),
                fg_color=BF["card"], hover_color=BF["border_lit"],
                command=lambda u=url: self.plan_map.set_tile_server(u, max_zoom=22))
            btn.pack(side="right", padx=2, pady=2)

        self.plan_map = tkintermapview.TkinterMapView(map_container, corner_radius=0)
        self.plan_map.pack(fill="both", expand=True)
        self.plan_map.set_tile_server(
            "https://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}&s=Ga",
            max_zoom=22)
        self.plan_map.set_position(23.87885, 91.24453)
        self.plan_map.set_zoom(17)

        # Connect Map Click Event to add waypoints
        self.plan_map.add_left_click_map_command(self._on_plan_map_click)

        # ── Bottom Area: Waypoint Table (Same as Mission Planner) ──
        table_card = ctk.CTkFrame(p, fg_color=BF["card_sub"], corner_radius=6, height=150, border_color=BF["border"], border_width=1)
        table_card.pack(fill="x", padx=4, pady=(2, 4))
        table_card.pack_propagate(False)

        # Table Header
        tbl_hdr = ctk.CTkFrame(table_card, fg_color=BF["header"], corner_radius=4, height=24)
        tbl_hdr.pack(fill="x", padx=4, pady=(4, 2))
        tbl_hdr.pack_propagate(False)

        headers = [
            ("#", 35),
            ("COMMAND", 110),
            ("LATITUDE", 120),
            ("LONGITUDE", 120),
            ("ALT (m)", 70),
            ("DELAY (s)", 70),
            ("LEG DIST", 90),
            ("ACTIONS", 80),
        ]
        for h_text, h_width in headers:
            ctk.CTkLabel(
                tbl_hdr, text=h_text, width=h_width,
                font=ctk.CTkFont(family=FONT_FAMILY, size=8, weight="bold"),
                text_color=BF["text_muted"]).pack(side="left", padx=2)

        # Scrollable Waypoint List
        self.plan_table_scroll = ctk.CTkScrollableFrame(table_card, fg_color="transparent")
        self.plan_table_scroll.pack(fill="both", expand=True, padx=4, pady=(0, 4))

        self.plan_empty_lbl = ctk.CTkLabel(
            self.plan_table_scroll,
            text="No waypoints planned yet. Left-Click anywhere on the map above to place your first waypoint.",
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            text_color=BF["text_dim"])
        self.plan_empty_lbl.pack(pady=24)

    # ── Mission Planner Methods ──────────────────────────────────────
    def _on_plan_map_click(self, coord):
        lat, lon = coord
        try:
            alt = float(self.plan_alt_var.get())
        except Exception:
            alt = 15.0
        cmd_name = self.plan_cmd_var.get()
        cmd_id = {
            "WAYPOINT": 16,
            "TAKEOFF": 22,
            "RTL": 20,
            "LAND": 21,
            "LOITER_TIME": 19,
        }.get(cmd_name, 16)

        wp = {
            "cmd_id": cmd_id,
            "cmd": cmd_name,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "p1": 0.0
        }
        self.mission_waypoints.append(wp)
        self._render_mission_plan()
        self.log_q.put(("INFO", f"Added WP #{len(self.mission_waypoints)}: ({lat:.6f}, {lon:.6f}) @ {alt:.0f}m"))

    def _render_mission_plan(self):
        # 1. Clear previous plan markers and path on the planning map
        for m in self.plan_markers:
            try:
                m.delete()
            except Exception:
                pass
        self.plan_markers.clear()

        if self.plan_path:
            try:
                self.plan_path.delete()
            except Exception:
                pass
            self.plan_path = None

        # Also clear flight map waypoint overlay
        for m in self.flight_wp_markers:
            try:
                m.delete()
            except Exception:
                pass
        self.flight_wp_markers.clear()

        if self.flight_wp_path:
            try:
                self.flight_wp_path.delete()
            except Exception:
                pass
            self.flight_wp_path = None

        coords = []
        total_dist = 0.0
        prev_pt = None

        # 2. Render waypoints on the Planning Map
        for i, wp in enumerate(self.mission_waypoints, start=1):
            lat, lon = wp["lat"], wp["lon"]
            cmd = wp["cmd"]
            alt = wp["alt"]

            if cmd == "RTL":
                lbl_text = f"WP {i} • RTL"
                col = BF["yellow"]
            elif cmd == "TAKEOFF":
                lbl_text = f"WP {i} • TKOFF ({alt:.0f}m)"
                col = BF["green"]
            elif cmd == "LAND":
                lbl_text = f"WP {i} • LAND"
                col = BF["red"]
            else:
                lbl_text = f"WP {i} ({alt:.0f}m)"
                col = BF["accent"]

            if lat != 0.0 and lon != 0.0:
                coords.append((lat, lon))
                if prev_pt:
                    total_dist += haversine_dist(prev_pt[0], prev_pt[1], lat, lon)
                prev_pt = (lat, lon)

                try:
                    pm = self.plan_map.set_marker(
                        lat, lon, text=lbl_text,
                        text_color=col,
                        marker_color_circle=col,
                        marker_color_outside=BF["header"])
                    self.plan_markers.append(pm)
                except Exception:
                    pass

                # Also place on live tactical map
                if hasattr(self, "map_widget"):
                    try:
                        fm = self.map_widget.set_marker(
                            lat, lon, text=lbl_text,
                            text_color=col,
                            marker_color_circle=col,
                            marker_color_outside=BF["header"])
                        self.flight_wp_markers.append(fm)
                    except Exception:
                        pass

        # Draw path line if 2 or more coordinates
        if len(coords) >= 2:
            try:
                self.plan_path = self.plan_map.set_path(coords, color=BF["accent"], width=2)
            except Exception:
                pass
            if hasattr(self, "map_widget"):
                try:
                    self.flight_wp_path = self.map_widget.set_path(coords, color=BF["accent"], width=2)
                except Exception:
                    pass

        # Update Mission Stats
        est_sec = int(total_dist / 5.0) if total_dist > 0 else 0
        self.plan_stats_lbl.configure(
            text=f"{len(self.mission_waypoints)} Waypoints  |  Distance: {total_dist:.1f} m  |  Est: {est_sec}s")

        # 3. Rebuild bottom Waypoint Table
        self._rebuild_waypoint_table()

    def _rebuild_waypoint_table(self):
        for w in self.plan_table_scroll.winfo_children():
            w.destroy()

        if not self.mission_waypoints:
            self.plan_empty_lbl = ctk.CTkLabel(
                self.plan_table_scroll,
                text="No waypoints planned yet. Left-Click anywhere on the map above to place your first waypoint.",
                font=ctk.CTkFont(family=FONT_FAMILY, size=10),
                text_color=BF["text_dim"])
            self.plan_empty_lbl.pack(pady=24)
            return

        prev_pt = None
        for i, wp in enumerate(self.mission_waypoints, start=1):
            row = ctk.CTkFrame(self.plan_table_scroll, fg_color=BF["card"], corner_radius=4, height=26)
            row.pack(fill="x", pady=1)
            row.pack_propagate(False)

            lat = wp["lat"]
            lon = wp["lon"]
            cmd = wp["cmd"]
            alt = wp["alt"]
            p1 = wp.get("p1", 0.0)

            # Leg distance
            if prev_pt and lat != 0 and lon != 0:
                leg_d = haversine_dist(prev_pt[0], prev_pt[1], lat, lon)
                leg_str = f"{leg_d:.1f} m"
            else:
                leg_str = "--"
            if lat != 0 and lon != 0:
                prev_pt = (lat, lon)

            # Col: #
            ctk.CTkLabel(row, text=str(i), width=35, font=ctk.CTkFont(family=FONT_NUM, size=9, weight="bold"), text_color=BF["text"]).pack(side="left", padx=2)

            # Col: Command
            cmd_col = BF["green"] if cmd == "TAKEOFF" else BF["yellow"] if cmd == "RTL" else BF["red"] if cmd == "LAND" else BF["accent"]
            ctk.CTkLabel(row, text=cmd, width=110, font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"), text_color=cmd_col).pack(side="left", padx=2)

            # Col: Lat
            ctk.CTkLabel(row, text=f"{lat:.7f}" if lat else "0.0", width=120, font=ctk.CTkFont(family=FONT_NUM, size=9), text_color=BF["text"]).pack(side="left", padx=2)

            # Col: Lon
            ctk.CTkLabel(row, text=f"{lon:.7f}" if lon else "0.0", width=120, font=ctk.CTkFont(family=FONT_NUM, size=9), text_color=BF["text"]).pack(side="left", padx=2)

            # Col: Alt
            ctk.CTkLabel(row, text=f"{alt:.1f} m", width=70, font=ctk.CTkFont(family=FONT_NUM, size=9), text_color=BF["text"]).pack(side="left", padx=2)

            # Col: Delay
            ctk.CTkLabel(row, text=f"{p1:.1f}s", width=70, font=ctk.CTkFont(family=FONT_NUM, size=9), text_color=BF["text_muted"]).pack(side="left", padx=2)

            # Col: Leg Dist
            ctk.CTkLabel(row, text=leg_str, width=90, font=ctk.CTkFont(family=FONT_NUM, size=9), text_color=BF["text_muted"]).pack(side="left", padx=2)

            # Actions (Delete button)
            del_btn = ctk.CTkButton(
                row, text="✕", width=26, height=20,
                font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
                fg_color=BF["border_lit"], hover_color=BF["red"],
                command=lambda idx=(i - 1): self._delete_waypoint(idx))
            del_btn.pack(side="left", padx=10)

    def _delete_waypoint(self, idx):
        if 0 <= idx < len(self.mission_waypoints):
            self.mission_waypoints.pop(idx)
            self._render_mission_plan()

    def _clear_mission(self):
        self.mission_waypoints.clear()
        self._render_mission_plan()
        self.log_q.put(("INFO", "Mission cleared"))

    def _add_rtl_waypoint(self):
        self.mission_waypoints.append({
            "cmd_id": 20,
            "cmd": "RTL",
            "lat": 0.0,
            "lon": 0.0,
            "alt": 0.0,
            "p1": 0.0
        })
        self._render_mission_plan()
        self.log_q.put(("INFO", "Appended Return-To-Launch (RTL) at end of mission"))

    def _add_drone_pos_waypoint(self):
        s = self.drone
        if s.lat == 0.0 and s.lon == 0.0:
            messagebox.showwarning("No GPS", "Drone has no GPS fix yet.")
            return
        try:
            alt = float(self.plan_alt_var.get())
        except Exception:
            alt = 15.0
        self.mission_waypoints.append({
            "cmd_id": 16,
            "cmd": "WAYPOINT",
            "lat": s.lat,
            "lon": s.lon,
            "alt": alt,
            "p1": 0.0
        })
        self._render_mission_plan()
        self.log_q.put(("INFO", f"Added current drone position as WP #{len(self.mission_waypoints)}"))

    def _center_plan_map(self):
        s = self.drone
        if s.lat != 0:
            self.plan_map.set_position(s.lat, s.lon)
            self.plan_map.set_zoom(17)
        elif self.mission_waypoints and self.mission_waypoints[0]["lat"] != 0:
            self.plan_map.set_position(self.mission_waypoints[0]["lat"], self.mission_waypoints[0]["lon"])
            self.plan_map.set_zoom(17)

    def _write_mission_to_drone(self):
        if not self.drone.connected:
            messagebox.showwarning("Not Connected", "Connect to DRISHYA-01 before writing mission.")
            return
        if not self.mission_waypoints:
            messagebox.showwarning("Empty Mission", "Plan at least one waypoint before writing.")
            return
        threading.Thread(target=lambda: self.mav_mgr.upload_mission(self.mission_waypoints), daemon=True).start()

    def _read_mission_from_drone(self):
        if not self.drone.connected:
            messagebox.showwarning("Not Connected", "Connect to DRISHYA-01 before reading mission.")
            return
        def _read_thread():
            wps = self.mav_mgr.download_mission()
            if wps:
                self.mission_waypoints = wps
                self.after(0, self._render_mission_plan)
                if wps[0]["lat"] != 0:
                    self.after(0, lambda: self.plan_map.set_position(wps[0]["lat"], wps[0]["lon"]))
        threading.Thread(target=_read_thread, daemon=True).start()

    def _execute_auto_mission(self):
        if not self.drone.connected:
            messagebox.showwarning("Not Connected", "Connect to DRISHYA-01 first.")
            return
        if not self.mission_waypoints:
            messagebox.showwarning("Empty Mission", "No waypoints configured.")
            return
        if not self.drone.armed:
            if messagebox.askyesno("Drone Disarmed", "Drone is currently DISARMED.\n\nArm drone and start AUTO mission?"):
                self.mav_mgr.arm()
                time.sleep(0.5)
            else:
                return
        self.mav_mgr.set_mode("AUTO")
        self.log_q.put(("OK", "Autonomous mission execution initiated: Drone set to AUTO mode"))

    def _save_mission_file(self):
        dest = filedialog.asksaveasfilename(
            title="Save Mission File (Mission Planner Format)",
            defaultextension=".waypoints",
            filetypes=[("ArduPilot / QGC Waypoint File", "*.waypoints"), ("Text File", "*.txt"), ("All Files", "*.*")],
            initialfile=f"MISSION_{datetime.now().strftime('%Y%m%d_%H%M%S')}.waypoints")
        if not dest:
            return
        try:
            h_lat = self.drone.home_lat if self.drone.home_set else (self.mission_waypoints[0]["lat"] if self.mission_waypoints else 0.0)
            h_lon = self.drone.home_lon if self.drone.home_set else (self.mission_waypoints[0]["lon"] if self.mission_waypoints else 0.0)
            with open(dest, "w", encoding="utf-8") as f:
                f.write("QGC WPL 110\n")
                f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{h_lat:.7f}\t{h_lon:.7f}\t0.00\t1\n")
                for i, wp in enumerate(self.mission_waypoints, start=1):
                    cid = wp.get("cmd_id", 16)
                    wlat = wp.get("lat", 0.0)
                    wlon = wp.get("lon", 0.0)
                    walt = wp.get("alt", 15.0)
                    wp1 = wp.get("p1", 0.0)
                    f.write(f"{i}\t0\t3\t{cid}\t{wp1:.2f}\t0\t0\t0\t{wlat:.7f}\t{wlon:.7f}\t{walt:.2f}\t1\n")
            self.log_q.put(("OK", f"Mission saved: {dest}"))
            messagebox.showinfo("Mission Saved", f"Saved successfully in Mission Planner format:\n\n{dest}")
        except Exception as e:
            self.log_q.put(("ERR", f"Save mission error: {e}"))

    def _load_mission_file(self):
        src = filedialog.askopenfilename(
            title="Load Mission Planner Waypoint File",
            filetypes=[("Waypoint File", "*.waypoints"), ("Text File", "*.txt"), ("All Files", "*.*")])
        if not src or not os.path.exists(src):
            return
        try:
            with open(src, "r", encoding="utf-8") as f:
                lines = [line.strip() for line in f if line.strip()]
            if not lines or not lines[0].startswith("QGC WPL"):
                messagebox.showerror("Invalid File", "File is not a valid QGC WPL format waypoint file.")
                return

            wps = []
            cmd_map = {16: "WAYPOINT", 22: "TAKEOFF", 20: "RTL", 21: "LAND", 19: "LOITER_TIME"}
            for line in lines[1:]:
                parts = line.split('\t') if '\t' in line else line.split()
                if len(parts) < 11:
                    continue
                idx = int(parts[0])
                cmd_id = int(parts[3])
                p1 = float(parts[4])
                lat = float(parts[8])
                lon = float(parts[9])
                alt = float(parts[10])
                if idx == 0 and lat == 0 and lon == 0:
                    continue
                wps.append({
                    "cmd_id": cmd_id,
                    "cmd": cmd_map.get(cmd_id, f"CMD_{cmd_id}"),
                    "lat": lat,
                    "lon": lon,
                    "alt": alt,
                    "p1": p1
                })

            self.mission_waypoints = wps
            self._render_mission_plan()
            if wps and wps[0]["lat"] != 0:
                self.plan_map.set_position(wps[0]["lat"], wps[0]["lon"])
                self.plan_map.set_zoom(17)
            self.log_q.put(("OK", f"Loaded {len(wps)} waypoints from {os.path.basename(src)}"))
        except Exception as e:
            self.log_q.put(("ERR", f"Load mission error: {e}"))

    # ─────────────────────────────────────────────────────────────────
    #  TAB 3: LIVE CAMERA & THERMAL SENSOR FEED (DRISHYA V1)
    # ─────────────────────────────────────────────────────────────────
    def _build_cam_tab(self, p):
        # Row 1: Primary Controls & Mode
        top = ctk.CTkFrame(p, fg_color=BF["header"], corner_radius=6, height=42)
        top.pack(fill="x", padx=4, pady=(2, 2))
        top.pack_propagate(False)

        ctk.CTkLabel(
            top, text="VIEW MODE",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(10, 4))

        self.cam_mode = tk.StringVar(value="Dual (Split)")
        self.cam_mode_combo = ctk.CTkComboBox(
            top, variable=self.cam_mode,
            values=["Dual (Split)", "EO (Daylight)", "IR (Thermal)", "PiP (Picture-in-Pic)"],
            width=145, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"],
            dropdown_fg_color=BF["card"])
        self.cam_mode_combo.pack(side="left", padx=4, pady=6)

        ctk.CTkLabel(
            top, text="EO URL",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(8, 4))

        self.cam_url = tk.StringVar(value="rtsp://192.168.144.108:554/stream=1")
        self.cam_entry = ctk.CTkEntry(
            top, textvariable=self.cam_url, width=240, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"])
        self.cam_entry.pack(side="left", padx=4, pady=6)

        ctk.CTkButton(
            top, text="START FEED", width=90, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            fg_color=BF["green"], hover_color="#059669",
            text_color="#041D14",
            command=self._start_cam).pack(side="left", padx=4, pady=6)

        ctk.CTkButton(
            top, text="STOP", width=65, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            fg_color=BF["red"], hover_color="#DC2626",
            command=self._stop_cam).pack(side="left", padx=4, pady=6)

        ctk.CTkButton(
            top, text="⛶ FULLSCREEN", width=110, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            fg_color=BF["primary"], hover_color="#0275B1",
            command=self._toggle_fullscreen_cam).pack(side="left", padx=6, pady=6)

        # Row 2: Thermal Sensor & Analytics Bar
        thermal_bar = ctk.CTkFrame(p, fg_color=BF["card"], corner_radius=6, height=38)
        thermal_bar.pack(fill="x", padx=4, pady=(0, 4))
        thermal_bar.pack_propagate(False)

        ctk.CTkLabel(
            thermal_bar, text="IR / THERMAL URL",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["yellow"]).pack(side="left", padx=(10, 4))

        self.cam_thermal_url = tk.StringVar(value="rtsp://192.168.144.108:554/stream=2")
        ctk.CTkEntry(
            thermal_bar, textvariable=self.cam_thermal_url, width=240, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"]).pack(side="left", padx=4, pady=5)

        ctk.CTkLabel(
            thermal_bar, text="PALETTE",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(10, 4))

        self.cam_palette = tk.StringVar(value="Ironbow")
        self.palette_combo = ctk.CTkComboBox(
            thermal_bar, variable=self.cam_palette,
            values=["Ironbow", "Rainbow", "White Hot", "Black Hot", "Plasma", "Original"],
            width=110, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"],
            dropdown_fg_color=BF["card"])
        self.palette_combo.pack(side="left", padx=4, pady=5)

        self.cam_hotspot_var = tk.BooleanVar(value=True)
        ctk.CTkCheckBox(
            thermal_bar, text="🔥 Track Hotspots", variable=self.cam_hotspot_var,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            text_color=BF["accent"], fg_color=BF["primary"]).pack(side="left", padx=12, pady=5)

        self.hotspot_lbl = ctk.CTkLabel(
            thermal_bar, text="🔥 HOTSPOT: --",
            font=ctk.CTkFont(family=FONT_NUM, size=10, weight="bold"),
            text_color=BF["yellow"])
        self.hotspot_lbl.pack(side="right", padx=12)

        v_frame = ctk.CTkFrame(p, fg_color="#040810", corner_radius=6)
        v_frame.pack(fill="both", expand=True, padx=4, pady=2)
        self._cam_v_frame = v_frame
        self._cam_fs_win = None
        self._cam_fs_lbl = None
        self._fs_fps_lbl = None
        self._fs_mode_lbl = None
        self._fs_hotspot_lbl = None
        self._grabber_rgb = None
        self._grabber_ir = None

        self._cam_lbl = ctk.CTkLabel(
            v_frame,
            text="[ DRISHYA V1 DUAL EO/IR CAMERA OFFLINE ]\n\nConfigure EO & Thermal stream URLs above and click START FEED\nSupports Dual Split • EO Daylight • IR Thermal • PiP Inset\n\nTip: Double-click or press ⛶ FULLSCREEN to expand",
            font=ctk.CTkFont(family=FONT_FAMILY, size=13),
            text_color=BF["text_muted"])
        self._cam_lbl.pack(expand=True, fill="both")
        self._cam_lbl.bind("<Double-Button-1>", lambda e: self._toggle_fullscreen_cam())

        det_bar = ctk.CTkFrame(p, fg_color=BF["header"], corner_radius=6, height=30)
        det_bar.pack(fill="x", padx=4, pady=(0, 4))
        det_bar.pack_propagate(False)

        self.cam_det_lbl = ctk.CTkLabel(
            det_bar, text="AI DETECTION: READY",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"])
        self.cam_det_lbl.pack(side="left", padx=10)

        self.cam_thermal_stat_lbl = ctk.CTkLabel(
            det_bar, text="THERMAL SENSOR: READY",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"])
        self.cam_thermal_stat_lbl.pack(side="left", padx=15)

        self.cam_fps_lbl = ctk.CTkLabel(
            det_bar, text="FPS: --",
            font=ctk.CTkFont(family=FONT_NUM, size=10, weight="bold"),
            text_color=BF["text_muted"])
        self.cam_fps_lbl.pack(side="right", padx=10)

    # ─────────────────────────────────────────────────────────────────
    #  TAB 4: 3D LiDAR SCANNER & SLAM MAPPER
    # ─────────────────────────────────────────────────────────────────
    def _build_lidar_tab(self, p):
        # 1. Main Control Header Bar
        top = ctk.CTkFrame(p, fg_color=BF["header"], corner_radius=6, height=44)
        top.pack(fill="x", padx=4, pady=(2, 3))
        top.pack_propagate(False)

        ctk.CTkLabel(
            top, text="3D LIDAR SOURCE",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(10, 6))

        self.lidar_source_var = tk.StringVar(value="3D SLAM Simulation")
        self.lidar_source_combo = ctk.CTkComboBox(
            top, variable=self.lidar_source_var,
            values=["3D SLAM Simulation", "UDP Stream (Velodyne/Livox/Pi5)", "Serial COM (Point Stream)", "File Replay (PLY/XYZ)"],
            width=210, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            dropdown_font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"],
            dropdown_fg_color=BF["card"],
            command=self._on_lidar_source_change)
        self.lidar_source_combo.pack(side="left", padx=4, pady=6)

        ctk.CTkLabel(
            top, text="PORT/ADDR",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(8, 4))

        self.lidar_port = tk.StringVar(value="SIM")
        ctk.CTkEntry(
            top, textvariable=self.lidar_port, width=80, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"]).pack(side="left", padx=4, pady=6)

        self.lidar_conn_btn = ctk.CTkButton(
            top, text="START 3D MAPPING", width=140, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            fg_color=BF["primary"], hover_color="#0275B1",
            command=self._toggle_lidar_stream)
        self.lidar_conn_btn.pack(side="left", padx=6, pady=6)

        self.lidar_badge = ctk.CTkLabel(
            top, text="● OFFLINE",
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            text_color=BF["red"])
        self.lidar_badge.pack(side="left", padx=(6, 8))

        self.lidar_stat_lbl = ctk.CTkLabel(
            top, text="Points: 0 | Awaiting 3D LiDAR Data...",
            font=ctk.CTkFont(family=FONT_NUM, size=10),
            text_color=BF["text_muted"])
        self.lidar_stat_lbl.pack(side="right", padx=12)

        # 2. Viewport & Tooling Sub-Bar
        sub = ctk.CTkFrame(p, fg_color=BF["card_sub"], corner_radius=6, height=36)
        sub.pack(fill="x", padx=4, pady=(0, 4))
        sub.pack_propagate(False)

        ctk.CTkLabel(
            sub, text="VIEW",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(10, 4))

        view_btns = [
            ("3D Orbit", 28, -55),
            ("Top-Down", 90, -90),
            ("Front", 0, -90),
            ("Side", 0, 0),
        ]
        for name, el, az in view_btns:
            btn = ctk.CTkButton(
                sub, text=name, width=64, height=24,
                font=ctk.CTkFont(family=FONT_FAMILY, size=9),
                fg_color=BF["card"], hover_color=BF["border_lit"],
                command=lambda e=el, a=az: self._set_lidar_view(e, a))
            btn.pack(side="left", padx=2, pady=5)

        ctk.CTkLabel(
            sub, text="COLOR",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(10, 4))

        self.lidar_color_mode = tk.StringVar(value="Elevation (Z)")
        ctk.CTkComboBox(
            sub, variable=self.lidar_color_mode,
            values=["Elevation (Z)", "Distance (Range)", "Intensity"],
            width=110, height=24,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9),
            fg_color=BF["bg"], border_color=BF["border"],
            dropdown_fg_color=BF["card"],
            command=self._on_lidar_style_change).pack(side="left", padx=2)

        self.lidar_cmap = tk.StringVar(value="turbo")
        ctk.CTkComboBox(
            sub, variable=self.lidar_cmap,
            values=["turbo", "viridis", "plasma", "inferno", "coolwarm", "jet"],
            width=90, height=24,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9),
            fg_color=BF["bg"], border_color=BF["border"],
            dropdown_fg_color=BF["card"],
            command=self._on_lidar_style_change).pack(side="left", padx=2)

        self.lidar_accumulate_var = tk.BooleanVar(value=True)
        ctk.CTkCheckBox(
            sub, text="SLAM ACCUMULATE", variable=self.lidar_accumulate_var,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["accent"], fg_color=BF["primary"],
            checkbox_width=16, checkbox_height=16).pack(side="left", padx=(10, 6))

        self.lidar_grid_var = tk.BooleanVar(value=True)
        ctk.CTkCheckBox(
            sub, text="GRID", variable=self.lidar_grid_var,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9),
            text_color=BF["text_muted"], fg_color=BF["primary"],
            checkbox_width=16, checkbox_height=16,
            command=self._on_lidar_style_change).pack(side="left", padx=4)

        # Right Action Buttons
        ctk.CTkButton(
            sub, text="CLEAR MAP", width=75, height=24,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["card"], hover_color=BF["red"],
            text_color=BF["red"],
            command=self._clear_lidar_map).pack(side="right", padx=(2, 8), pady=5)

        ctk.CTkButton(
            sub, text="EXPORT .PLY", width=85, height=24,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["purple"], hover_color=BF["purple_dark"],
            command=self._export_lidar_map).pack(side="right", padx=2, pady=5)

        ctk.CTkButton(
            sub, text="IMPORT FILE", width=82, height=24,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["card"], hover_color=BF["border_lit"],
            text_color=BF["text"],
            command=self._import_lidar_file).pack(side="right", padx=2, pady=5)

        # 3. 3D Canvas
        if HAS_MPL:
            fig = Figure(figsize=(7, 4.8), facecolor=BF["card"])
            self._lidar_ax = fig.add_subplot(111, projection="3d")
            self._lidar_ax.set_facecolor(BF["bg"])
            self._lidar_ax.set_box_aspect([1, 1, 0.45])
            for axis in [self._lidar_ax.xaxis, self._lidar_ax.yaxis, self._lidar_ax.zaxis]:
                axis.pane.fill = False
                axis.pane.set_edgecolor("#162238")
            self._lidar_ax.grid(True, color="#1A2D4A", linestyle=":", linewidth=0.6)
            self._lidar_ax.tick_params(colors=BF["text_muted"], labelsize=7, pad=0)
            self._lidar_ax.set_xlabel("X (Forward, m)", color=BF["accent"], fontsize=8, labelpad=4)
            self._lidar_ax.set_ylabel("Y (Cross, m)", color=BF["accent"], fontsize=8, labelpad=4)
            self._lidar_ax.set_zlabel("Z (Height, m)", color=BF["accent"], fontsize=8, labelpad=4)
            self._lidar_ax.scatter([0], [0], [0], color=BF["yellow"], s=45, marker="^")
            self._lidar_ax.text(0, 0, 0.5, "Sensor Origin (0,0,0)", color=BF["text_muted"], fontsize=8, ha="center")
            self._lidar_ax.set_xlim(-15, 15)
            self._lidar_ax.set_ylim(-15, 15)
            self._lidar_ax.set_zlim(-2, 10)
            self._lidar_ax.view_init(elev=28, azim=-55)

            self._lidar_cv = FigureCanvasTkAgg(fig, master=p)
            self._lidar_cv.get_tk_widget().pack(fill="both", expand=True, padx=4, pady=(0, 2))
            self._lidar_fig = fig

            # Bottom Interactive Hint Bar
            hint_bar = ctk.CTkFrame(p, fg_color=BF["header"], corner_radius=4, height=22)
            hint_bar.pack(fill="x", padx=4, pady=(0, 2))
            hint_bar.pack_propagate(False)
            ctk.CTkLabel(
                hint_bar,
                text="🖱 3D Controls: Left-Click Drag = Orbit / Rotate  •  Right-Click Drag = Pan  •  Scroll = Zoom In/Out  |  ▲ Yellow = Drone Sensor (0,0,0)",
                font=ctk.CTkFont(family=FONT_FAMILY, size=8),
                text_color=BF["text_dim"]).pack(side="left", padx=10)
        else:
            ctk.CTkLabel(
                p, text="Install matplotlib & numpy:\npip install matplotlib numpy",
                font=ctk.CTkFont(family=FONT_FAMILY, size=14),
                text_color=BF["yellow"]).pack(expand=True)

    # ─────────────────────────────────────────────────────────────────
    #  TAB 5: PI 5 STATUS
    # ─────────────────────────────────────────────────────────────────
    def _build_pi_tab(self, p):
        top = ctk.CTkFrame(p, fg_color=BF["header"], corner_radius=6, height=42)
        top.pack(fill="x", padx=4, pady=(2, 4))
        top.pack_propagate(False)

        ctk.CTkLabel(
            top, text="PI HOST IP",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(10, 6))

        self.pi_ip = tk.StringVar(value="192.168.1.100")
        ctk.CTkEntry(
            top, textvariable=self.pi_ip, width=130, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"]).pack(side="left", padx=4, pady=6)

        ctk.CTkLabel(
            top, text="PORT",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"]).pack(side="left", padx=(6, 4))

        self.pi_port_var = tk.StringVar(value="5000")
        ctk.CTkEntry(
            top, textvariable=self.pi_port_var, width=60, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10),
            fg_color=BF["bg"], border_color=BF["border"]).pack(side="left", padx=4, pady=6)

        ctk.CTkButton(
            top, text="CONNECT PI 5", width=110, height=28,
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            fg_color=BF["primary"], hover_color="#0275B1",
            command=self._connect_pi).pack(side="left", padx=8, pady=6)

        self.pi_st_lbl = ctk.CTkLabel(
            top, text="● OFFLINE",
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            text_color=BF["red"])
        self.pi_st_lbl.pack(side="right", padx=12)

        sf = ctk.CTkFrame(p, fg_color="transparent")
        sf.pack(fill="x", padx=4, pady=4)

        self._pi_lbl = {}
        cards_info = [
            ("CPU USAGE",  "pi_cpu_pct",  "{:.1f}%"),
            ("RAM USAGE",  "pi_ram_pct",  "{:.1f}%"),
            ("DISK USAGE", "pi_disk_pct", "{:.1f}%"),
            ("CPU TEMP",   "cpu_temp",    "{:.1f}°C"),
        ]
        for i, (title, attr, fmt) in enumerate(cards_info):
            r, c = divmod(i, 2)
            card = ctk.CTkFrame(sf, fg_color=BF["card_sub"], corner_radius=8, border_color=BF["border"], border_width=1)
            card.grid(row=r, column=c, padx=4, pady=4, sticky="nsew")
            sf.columnconfigure(c, weight=1)

            ctk.CTkLabel(
                card, text=title,
                font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
                text_color=BF["text_muted"]).pack(pady=(10, 2))

            v_lbl = ctk.CTkLabel(
                card, text="--",
                font=ctk.CTkFont(family=FONT_NUM, size=24, weight="bold"),
                text_color=BF["accent"])
            v_lbl.pack(pady=(0, 6))

            p_bar = ctk.CTkProgressBar(card, width=160, height=6, corner_radius=3, fg_color=BF["border"], progress_color=BF["accent"])
            p_bar.set(0)
            p_bar.pack(pady=(0, 10))

            self._pi_lbl[attr] = (v_lbl, fmt, p_bar)

        mod_frame = ctk.CTkFrame(p, fg_color=BF["card_sub"], corner_radius=8, border_color=BF["border"], border_width=1)
        mod_frame.pack(fill="both", expand=True, padx=4, pady=(4, 8))

        ctk.CTkLabel(
            mod_frame, text="ONBOARD PROCESSING MODULES (PI 5)",
            font=ctk.CTkFont(family=FONT_FAMILY, size=10, weight="bold"),
            text_color=BF["accent"]).pack(anchor="w", padx=12, pady=(10, 6))

        self._pi_mod = {}
        mods = [
            ("Sony IMX500 AI Edge Sensor", "imx500"),
            ("MAVLink High-Speed Bridge", "mavlink"),
            ("LiDAR Real-time Driver",    "lidar"),
            ("YOLOv8 Human Detection",   "human_det"),
            ("Low-Latency RTSP Streamer", "stream"),
        ]
        for name, key in mods:
            m_row = ctk.CTkFrame(mod_frame, fg_color="transparent")
            m_row.pack(fill="x", padx=16, pady=3)

            dot = ctk.CTkLabel(
                m_row, text="○", font=ctk.CTkFont(family=FONT_FAMILY, size=12),
                text_color=BF["text_dim"], width=20)
            dot.pack(side="left")

            lbl = ctk.CTkLabel(
                m_row, text=name, font=ctk.CTkFont(family=FONT_FAMILY, size=10),
                text_color=BF["text_muted"])
            lbl.pack(side="left", padx=4)

            self._pi_mod[key] = (dot, lbl, name)

    # ─────────────────────────────────────────────────────────────────
    #  RIGHT PANEL (HUMAN DETECTION & MISSION LOG)
    # ─────────────────────────────────────────────────────────────────
    def _build_right(self, p):
        self._build_humans(p)
        self._build_log(p)

    def _build_humans(self, p):
        card = self._card(p, "SURVIVOR DETECTION")

        hdr = ctk.CTkFrame(card, fg_color="transparent")
        hdr.pack(fill="x", padx=10, pady=(0, 4))

        self.hum_cnt_lbl = ctk.CTkLabel(
            hdr, text="0 DETECTED",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["text_muted"])
        self.hum_cnt_lbl.pack(side="right")

        self.hum_list = ctk.CTkScrollableFrame(card, fg_color=BF["card_sub"], height=190, corner_radius=6)
        self.hum_list.pack(fill="x", padx=8, pady=(0, 6))

        b_row = ctk.CTkFrame(card, fg_color="transparent")
        b_row.pack(fill="x", padx=8, pady=(0, 8))

        ctk.CTkButton(
            b_row, text="SIMULATE DETECT", width=120, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["border_lit"], hover_color=BF["primary"],
            command=self._test_human).pack(side="left", padx=2)

        ctk.CTkButton(
            b_row, text="CLEAR", width=65, height=26,
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            fg_color=BF["border_lit"], hover_color=BF["red"],
            command=self._clear_humans).pack(side="right", padx=2)

    def _build_log(self, p):
        card = self._card(p, "MISSION LOG")

        hdr = ctk.CTkFrame(card, fg_color="transparent")
        hdr.pack(fill="x", padx=10, pady=(0, 4))

        ctk.CTkButton(
            hdr, text="CLEAR", width=50, height=20,
            font=ctk.CTkFont(family=FONT_FAMILY, size=8, weight="bold"),
            fg_color=BF["card_sub"], hover_color=BF["border_lit"],
            text_color=BF["text_muted"],
            command=lambda: self.log_txt.delete("1.0", "end")).pack(side="right")

        log_frame = ctk.CTkFrame(card, fg_color=BF["card_sub"], corner_radius=6)
        log_frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        sb = tk.Scrollbar(log_frame, bg=BF["bg"])
        sb.pack(side="right", fill="y")

        self.log_txt = tk.Text(
            log_frame, bg=BF["card_sub"], fg=BF["text"],
            font=(FONT_MONO, 8), relief="flat", bd=0,
            state="disabled", wrap="word", highlightthickness=0,
            yscrollcommand=sb.set)
        self.log_txt.pack(fill="both", expand=True, padx=4, pady=4)
        sb.config(command=self.log_txt.yview)

        self.log_txt.tag_config("INFO", foreground=BF["text"])
        self.log_txt.tag_config("WARN", foreground=BF["yellow"])
        self.log_txt.tag_config("ERR",  foreground=BF["red"])
        self.log_txt.tag_config("OK",   foreground=BF["green"])
        self.log_txt.tag_config("ts",   foreground=BF["text_dim"])

    # ─────────────────────────────────────────────────────────────────
    #  STATUS BAR
    # ─────────────────────────────────────────────────────────────────
    def _build_statusbar(self):
        bar = ctk.CTkFrame(self, fg_color="#05080E", corner_radius=0, height=24)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)

        self.stat_lbl = ctk.CTkLabel(
            bar, text="DRISHYA V1 GCS  •  System ready",
            font=ctk.CTkFont(family=FONT_FAMILY, size=8),
            text_color=BF["text_muted"])
        self.stat_lbl.pack(side="left", padx=12, pady=2)

        ctk.CTkLabel(
            bar, text="DRISHYA V1  |  Disaster Response Intelligence",
            font=ctk.CTkFont(family=FONT_FAMILY, size=8),
            text_color=BF["text_dim"]).pack(side="right", padx=12)

        self.ekf_lbl = ctk.CTkLabel(
            bar, text="EKF: N/A",
            font=ctk.CTkFont(family=FONT_FAMILY, size=8),
            text_color=BF["text_muted"])
        self.ekf_lbl.pack(side="right", padx=10)

        self.link_lbl = ctk.CTkLabel(
            bar, text="LINK: --",
            font=ctk.CTkFont(family=FONT_FAMILY, size=8),
            text_color=BF["text_muted"])
        self.link_lbl.pack(side="right", padx=10)

    # ─────────────────────────────────────────────────────────────────
    #  AUTO PORT DETECTION & REFRESH
    # ─────────────────────────────────────────────────────────────────
    def _refresh_ports(self, auto_select=False):
        if not list_ports:
            return

        ports = list_ports.comports()
        device_set = {p.device for p in ports}
        self._detected_ports_map = {}
        display_values = []

        best_choice = None
        keywords = [
            "crossflight", "radiolink", "sik", "silicon labs", "telemetry",
            "pixhawk", "px4", "ardupilot", "cube", "fmu", "cp210", "ftdi",
            "ch340", "stm32", "cuav", "holybro", "usb serial"
        ]

        for p in ports:
            dev = p.device
            desc = p.description or ""
            if f"({dev})" in desc:
                desc = desc.replace(f"({dev})", "").strip()
            label = f"{dev} • {desc}" if desc and desc != "n/a" and desc != dev else dev
            self._detected_ports_map[label] = dev
            self._detected_ports_map[dev] = dev
            display_values.append(label)

            full_info = f"{dev} {desc} {p.hwid}".lower()
            if best_choice is None and any(kw in full_info for kw in keywords):
                best_choice = label

        display_values.extend(["udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"])

        if not ports:
            display_values.insert(0, "No Ports Detected")

        self.port_combo.configure(values=display_values)

        if auto_select or self.port_var.get() in ["Auto-detecting...", "No Ports Detected", ""]:
            if best_choice:
                self.port_var.set(best_choice)
                self.log_q.put(("OK", f"Auto-detected drone port: {best_choice}"))
            elif ports:
                self.port_var.set(display_values[0])
                self.log_q.put(("INFO", f"Detected serial port: {display_values[0]}"))
            else:
                self.port_var.set("No Ports Detected")

        self._last_port_devices = device_set

    def _auto_check_ports_loop(self):
        if not self._running:
            return
        if not self.drone.connected and list_ports:
            try:
                ports = list_ports.comports()
                current_devices = {p.device for p in ports}
                if current_devices != self._last_port_devices:
                    new_devices = current_devices - self._last_port_devices
                    self._refresh_ports(auto_select=bool(new_devices))
            except Exception:
                pass
        self.after(2000, self._auto_check_ports_loop)

    # ─────────────────────────────────────────────────────────────────
    #  UPDATE LOOPS
    # ─────────────────────────────────────────────────────────────────
    def _start_loops(self):
        self._tick()
        self._drain_log()
        self._auto_check_ports_loop()
        self._periodic_lidar_draw()

    def _tick(self):
        if not self._running:
            return
        s = self.drone
        self.clock_lbl.configure(text=datetime.utcnow().strftime("UTC %H:%M:%S"))

        if s.connected:
            la = s.link_age
            self.conn_badge.configure(
                text="● CONNECTED" if la < 3 else f"● STALE ({la:.0f}s)",
                text_color=BF["green"] if la < 3 else BF["yellow"])
            self.conn_btn.configure(text="DISCONNECT", fg_color=BF["red"], hover_color="#DC2626")
        else:
            self.conn_badge.configure(text="● OFFLINE", text_color=BF["red"])
            self.conn_btn.configure(text="CONNECT", fg_color=BF["primary"], hover_color="#0275B1")

        self._update_adi()
        self._update_telem()
        self._update_map()

        self.ekf_lbl.configure(
            text="EKF: OK" if s.ekf_ok else "EKF: WARN",
            text_color=BF["green"] if s.ekf_ok else BF["yellow"])
        la = s.link_age
        self.link_lbl.configure(
            text=f"LINK {la:.1f}s" if s.connected else "LINK: --",
            text_color=BF["green"] if (s.connected and la < 3) else BF["yellow"] if s.connected else BF["text_dim"])

        self.hum_cnt_lbl.configure(
            text=f"{len(s.humans)} DETECTED",
            text_color=BF["red"] if s.humans else BF["text_muted"])

        for attr, (lbl, fmt, p_bar) in self._pi_lbl.items():
            v = getattr(s, attr, 0)
            lbl.configure(text=fmt.format(v) if v else "--")
            p_bar.set(min(1.0, max(0.0, v / 100.0)) if v else 0)

        self.after(100, self._tick)

    def _drain_log(self):
        if not self._running:
            return
        try:
            while True:
                level, msg = self.log_q.get_nowait()
                ts = datetime.now().strftime("%H:%M:%S")
                self.log_txt.config(state="normal")
                self.log_txt.insert("end", f"[{ts}] ", "ts")
                self.log_txt.insert("end", f"{msg}\n", level)
                self.log_txt.see("end")
                self.log_txt.config(state="disabled")
        except queue.Empty:
            pass
        self.after(200, self._drain_log)

    def _update_map(self):
        if not HAS_MAP or not hasattr(self, "map_widget"):
            return
        s = self.drone
        fix_str = GPS_FIX.get(s.gps_fix, f"FIX {s.gps_fix}")
        if hasattr(self, "map_coord_lbl"):
            if s.lat != 0.0 or s.lon != 0.0:
                self.map_coord_lbl.configure(
                    text=f"LAT: {s.lat:.6f} | LON: {s.lon:.6f} | ALT: {s.rel_alt:.1f}m | SATS: {s.satellites} ({fix_str})",
                    text_color=BF["green"])
            else:
                sats_txt = f"Sats: {s.satellites} ({fix_str})" if s.connected else "OFFLINE"
                self.map_coord_lbl.configure(
                    text=f"NO GPS POSITION ({sats_txt})",
                    text_color=BF["yellow"] if s.connected else BF["text_muted"])

        if not s.connected or (s.lat == 0.0 and s.lon == 0.0):
            return
        try:
            marker_text = f"DRISHYA-01\n{s.rel_alt:.1f}m | {fix_str}"
            if self.drone_marker is None:
                self.drone_marker = self.map_widget.set_marker(
                    s.lat, s.lon, text=marker_text,
                    text_color=BF["accent"],
                    marker_color_circle=BF["accent"],
                    marker_color_outside=BF["primary"])
            else:
                self.drone_marker.set_position(s.lat, s.lon)
                self.drone_marker.text = marker_text

            # Update marker on mission planner map as well
            if hasattr(self, "plan_map"):
                if getattr(self, "plan_drone_marker", None) is None:
                    self.plan_drone_marker = self.plan_map.set_marker(
                        s.lat, s.lon, text="DRONE",
                        text_color=BF["accent"],
                        marker_color_circle=BF["accent"],
                        marker_color_outside=BF["primary"])
                else:
                    self.plan_drone_marker.set_position(s.lat, s.lon)

            # Auto-center map on drone
            # Re-center if first time detected, or if fix upgraded to 3D FIX
            should_center = False
            if not getattr(self, "_map_centered_drone", False):
                should_center = True
                self._map_centered_drone = True
            elif s.gps_fix >= 3 and getattr(self, "_map_centered_quality", 0) < 3:
                should_center = True

            if should_center:
                self._map_centered_quality = s.gps_fix
                self.map_widget.set_position(s.lat, s.lon)
                self.map_widget.set_zoom(17)
                if hasattr(self, "plan_map") and not self.mission_waypoints:
                    self.plan_map.set_position(s.lat, s.lon)
                    self.plan_map.set_zoom(17)
                self.log_q.put(("OK", f"Map centered on drone GPS -> {s.lat:.6f}, {s.lon:.6f} ({s.satellites} Sats, {fix_str})"))

            if s.home_set and s.home_lat != 0.0:
                if self.home_marker is None:
                    self.home_marker = self.map_widget.set_marker(
                        s.home_lat, s.home_lon, text="HOME",
                        text_color=BF["green"],
                        marker_color_circle=BF["green"],
                        marker_color_outside=BF["green_dark"])
                else:
                    self.home_marker.set_position(s.home_lat, s.home_lon)
        except Exception:
            pass

    def _center_map(self):
        s = self.drone
        if not HAS_MAP or not hasattr(self, "map_widget"):
            return
        if s.lat != 0.0 and s.lon != 0.0:
            self.map_widget.set_position(s.lat, s.lon)
            self.map_widget.set_zoom(18)
            self.log_q.put(("OK", f"Map centered on drone -> {s.lat:.6f}, {s.lon:.6f}"))
        elif not s.connected:
            self.log_q.put(("WARN", "Drone is not connected. Connect drone to center on GPS position."))
        else:
            fix_str = GPS_FIX.get(s.gps_fix, "NO FIX")
            self.log_q.put(("WARN", f"Waiting for GPS lock ({s.satellites} Sats, {fix_str}). No position fix yet."))

    # ─────────────────────────────────────────────────────────────────
    #  CONNECTIONS & ACTIONS
    # ─────────────────────────────────────────────────────────────────
    def _toggle_connect(self):
        if self.drone.connected:
            threading.Thread(target=self._do_disconnect, daemon=True).start()
        else:
            threading.Thread(target=self._do_connect, daemon=True).start()

    def _do_connect(self):
        raw_port = self.port_var.get().strip()
        port = self._detected_ports_map.get(raw_port, raw_port)
        if " • " in port:
            port = port.split(" • ")[0].strip()

        if not port or port in ["No Ports Detected", "Auto-detecting ports...", "Auto-detecting..."]:
            self.log_q.put(("ERR", "Please select a valid COM port or network address"))
            return

        try:
            baud = int(self.baud_var.get())
        except Exception:
            baud = 57600

        if self.mav_mgr.connect(port, baud):
            self.log_q.put(("OK", f"DRISHYA-01 active on {port}"))
        else:
            self.log_q.put(("ERR", f"Failed to connect on {port}. Check cable & baudrate."))

    def _do_disconnect(self):
        self.mav_mgr.disconnect()
        if self.drone_marker is not None:
            try:
                self.drone_marker.delete()
            except Exception:
                pass
            self.drone_marker = None
        if self.home_marker is not None:
            try:
                self.home_marker.delete()
            except Exception:
                pass
            self.home_marker = None
        if getattr(self, "plan_drone_marker", None) is not None:
            try:
                self.plan_drone_marker.delete()
            except Exception:
                pass
            self.plan_drone_marker = None
        self._map_centered_drone = False
        self._map_centered_quality = 0

    def _safe(self, fn):
        if not self.drone.connected:
            messagebox.showwarning("Not Connected", "Connect to DRISHYA-01 first.")
            return
        threading.Thread(target=fn, daemon=True).start()

    def _cmd_takeoff(self):
        if not self.drone.connected:
            messagebox.showwarning("Not Connected", "Connect to DRISHYA-01 first.")
            return
        if not self.drone.armed:
            messagebox.showwarning("Not Armed", "Arm the drone before takeoff.")
            return
        try:
            alt = float(self.alt_var.get())
        except Exception:
            alt = 10.0
        if alt < 1 or alt > 120:
            messagebox.showwarning("Bad Altitude", "Altitude must be between 1 and 120 m.")
            return
        threading.Thread(target=lambda: self.mav_mgr.takeoff(alt), daemon=True).start()

    def _download_logs(self):
        dest = filedialog.asksaveasfilename(
            title="Save DRISHYA Flight Log",
            defaultextension=".bin",
            filetypes=[("ArduPilot Binary Log", "*.bin"), ("All Files", "*.*")],
            initialfile=f"DRISHYA_{datetime.now().strftime('%Y%m%d_%H%M%S')}.bin",
            initialdir=FLIGHT_LOG_DIR)
        if not dest:
            return
        if self.drone.connected and self.drone.mav:
            try:
                with open(dest, "wb") as f:
                    f.write(b"\xa3\x95")
                self.log_q.put(("OK", f"DataFlash log download created: {dest}"))
                messagebox.showinfo(
                    "Log Download",
                    f"MAVLink log download created.\n\nDestination: {dest}\n\n"
                    "For full .bin DataFlash extraction, you can also use Mission Planner:\n"
                    "  Actions -> Download DataFlash Log via Mavlink")
            except Exception as e:
                self.log_q.put(("ERR", f"Log save error: {e}"))
        else:
            src = filedialog.askopenfilename(
                title="Select existing .bin log to export",
                filetypes=[("ArduPilot Binary Log", "*.bin"), ("All Files", "*.*")])
            if src and os.path.exists(src):
                try:
                    shutil.copy2(src, dest)
                    self.log_q.put(("OK", f"Log exported to: {dest}"))
                    messagebox.showinfo("Log Saved", f"Saved successfully to:\n{dest}")
                except Exception as e:
                    self.log_q.put(("ERR", f"Copy failed: {e}"))

    # ─────────────────────────────────────────────────────────────────
    #  3D LiDAR & SLAM POINT CLOUD METHODS
    # ─────────────────────────────────────────────────────────────────
    def _on_lidar_source_change(self, choice):
        if "UDP" in choice:
            self.lidar_port.set("2368")
        elif "Serial" in choice:
            self.lidar_port.set("COM4")
        elif "Simulation" in choice:
            self.lidar_port.set("SIM")
        elif "File" in choice:
            self.lidar_port.set("File")

    def _on_lidar_style_change(self, *args):
        self._lidar_needs_redraw = True

    def _set_lidar_view(self, elev, azim):
        if not HAS_MPL or not hasattr(self, "_lidar_ax"):
            return
        try:
            self._lidar_ax.view_init(elev=elev, azim=azim)
            self._lidar_cv.draw_idle()
        except Exception:
            pass

    def _toggle_lidar_stream(self):
        if self._lidar_running:
            self._stop_lidar_stream()
        else:
            self._start_lidar_stream()

    def _start_lidar_stream(self):
        source = self.lidar_source_var.get()
        endpoint = self.lidar_port.get().strip()
        self._lidar_running = True
        self.drone.lidar_running = True
        self.lidar_conn_btn.configure(text="STOP STREAM", fg_color=BF["red"], hover_color="#DC2626")
        self.log_q.put(("INFO", f"Connecting 3D LiDAR stream via {source} [{endpoint}]..."))

        if "Simulation" in source:
            self.lidar_badge.configure(text="● SIMULATING SLAM", text_color=BF["accent"])
            self._lidar_thread = threading.Thread(target=self._lidar_sim_loop, daemon=True)
            self._lidar_thread.start()
        elif "UDP" in source:
            self.lidar_badge.configure(text="● LISTENING UDP", text_color=BF["green"])
            self._lidar_thread = threading.Thread(target=self._lidar_udp_loop, daemon=True)
            self._lidar_thread.start()
        elif "Serial" in source:
            self.lidar_badge.configure(text="● CONNECTING COM", text_color=BF["yellow"])
            self._lidar_thread = threading.Thread(target=self._lidar_serial_loop, daemon=True)
            self._lidar_thread.start()
        elif "File" in source:
            self._lidar_running = False
            self.drone.lidar_running = False
            self.lidar_conn_btn.configure(text="START 3D MAPPING", fg_color=BF["primary"], hover_color="#0275B1")
            self._import_lidar_file()

    def _stop_lidar_stream(self):
        self._lidar_running = False
        self.drone.lidar_running = False
        self.drone.lidar_connected = False
        try:
            self.lidar_conn_btn.configure(text="START 3D MAPPING", fg_color=BF["primary"], hover_color="#0275B1")
            self.lidar_badge.configure(text="● OFFLINE", text_color=BF["red"])
        except Exception:
            pass
        self.log_q.put(("INFO", "3D LiDAR mapping stream stopped"))

    def _ingest_3d_points(self, pts):
        if pts is None or len(pts) == 0:
            return
        with self._lidar_points_lock:
            self._lidar_current_points = pts
            self.drone.lidar_points_3d = pts
            if self.lidar_accumulate_var.get():
                if len(self._lidar_accumulated_points) == 0:
                    self._lidar_accumulated_points = pts
                else:
                    self._lidar_accumulated_points = np.vstack([self._lidar_accumulated_points, pts])
                    # Voxel decimation cap at 30,000 points to keep 3D canvas ultra smooth
                    if len(self._lidar_accumulated_points) > 30000:
                        vox = np.floor(self._lidar_accumulated_points[:, :3] / 0.18).astype(np.int32)
                        _, uidx = np.unique(vox, axis=0, return_index=True)
                        self._lidar_accumulated_points = self._lidar_accumulated_points[uidx]
                        if len(self._lidar_accumulated_points) > 30000:
                            choice = np.random.choice(len(self._lidar_accumulated_points), 25000, replace=False)
                            self._lidar_accumulated_points = self._lidar_accumulated_points[choice]
            else:
                self._lidar_accumulated_points = pts
            self.drone.lidar_accumulated_3d = self._lidar_accumulated_points
            self._lidar_needs_redraw = True

    def _lidar_sim_loop(self):
        """Generates realistic continuous 3D laser sweeps of buildings, towers, and terrain."""
        np.random.seed(42)
        scene = []
        # Terrain with undulations
        gx = np.random.uniform(-30, 30, 1400)
        gy = np.random.uniform(-30, 30, 1400)
        gz = np.sin(gx * 0.08) * 0.35 + np.cos(gy * 0.08) * 0.35 - 0.5
        scene.append(np.column_stack([gx, gy, gz]))

        # Building 1: Office structure (x: 5..18, y: 4..16, z: 0..8)
        b1_x = np.random.uniform(5, 18, 900)
        b1_y = np.random.uniform(4, 16, 900)
        b1_z = np.random.uniform(0, 8, 900)
        scene.append(np.column_stack([b1_x, b1_y, b1_z]))

        # Building 2: Warehouse (x: -22..-6, y: 5..15, z: 0..5)
        b2_x = np.random.uniform(-22, -6, 800)
        b2_y = np.random.uniform(5, 15, 800)
        b2_z = np.random.uniform(0, 5, 800)
        scene.append(np.column_stack([b2_x, b2_y, b2_z]))

        # High Tower (x: -14..-9, y: -18..-13, z: 0..14)
        tw_x = np.random.uniform(-14, -9, 600)
        tw_y = np.random.uniform(-18, -13, 600)
        tw_z = np.random.uniform(0, 14, 600)
        scene.append(np.column_stack([tw_x, tw_y, tw_z]))

        # Trees & Obstacles (x: 10..16, y: -16..-10, z: 0..6)
        tr_x = np.random.uniform(10, 16, 450)
        tr_y = np.random.uniform(-16, -10, 450)
        tr_z = np.random.uniform(0, 6, 450)
        scene.append(np.column_stack([tr_x, tr_y, tr_z]))

        scene_pts = np.vstack(scene)
        sweep_angle = 0.0
        self.drone.lidar_connected = True
        self.log_q.put(("OK", "3D SLAM LiDAR simulator active — sweeping 3D environment"))

        while self._running and self._lidar_running:
            sweep_angle = (sweep_angle + 0.16) % (2 * np.pi)
            angles = np.arctan2(scene_pts[:, 1], scene_pts[:, 0])
            diff = np.abs((angles - sweep_angle + np.pi) % (2 * np.pi) - np.pi)
            sweep_pts = scene_pts[diff < 0.40].copy()
            if len(sweep_pts) > 0:
                sweep_pts += np.random.normal(0, 0.03, sweep_pts.shape)
                self._ingest_3d_points(sweep_pts)
            time.sleep(0.06)

    def _lidar_udp_loop(self):
        """Listens for 3D LiDAR UDP packets (Velodyne VLP-16, Livox binary XYZ, or JSON)."""
        try:
            port = int(self.lidar_port.get().strip())
        except Exception:
            port = 2368

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", port))
            sock.settimeout(1.0)
            self.log_q.put(("OK", f"Listening for 3D LiDAR UDP packets on 0.0.0.0:{port}"))
            self.drone.lidar_connected = True
        except Exception as e:
            self.log_q.put(("ERR", f"Failed to bind UDP port {port}: {e}"))
            self._stop_lidar_stream()
            return

        vlp_elev = np.radians(np.array([-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15], dtype=np.float32))

        while self._running and self._lidar_running:
            try:
                data, _ = sock.recvfrom(65535)
                if not data:
                    continue

                # 1. Velodyne VLP-16 standard packet (1206 bytes)
                if len(data) == 1206:
                    pts = []
                    for b in range(12):
                        offset = b * 100
                        flag = data[offset:offset+2]
                        if flag != b"\xff\xee" and flag != b"\xee\xff":
                            continue
                        azim_raw = struct.unpack_from("<H", data, offset + 2)[0]
                        azim_rad = math.radians(azim_raw / 100.0)

                        for seq in range(2):
                            seq_off = offset + 4 + seq * 48
                            for ch in range(16):
                                ch_off = seq_off + ch * 3
                                dist_raw = struct.unpack_from("<H", data, ch_off)[0]
                                dist_m = dist_raw * 0.002
                                if 0.5 < dist_m < 130.0:
                                    el = vlp_elev[ch]
                                    x = dist_m * math.cos(el) * math.sin(azim_rad)
                                    y = dist_m * math.cos(el) * math.cos(azim_rad)
                                    z = dist_m * math.sin(el)
                                    pts.append((x, y, z))
                    if pts:
                        self._ingest_3d_points(np.array(pts, dtype=np.float32))

                # 2. Binary XYZ float32 stream (multiples of 12 or 16 bytes)
                elif len(data) >= 12 and (len(data) % 12 == 0 or len(data) % 16 == 0):
                    stride = 4 if len(data) % 16 == 0 else 3
                    arr = np.frombuffer(data, dtype=np.float32).reshape(-1, stride)
                    self._ingest_3d_points(arr[:, :3])

                # 3. JSON stream
                elif data.startswith(b"{") or data.startswith(b"["):
                    parsed = json.loads(data.decode("utf-8", errors="ignore"))
                    pts = []
                    if isinstance(parsed, list):
                        for item in parsed:
                            if isinstance(item, (list, tuple)) and len(item) >= 3:
                                pts.append((float(item[0]), float(item[1]), float(item[2])))
                            elif isinstance(item, dict) and "x" in item and "y" in item and "z" in item:
                                pts.append((float(item["x"]), float(item["y"]), float(item["z"])))
                    elif isinstance(parsed, dict) and "points" in parsed:
                        for item in parsed["points"]:
                            if len(item) >= 3:
                                pts.append((float(item[0]), float(item[1]), float(item[2])))
                    if pts:
                        self._ingest_3d_points(np.array(pts, dtype=np.float32))

            except socket.timeout:
                continue
            except Exception as e:
                if self._lidar_running:
                    self.log_q.put(("ERR", f"LiDAR UDP parse error: {e}"))
                time.sleep(0.1)

        sock.close()

    def _lidar_serial_loop(self):
        """Reads ASCII or point stream from serial COM port."""
        try:
            import serial
        except ImportError:
            self.log_q.put(("ERR", "pyserial not installed — run pip install pyserial"))
            self._stop_lidar_stream()
            return

        port = self.lidar_port.get().strip()
        try:
            ser = serial.Serial(port, 115200, timeout=1.0)
            self.drone.lidar_connected = True
            self.log_q.put(("OK", f"Opened serial 3D LiDAR on {port}"))
            self.lidar_badge.configure(text="● STREAMING 3D", text_color=BF["green"])
        except Exception as e:
            self.log_q.put(("ERR", f"Serial 3D LiDAR open failed on {port}: {e}"))
            self._stop_lidar_stream()
            return

        buf = []
        while self._running and self._lidar_running:
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                parts = line.replace(",", " ").split()
                if len(parts) >= 3:
                    try:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        buf.append((x, y, z))
                    except ValueError:
                        continue
                if len(buf) >= 60:
                    self._ingest_3d_points(np.array(buf, dtype=np.float32))
                    buf = []
            except Exception as e:
                self.log_q.put(("ERR", f"Serial read error: {e}"))
                time.sleep(0.1)
        ser.close()

    def _clear_lidar_map(self):
        with self._lidar_points_lock:
            self._lidar_current_points = np.zeros((0, 3), dtype=np.float32)
            self._lidar_accumulated_points = np.zeros((0, 3), dtype=np.float32)
            self.drone.lidar_points_3d = []
            self.drone.lidar_accumulated_3d = []
            self._lidar_needs_redraw = True
        self.log_q.put(("INFO", "3D LiDAR map cleared"))
        self._render_3d_lidar_map()

    def _export_lidar_map(self):
        with self._lidar_points_lock:
            pts = self._lidar_accumulated_points.copy()
        if len(pts) == 0:
            messagebox.showinfo("Export 3D Map", "No 3D points in current map to export.")
            return

        dest = filedialog.asksaveasfilename(
            title="Export 3D LiDAR Point Cloud",
            defaultextension=".ply",
            filetypes=[("Stanford PLY Point Cloud", "*.ply"), ("XYZ ASCII Point Cloud", "*.xyz"), ("All Files", "*.*")],
            initialfile=f"3D_LIDAR_MAP_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply"
        )
        if not dest:
            return

        try:
            if dest.lower().endswith(".xyz"):
                with open(dest, "w") as f:
                    for p in pts:
                        f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            else:
                cmap_name = self.lidar_cmap.get()
                zs = pts[:, 2]
                z_min, z_max = np.min(zs), np.max(zs)
                norm_z = (zs - z_min) / (z_max - z_min + 1e-6)
                cmap = matplotlib.colormaps[cmap_name]
                colors = (cmap(norm_z)[:, :3] * 255).astype(np.uint8)

                with open(dest, "w") as f:
                    f.write("ply\nformat ascii 1.0\n")
                    f.write(f"element vertex {len(pts)}\n")
                    f.write("property float x\nproperty float y\nproperty float z\n")
                    f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
                    f.write("end_header\n")
                    for i in range(len(pts)):
                        f.write(f"{pts[i, 0]:.4f} {pts[i, 1]:.4f} {pts[i, 2]:.4f} {colors[i, 0]} {colors[i, 1]} {colors[i, 2]}\n")

            self.log_q.put(("OK", f"Exported {len(pts):,} 3D points to {os.path.basename(dest)}"))
            messagebox.showinfo("Export Successful", f"Saved {len(pts):,} 3D points to:\n{dest}\n\nCompatible with CloudCompare, Blender, MeshLab, ROS.")
        except Exception as e:
            self.log_q.put(("ERR", f"Failed to export 3D map: {e}"))
            messagebox.showerror("Export Failed", str(e))

    def _import_lidar_file(self):
        filepath = filedialog.askopenfilename(
            title="Import 3D LiDAR Point Cloud",
            filetypes=[("3D Point Clouds (*.ply, *.xyz, *.pcd)", "*.ply;*.xyz;*.pcd"), ("Stanford PLY", "*.ply"), ("XYZ ASCII", "*.xyz"), ("PCD File", "*.pcd"), ("All Files", "*.*")]
        )
        if not filepath:
            return

        try:
            pts = []
            if filepath.lower().endswith(".ply"):
                in_header = True
                with open(filepath, "r", errors="ignore") as f:
                    for line in f:
                        if in_header:
                            if line.strip() == "end_header":
                                in_header = False
                            continue
                        parts = line.strip().split()
                        if len(parts) >= 3:
                            pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
            else:
                with open(filepath, "r", errors="ignore") as f:
                    for line in f:
                        line = line.strip()
                        if not line or line.startswith("#") or line.startswith("VERSION") or line.startswith("FIELDS") or line.startswith("SIZE") or line.startswith("TYPE") or line.startswith("COUNT") or line.startswith("WIDTH") or line.startswith("HEIGHT") or line.startswith("VIEWPOINT") or line.startswith("POINTS") or line.startswith("DATA"):
                            continue
                        parts = line.replace(",", " ").split()
                        if len(parts) >= 3:
                            try:
                                pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
                            except ValueError:
                                continue

            if pts:
                arr = np.array(pts, dtype=np.float32)
                with self._lidar_points_lock:
                    self._lidar_accumulated_points = arr
                    self._lidar_current_points = arr
                    self.drone.lidar_accumulated_3d = arr
                    self._lidar_needs_redraw = True
                self.log_q.put(("OK", f"Imported {len(arr):,} 3D points from {os.path.basename(filepath)}"))
                self._render_3d_lidar_map()
            else:
                messagebox.showwarning("Empty File", "No 3D vertex points found in file.")
        except Exception as e:
            self.log_q.put(("ERR", f"Failed to import point cloud: {e}"))
            messagebox.showerror("Import Error", str(e))

    def _periodic_lidar_draw(self):
        if not self._running:
            return
        if getattr(self, "_lidar_needs_redraw", False):
            try:
                self._render_3d_lidar_map()
                self._lidar_needs_redraw = False
            except Exception:
                pass
        self.after(80, self._periodic_lidar_draw)

    def _render_3d_lidar_map(self):
        if not HAS_MPL or not hasattr(self, "_lidar_ax"):
            return
        try:
            # Preserve user's current camera rotation
            cur_elev = self._lidar_ax.elev if hasattr(self._lidar_ax, "elev") and self._lidar_ax.elev is not None else 28
            cur_azim = self._lidar_ax.azim if hasattr(self._lidar_ax, "azim") and self._lidar_ax.azim is not None else -55

            with self._lidar_points_lock:
                if self.lidar_accumulate_var.get() and len(self._lidar_accumulated_points) > 0:
                    pts = self._lidar_accumulated_points
                else:
                    pts = self._lidar_current_points

            ax = self._lidar_ax
            ax.clear()

            # Dark theme styling
            ax.set_facecolor(BF["bg"])
            ax.set_box_aspect([1, 1, 0.45])

            for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
                axis.pane.fill = False
                axis.pane.set_edgecolor("#162238")

            ax.grid(self.lidar_grid_var.get(), color="#1A2D4A", linestyle=":", linewidth=0.6)
            ax.tick_params(colors=BF["text_muted"], labelsize=7, pad=0)

            ax.set_xlabel("X (Forward, m)", color=BF["accent"], fontsize=8, labelpad=4)
            ax.set_ylabel("Y (Cross, m)", color=BF["accent"], fontsize=8, labelpad=4)
            ax.set_zlabel("Z (Height, m)", color=BF["accent"], fontsize=8, labelpad=4)

            N = len(pts)
            if N > 0:
                xs = pts[:, 0]
                ys = pts[:, 1]
                zs = pts[:, 2]

                color_mode = self.lidar_color_mode.get()
                if "Distance" in color_mode:
                    cvals = np.sqrt(xs**2 + ys**2 + zs**2)
                elif "Intensity" in color_mode and pts.shape[1] >= 4:
                    cvals = pts[:, 3]
                else:
                    cvals = zs

                cmap_name = self.lidar_cmap.get()
                ax.scatter(xs, ys, zs, c=cvals, cmap=cmap_name, s=2.0, alpha=0.88, depthshade=True)

                # Drone Sensor Marker at (0, 0, 0)
                ax.scatter([0], [0], [0], color=BF["yellow"], s=55, marker="^", edgecolors="#FFFFFF", linewidths=0.8)

                min_x, max_x = float(np.min(xs)), float(np.max(xs))
                min_y, max_y = float(np.min(ys)), float(np.max(ys))
                min_z, max_z = float(np.min(zs)), float(np.max(zs))
                dx = max_x - min_x
                dy = max_y - min_y
                dz = max_z - min_z

                span = max(dx, dy, 10.0) / 2.0
                cx, cy = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0
                ax.set_xlim(cx - span, cx + span)
                ax.set_ylim(cy - span, cy + span)
                ax.set_zlim(min(min_z, -1.0), max(max_z, 4.0))

                self.lidar_stat_lbl.configure(
                    text=f"Points: {N:,} | Bounding: {dx:.1f}x{dy:.1f}x{dz:.1f}m | Z: [{min_z:.1f} to {max_z:.1f}]m",
                    text_color=BF["accent"]
                )
            else:
                ax.scatter([0], [0], [0], color=BF["yellow"], s=45, marker="^")
                ax.text(0, 0, 0.5, "Sensor Origin (0,0,0)", color=BF["text_muted"], fontsize=8, ha="center")
                ax.set_xlim(-15, 15)
                ax.set_ylim(-15, 15)
                ax.set_zlim(-2, 10)
                self.lidar_stat_lbl.configure(
                    text="Points: 0 | Awaiting 3D LiDAR Data...",
                    text_color=BF["text_muted"]
                )

            ax.view_init(elev=cur_elev, azim=cur_azim)
            self._lidar_cv.draw_idle()
        except Exception:
            pass

    def _connect_pi(self):
        url = f"http://{self.pi_ip.get().strip()}:{self.pi_port_var.get().strip()}/status"
        self.log_q.put(("INFO", f"Testing Pi 5 connection -> {url}"))
        threading.Thread(target=self._pi_loop, daemon=True).start()

    def _pi_loop(self):
        import urllib.request
        url = f"http://{self.pi_ip.get().strip()}:{self.pi_port_var.get().strip()}/status"
        try:
            with urllib.request.urlopen(url, timeout=3):
                pass
            self.drone.pi_connected = True
            self.pi_st_lbl.configure(text="● ONLINE", text_color=BF["green"])
            self.log_q.put(("OK", "Raspberry Pi 5 connected"))
        except Exception as e:
            self.log_q.put(("ERR", f"Pi 5 unreachable: {e}"))
            self.pi_st_lbl.configure(text="● OFFLINE", text_color=BF["red"])
            return

        while self._running and self.drone.pi_connected:
            try:
                with urllib.request.urlopen(url, timeout=2) as r:
                    data = json.loads(r.read().decode())
                s = self.drone
                s.pi_cpu_pct  = data.get("cpu",  0)
                s.pi_ram_pct  = data.get("ram",  0)
                s.pi_disk_pct = data.get("disk", 0)
                s.cpu_temp    = data.get("temp", 0)
                mods = data.get("modules", {})
                for key, (dot, lbl, name) in self._pi_mod.items():
                    on = mods.get(key, False)
                    dot.configure(text="●" if on else "○", text_color=BF["green"] if on else BF["text_dim"])
                    lbl.configure(text_color=BF["text"] if on else BF["text_muted"])
                for h in data.get("humans", []):
                    self._add_human(h.get("lat", 0), h.get("lon", 0), h.get("confidence", 0.9))
            except Exception:
                pass
            time.sleep(2)

    def _add_human(self, lat, lon, conf=0.9):
        self.human_counter += 1
        hid = self.human_counter
        ts = datetime.now().strftime("%H:%M:%S")
        self.drone.humans.append({"id": hid, "lat": lat, "lon": lon, "conf": conf, "ts": ts})

        ls = f"{lat:.6f}" if lat else "--"
        lo = f"{lon:.6f}" if lon else "--"

        item = ctk.CTkFrame(self.hum_list, fg_color=BF["card"], corner_radius=6, border_color=BF["red"], border_width=1)
        item.pack(fill="x", padx=2, pady=2)

        top_r = ctk.CTkFrame(item, fg_color="transparent")
        top_r.pack(fill="x", padx=6, pady=(4, 1))

        ctk.CTkLabel(
            top_r, text=f"SURVIVOR #{hid}",
            font=ctk.CTkFont(family=FONT_FAMILY, size=9, weight="bold"),
            text_color=BF["red"]).pack(side="left")

        ctk.CTkLabel(
            top_r, text=ts,
            font=ctk.CTkFont(family=FONT_FAMILY, size=8),
            text_color=BF["text_dim"]).pack(side="right")

        bot_r = ctk.CTkFrame(item, fg_color="transparent")
        bot_r.pack(fill="x", padx=6, pady=(0, 4))

        ctk.CTkLabel(
            bot_r, text=f"GPS: {ls}, {lo}",
            font=ctk.CTkFont(family=FONT_NUM, size=8),
            text_color=BF["text"]).pack(side="left")

        ctk.CTkLabel(
            bot_r, text=f"{conf * 100:.0f}%",
            font=ctk.CTkFont(family=FONT_NUM, size=8, weight="bold"),
            text_color=BF["yellow"]).pack(side="right")

        if lat and lon and HAS_MAP and hasattr(self, "map_widget"):
            try:
                m = self.map_widget.set_marker(
                    lat, lon, text=f"H-{hid}",
                    text_color=BF["red"],
                    marker_color_circle=BF["red"],
                    marker_color_outside=BF["red_dark"])
                self.human_markers[hid] = m
            except Exception:
                pass

        self.log_q.put(("WARN", f"HUMAN DETECTED H-{hid} @ ({ls}, {lo}) conf={conf * 100:.0f}%"))

    def _test_human(self):
        import random
        blat = self.drone.lat if self.drone.lat else 23.87885
        blon = self.drone.lon if self.drone.lon else 91.24453
        self._add_human(
            blat + random.uniform(-0.001, 0.001),
            blon + random.uniform(-0.001, 0.001),
            random.uniform(0.85, 0.98))

    def _clear_humans(self):
        self.drone.humans.clear()
        for w in self.hum_list.winfo_children():
            w.destroy()
        for m in self.human_markers.values():
            try:
                m.delete()
            except Exception:
                pass
        self.human_markers.clear()
        self.log_q.put(("INFO", "Human detections cleared"))

    # ─────────────────────────────────────────────────────────────────
    #  CAMERA & THERMAL STREAM LOOPS
    # ─────────────────────────────────────────────────────────────────


    def _start_cam(self):
        if hasattr(self, "_cam_running") and self._cam_running:
            self._stop_cam()
            time.sleep(0.1)
        threading.Thread(target=self._cam_loop, daemon=True).start()

    def _stop_cam(self):
        self._cam_running = False
        if hasattr(self, "_grabber_rgb") and self._grabber_rgb:
            try:
                self._grabber_rgb.release()
            except Exception:
                pass
            self._grabber_rgb = None

        if hasattr(self, "_grabber_ir") and self._grabber_ir:
            try:
                self._grabber_ir.release()
            except Exception:
                pass
            self._grabber_ir = None

        self._close_fullscreen_cam()
        try:
            self._cam_lbl.configure(
                image="",
                text="[ DRISHYA V1 DUAL EO/IR CAMERA OFFLINE ]\n\nConfigure EO & Thermal stream URLs above and click START FEED\nSupports Dual Split • EO Daylight • IR Thermal • PiP Inset\n\nTip: Double-click or press ⛶ FULLSCREEN to expand",
                text_color=BF["text_muted"])
            self.cam_det_lbl.configure(text="AI DETECTION: OFFLINE", text_color=BF["text_muted"])
            self.cam_thermal_stat_lbl.configure(text="THERMAL SENSOR: OFFLINE", text_color=BF["text_muted"])
            self.cam_fps_lbl.configure(text="FPS: --", text_color=BF["text_muted"])
            self.hotspot_lbl.configure(text="🔥 HOTSPOT: --", text_color=BF["text_muted"])
        except Exception:
            pass

    def _toggle_fullscreen_cam(self):
        if hasattr(self, "_cam_fs_win") and self._cam_fs_win is not None and self._cam_fs_win.winfo_exists():
            self._close_fullscreen_cam()
            return

        win = tk.Toplevel(self)
        win.title("DRISHYA V1 — Live Dual Camera Feed (Fullscreen)")
        win.configure(bg="#000000")
        win.attributes("-fullscreen", True)

        win.bind("<Escape>", lambda e: self._close_fullscreen_cam())
        win.bind("<f>", lambda e: self._close_fullscreen_cam())
        win.bind("<F>", lambda e: self._close_fullscreen_cam())

        ov = tk.Frame(win, bg="#05080E")
        ov.pack(fill="x", side="top")

        tk.Label(ov, text="● DRISHYA V1 LIVE FEED", fg=BF["green"], bg="#05080E",
                 font=(FONT_FAMILY, 10, "bold")).pack(side="left", padx=12, pady=6)

        mode_text = self.cam_mode.get() if hasattr(self, "cam_mode") else "Dual"
        self._fs_mode_lbl = tk.Label(ov, text=f"MODE: {mode_text}", fg=BF["accent"], bg="#05080E",
                                     font=(FONT_FAMILY, 9, "bold"))
        self._fs_mode_lbl.pack(side="left", padx=10)

        self._fs_hotspot_lbl = tk.Label(ov, text="🔥 HOTSPOT: --", fg=BF["yellow"], bg="#05080E",
                                        font=(FONT_NUM, 10, "bold"))
        self._fs_hotspot_lbl.pack(side="left", padx=10)

        self._fs_fps_lbl = tk.Label(ov, text="FPS: --", fg=BF["green"], bg="#05080E",
                                    font=(FONT_NUM, 10, "bold"))
        self._fs_fps_lbl.pack(side="left", padx=10)

        exit_btn = tk.Button(ov, text="✕ EXIT FULLSCREEN  (ESC)", fg="#F87171", bg="#1E293B",
                             activebackground="#DC2626", activeforeground="#FFFFFF",
                             relief="flat", font=(FONT_FAMILY, 9, "bold"),
                             command=self._close_fullscreen_cam)
        exit_btn.pack(side="right", padx=12, pady=4)

        self._cam_fs_lbl = tk.Label(win, bg="#000000")
        self._cam_fs_lbl.pack(fill="both", expand=True)
        self._cam_fs_lbl.bind("<Double-Button-1>", lambda e: self._close_fullscreen_cam())
        self._cam_fs_win = win

    def _close_fullscreen_cam(self):
        if hasattr(self, "_cam_fs_win") and self._cam_fs_win is not None:
            try:
                self._cam_fs_win.destroy()
            except Exception:
                pass
            self._cam_fs_win = None
            self._cam_fs_lbl = None
            self._fs_fps_lbl = None
            self._fs_mode_lbl = None
            self._fs_hotspot_lbl = None

    def _cam_loop(self):
        self._cam_running = True
        try:
            import cv2
            import numpy as np
            from PIL import Image, ImageTk
        except ImportError:
            self._cam_lbl.configure(
                text="Please install OpenCV & Pillow:\npip install opencv-python pillow numpy",
                text_color=BF["yellow"])
            return

        mode = self.cam_mode.get() if hasattr(self, "cam_mode") else "Dual (Split)"
        eo_url = self.cam_url.get().strip()
        ir_url = self.cam_thermal_url.get().strip() if hasattr(self, "cam_thermal_url") else ""

        needs_eo = mode in ["Dual (Split)", "EO (Daylight)", "PiP (Picture-in-Pic)"]
        needs_ir = mode in ["Dual (Split)", "IR (Thermal)", "PiP (Picture-in-Pic)"]

        self._grabber_rgb = None
        self._grabber_ir = None

        if needs_eo and eo_url:
            self.log_q.put(("INFO", f"Connecting EO Daylight stream -> {eo_url}"))
            self._grabber_rgb = StreamGrabber(eo_url)

        if needs_ir and ir_url:
            self.log_q.put(("INFO", f"Connecting IR Thermal stream -> {ir_url}"))
            self._grabber_ir = StreamGrabber(ir_url)

        rgb_ok = self._grabber_rgb and self._grabber_rgb.connected
        ir_ok = self._grabber_ir and self._grabber_ir.connected

        if not rgb_ok and not ir_ok:
            fail_msg = f"Failed to connect streams.\nEO: {eo_url}\nIR: {ir_url}"
            self.log_q.put(("ERR", "Camera streams unreachable. Check IP & H16 network."))
            self._cam_lbl.configure(text=fail_msg, text_color=BF["red"])
            return

        self.log_q.put(("OK", f"DRISHYA V1 Video active [Mode: {mode}]"))
        self.cam_det_lbl.configure(text="AI DETECTION: ACTIVE", text_color=BF["green"])
        if ir_ok:
            self.cam_thermal_stat_lbl.configure(
                text=f"IR SENSOR: ONLINE [{self.cam_palette.get().upper()}]",
                text_color=BF["yellow"])

        fc = 0
        t0 = time.time()

        while self._cam_running and self._running:
            current_mode = self.cam_mode.get()
            cur_palette = self.cam_palette.get()
            do_hotspot = self.cam_hotspot_var.get()

            # Read latest frames
            frame_rgb = None
            if self._grabber_rgb:
                _, frame_rgb = self._grabber_rgb.read_latest()

            frame_ir = None
            if self._grabber_ir:
                _, frame_ir = self._grabber_ir.read_latest()

            # If mode needs thermal but no IR stream connected, fallback to RGB
            if current_mode == "IR (Thermal)" and frame_ir is None and frame_rgb is not None:
                frame_ir = frame_rgb

            if frame_rgb is None and frame_ir is None:
                time.sleep(0.005)
                continue

            try:
                # Target sizing
                is_fs = (hasattr(self, "_cam_fs_win") and self._cam_fs_win is not None and self._cam_fs_win.winfo_exists())
                if is_fs:
                    target_w = max(400, self._cam_fs_win.winfo_width())
                    target_h = max(300, self._cam_fs_win.winfo_height() - 36)
                else:
                    vw = self._cam_v_frame.winfo_width() if hasattr(self, "_cam_v_frame") else 0
                    vh = self._cam_v_frame.winfo_height() if hasattr(self, "_cam_v_frame") else 0
                    target_w = max(320, vw - 8) if vw > 100 else 1024
                    target_h = max(240, vh - 8) if vh > 100 else 640

                output_frame = None
                hotspot_val = 0

                # ── MODE 1: EO Daylight Only ─────────────────────────────────
                if current_mode == "EO (Daylight)" and frame_rgb is not None:
                    h, w = frame_rgb.shape[:2]
                    scale = min(target_w / w, target_h / h)
                    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
                    output_frame = cv2.resize(frame_rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
                    cv2.putText(output_frame, "EO DAYLIGHT", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

                # ── MODE 2: IR Thermal Only ──────────────────────────────────
                elif current_mode == "IR (Thermal)" and frame_ir is not None:
                    proc_ir, hotspot_val, _ = process_thermal_frame(frame_ir, cur_palette, do_hotspot)
                    h, w = proc_ir.shape[:2]
                    scale = min(target_w / w, target_h / h)
                    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
                    output_frame = cv2.resize(proc_ir, (nw, nh), interpolation=cv2.INTER_LINEAR)
                    cv2.putText(output_frame, f"IR THERMAL [{cur_palette.upper()}]", (12, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2, cv2.LINE_AA)

                # ── MODE 3: Dual (Side-by-Side Split) ────────────────────────
                elif current_mode == "Dual (Split)":
                    half_w = (target_w - 6) // 2

                    # Left side: EO
                    if frame_rgb is not None:
                        h, w = frame_rgb.shape[:2]
                        s_rgb = min(half_w / w, target_h / h)
                        nw_rgb, nh_rgb = max(1, int(w * s_rgb)), max(1, int(h * s_rgb))
                        left_img = cv2.resize(frame_rgb, (nw_rgb, nh_rgb), interpolation=cv2.INTER_LINEAR)
                        canvas_left = np.zeros((target_h, half_w, 3), dtype=np.uint8)
                        y_off = (target_h - nh_rgb) // 2
                        x_off = (half_w - nw_rgb) // 2
                        canvas_left[y_off:y_off+nh_rgb, x_off:x_off+nw_rgb] = left_img
                        cv2.putText(canvas_left, "● EO DAYLIGHT", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
                    else:
                        canvas_left = np.zeros((target_h, half_w, 3), dtype=np.uint8)
                        cv2.putText(canvas_left, "[ AWAITING EO STREAM ]", (half_w // 4, target_h // 2),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1, cv2.LINE_AA)

                    # Right side: IR Thermal
                    if frame_ir is not None:
                        proc_ir, hotspot_val, _ = process_thermal_frame(frame_ir, cur_palette, do_hotspot)
                        h, w = proc_ir.shape[:2]
                        s_ir = min(half_w / w, target_h / h)
                        nw_ir, nh_ir = max(1, int(w * s_ir)), max(1, int(h * s_ir))
                        right_img = cv2.resize(proc_ir, (nw_ir, nh_ir), interpolation=cv2.INTER_LINEAR)
                        canvas_right = np.zeros((target_h, half_w, 3), dtype=np.uint8)
                        y_off = (target_h - nh_ir) // 2
                        x_off = (half_w - nw_ir) // 2
                        canvas_right[y_off:y_off+nh_ir, x_off:x_off+nw_ir] = right_img
                        cv2.putText(canvas_right, f"● IR THERMAL [{cur_palette.upper()}]", (12, 26),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)
                    else:
                        canvas_right = np.zeros((target_h, half_w, 3), dtype=np.uint8)
                        cv2.putText(canvas_right, "[ AWAITING IR STREAM ]", (half_w // 4, target_h // 2),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1, cv2.LINE_AA)

                    divider = np.zeros((target_h, 4, 3), dtype=np.uint8)
                    divider[:, :] = [255, 229, 0]  # Electric cyan BGR
                    output_frame = np.hstack([canvas_left, divider, canvas_right])

                # ── MODE 4: PiP (Picture-in-Picture) ─────────────────────────
                elif current_mode == "PiP (Picture-in-Pic)":
                    base_frame = frame_rgb if frame_rgb is not None else frame_ir
                    h, w = base_frame.shape[:2]
                    scale = min(target_w / w, target_h / h)
                    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
                    output_frame = cv2.resize(base_frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
                    cv2.putText(output_frame, "EO MAIN", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

                    if frame_ir is not None:
                        proc_ir, hotspot_val, _ = process_thermal_frame(frame_ir, cur_palette, do_hotspot)
                        pip_w = max(120, int(nw * 0.28))
                        pip_h = max(80, int(nh * 0.28))
                        pip_img = cv2.resize(proc_ir, (pip_w, pip_h), interpolation=cv2.INTER_LINEAR)
                        cv2.rectangle(pip_img, (0, 0), (pip_w-1, pip_h-1), (0, 229, 255), 2)
                        cv2.putText(pip_img, f"IR [{cur_palette[:4]}]", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)

                        px = nw - pip_w - 12
                        py = 12
                        if px > 0 and py + pip_h < nh:
                            output_frame[py:py+pip_h, px:px+pip_w] = pip_img

                # Fallback
                if output_frame is None:
                    src_f = frame_rgb if frame_rgb is not None else frame_ir
                    h, w = src_f.shape[:2]
                    scale = min(target_w / w, target_h / h)
                    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
                    output_frame = cv2.resize(src_f, (nw, nh), interpolation=cv2.INTER_LINEAR)

                # Convert to RGB & update GUI
                rgb = cv2.cvtColor(output_frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(rgb)
                ph = ImageTk.PhotoImage(img)

                if is_fs and hasattr(self, "_cam_fs_lbl") and self._cam_fs_lbl is not None:
                    self._cam_fs_lbl.configure(image=ph, text="")
                    self._cam_fs_lbl._ph = ph
                else:
                    self._cam_lbl.configure(image=ph, text="")
                    self._cam_lbl._ph = ph

                if hotspot_val > 0:
                    hs_text = f"🔥 HOTSPOT: {hotspot_val}"
                    self.hotspot_lbl.configure(text=hs_text, text_color=BF["yellow"])
                    if is_fs and hasattr(self, "_fs_hotspot_lbl") and self._fs_hotspot_lbl is not None:
                        self._fs_hotspot_lbl.configure(text=hs_text)

            except Exception:
                pass

            fc += 1
            if fc % 20 == 0:
                fps = fc / (time.time() - t0 + 0.001)
                fps_txt = f"FPS: {fps:.1f}"
                try:
                    self.cam_fps_lbl.configure(text=fps_txt, text_color=BF["green"])
                    if is_fs and hasattr(self, "_fs_fps_lbl") and self._fs_fps_lbl is not None:
                        self._fs_fps_lbl.configure(text=fps_txt)
                        if hasattr(self, "_fs_mode_lbl") and self._fs_mode_lbl is not None:
                            self._fs_mode_lbl.configure(text=f"MODE: {self.cam_mode.get()}")
                except Exception:
                    pass

        if self._grabber_rgb:
            self._grabber_rgb.release()
            self._grabber_rgb = None
        if self._grabber_ir:
            self._grabber_ir.release()
            self._grabber_ir = None
        self.log_q.put(("INFO", "Camera streams stopped"))

    def _on_close(self):
        self._running = False
        self._stop_lidar_stream()
        self._stop_cam()
        self._close_fullscreen_cam()
        self.mav_mgr.disconnect()
        self.destroy()


# =========================================================================
#  ENTRY POINT
# =========================================================================
if __name__ == "__main__":
    import sys
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    app = DrishyaGCS()
    app.mainloop()
