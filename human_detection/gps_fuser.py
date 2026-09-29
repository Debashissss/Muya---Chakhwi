"""
gps_fuser.py — Maps pixel (u, v) coordinates to GPS (lat, lon) using
drone telemetry received over MAVLink.

Theory:
  Given drone GPS position, altitude AGL, camera FOV, and heading,
  compute the ground footprint of each pixel and derive the world
  coordinate of a detection bounding box centre.
"""
import math
import threading
import logging
from dataclasses import dataclass, field
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

try:
    from pymavlink import mavutil
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False
    logger.warning("pymavlink not installed — GPS tagging disabled.")


@dataclass
class DroneState:
    lat: float = 0.0          # degrees
    lon: float = 0.0          # degrees
    alt_agl: float = 0.0      # metres above ground
    heading: float = 0.0      # degrees (0 = North)
    roll: float = 0.0         # radians
    pitch: float = 0.0        # radians
    last_updated: float = 0.0


class GPSFuser:
    """
    Subscribes to MAVLink telemetry and converts pixel positions to GPS coords.
    """

    # Default: wide-angle RGB camera FOV (adjust to your lens)
    H_FOV_DEG: float = 82.0
    V_FOV_DEG: float = 62.0

    def __init__(self, port: Optional[str] = None, baud: int = 57600,
                 frame_w: int = 640, frame_h: int = 480):
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.state = DroneState()
        self._lock = threading.Lock()
        self._running = False
        self._conn = None

        if port and HAS_MAVLINK:
            self._start_mavlink(port, baud)

    # ------------------------------------------------------------------ MAVLink
    def _start_mavlink(self, port: str, baud: int) -> None:
        try:
            self._conn = mavutil.mavlink_connection(port, baud=baud)
            self._conn.wait_heartbeat(timeout=5)
            logger.info("MAVLink connected on %s", port)
            self._running = True
            t = threading.Thread(target=self._listen, daemon=True)
            t.start()
        except Exception as exc:
            logger.error("MAVLink connection failed: %s", exc)

    def _listen(self) -> None:
        while self._running:
            try:
                msg = self._conn.recv_match(
                    type=["GLOBAL_POSITION_INT", "ATTITUDE"],
                    blocking=True, timeout=1.0
                )
                if msg is None:
                    continue
                mtype = msg.get_type()
                with self._lock:
                    if mtype == "GLOBAL_POSITION_INT":
                        self.state.lat = msg.lat / 1e7
                        self.state.lon = msg.lon / 1e7
                        self.state.alt_agl = msg.relative_alt / 1000.0
                        import time; self.state.last_updated = time.time()
                    elif mtype == "ATTITUDE":
                        self.state.roll    = msg.roll
                        self.state.pitch   = msg.pitch
                        self.state.heading = math.degrees(msg.yaw) % 360
            except Exception as exc:
                logger.debug("MAVLink listen error: %s", exc)

    def stop(self) -> None:
        self._running = False
        if self._conn:
            self._conn.close()

    # ------------------------------------------------------------------ pixel→GPS
    def pixel_to_gps(self, cx: float, cy: float) -> Tuple[float, float]:
        """
        Convert pixel centre (cx, cy) of a bounding box to (lat, lon).
        Returns (0.0, 0.0) if no valid GPS fix.
        """
        with self._lock:
            lat0 = self.state.lat
            lon0 = self.state.lon
            alt  = self.state.alt_agl
            hdg  = self.state.heading

        if alt < 0.5 or (lat0 == 0.0 and lon0 == 0.0):
            return 0.0, 0.0

        # Normalised pixel offsets from frame centre (range −0.5 to +0.5)
        nx = (cx / self.frame_w) - 0.5
        ny = (cy / self.frame_h) - 0.5

        # Angular offsets
        ang_x = nx * math.radians(self.H_FOV_DEG)
        ang_y = ny * math.radians(self.V_FOV_DEG)

        # Ground offsets (metres) — simple pinhole, no lens distortion
        dx = alt * math.tan(ang_x)
        dy = alt * math.tan(ang_y)

        # Rotate by drone heading
        hdg_r = math.radians(hdg)
        north = -dy * math.cos(hdg_r) + dx * math.sin(hdg_r)
        east  =  dy * math.sin(hdg_r) + dx * math.cos(hdg_r)

        # Convert metre offsets to degrees
        d_lat = north / 111_139.0
        d_lon = east  / (111_139.0 * math.cos(math.radians(lat0)))

        return round(lat0 + d_lat, 8), round(lon0 + d_lon, 8)

    def get_drone_gps(self) -> Tuple[float, float, float]:
        with self._lock:
            return self.state.lat, self.state.lon, self.state.alt_agl
