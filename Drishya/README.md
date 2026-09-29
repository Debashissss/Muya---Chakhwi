# DRISHYA V1 GCS
### Disaster Response Intelligence & Dual EO/IR Mission Control System
**DRISHYA V1 GCS.**

---

## Hardware Requirements
| Component | Spec |
|-----------|------|
| Flight Controller | CrossFlight FC (ArduPilot-based) |
| Telemetry | 2× 433 MHz SiK Radio modules |
| Battery | 3S LiPo |
| GPS | TS100 |
| Motors | BLDC + 10" propellers |

---

## Installation

```bash
pip install -r requirements.txt
```

> On Linux/Raspberry Pi, also run:
> `sudo usermod -a -G dialout $USER`
> (then log out and back in for serial port access)

---

## Running

```bash
python main.py
```

---

## Connecting Drones

1. Plug both 433 MHz USB ground radios into your PC
2. Click **CONNECT D1** (Leader) → enter COM port (e.g. `COM3` on Windows, `/dev/ttyUSB0` on Linux)
3. Click **CONNECT D2** (Follower) → enter second COM port
4. Set baud rate to **57600** (default for SiK radios)
5. Check **Simulation Mode** if no hardware is connected

### Important: ArduPilot Configuration
Before connecting, set unique SYSID on each drone via Mission Planner (one-time):
- Drone 1 (Leader): `SYSID_THISMAV = 1`
- Drone 2 (Follower): `SYSID_THISMAV = 2`

---

## Features

The connection window automatically scans serial devices and shows both the COM
port and its operating-system description. Use **REFRESH** after plugging in a
radio. Once connected, the active port is displayed in the main dashboard.

- **Dual drone telemetry** — GPS, altitude, battery, heading, speed, EKF status
- **Live swarm map** — auto-centred GPS tracks, heading markers and altitude
- **Real OpenStreetMap** — interactive pan/zoom map using actual MAVLink GPS coordinates
- **Satellite view** — switch between road tiles and Esri World Imagery
- **Level status** — clear LEVEL/TILTED indication from live roll and pitch
- **Attitude indicator** — visual roll/pitch display per drone
- **Compass widget** — live heading display
- **Mini graphs** — altitude and battery history plots
- **Per-drone commands** — ARM, DISARM, TAKEOFF, RTL, LAND, mode change
- **Swarm control bar** — ARM ALL, SWARM TAKEOFF (staged), RTL ALL, LAND ALL, KILL ALL
- **Follower offset** — configurable N/E offset for formation flying
- **System log console** — timestamped command and status log
- **Simulation mode** — full UI demo without hardware

Simulation starts parked, disarmed and stationary. A simulated drone moves only
after explicit ARM and TAKEOFF commands. Real-aircraft arming uses the normal
ArduPilot pre-arm checks; the application does not bypass them.

---

## Swarm Takeoff Sequence
1. Both drones armed via **ARM ALL**
2. **SWARM TKO** → enter target altitude
3. Leader takes off first to full altitude
4. Follower takes off 2 seconds later to `altitude - 3m` (separation buffer)
5. Follower tracks leader GPS position + configured N/E offset

---

## File Structure
```
gcs/
├── main.py           ← single-file application
├── requirements.txt  ← Python dependencies
└── README.md         ← this file
```

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| Serial port not found | Check device manager / `ls /dev/ttyUSB*` |
| Drone won't arm | Check pre-arm messages in log console |
| GPS no fix | Wait outdoors for 3D fix (14+ satellites) |
| Connection timeout | Verify baud rate matches radio config (57600) |
| dronekit install fails | `pip install dronekit --break-system-packages` |
