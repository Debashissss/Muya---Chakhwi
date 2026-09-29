"""
db.py — SQLite detection database for DRISHYA human detection module.
"""
import sqlite3
import json
import logging
from datetime import datetime
from pathlib import Path

DB_PATH = Path(__file__).parent / "human_detections.db"
logger = logging.getLogger(__name__)


def init_db() -> sqlite3.Connection:
    """Initialise (or open) the SQLite database and create tables."""
    conn = sqlite3.connect(str(DB_PATH))
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("""
        CREATE TABLE IF NOT EXISTS detections (
            id          INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id  TEXT    NOT NULL,
            timestamp   TEXT    NOT NULL,
            frame_id    INTEGER NOT NULL,
            track_id    INTEGER NOT NULL,
            confidence  REAL    NOT NULL,
            priority    TEXT    NOT NULL,
            bbox_x1     REAL,
            bbox_y1     REAL,
            bbox_x2     REAL,
            bbox_y2     REAL,
            center_x    REAL,
            center_y    REAL,
            lat         REAL,
            lon         REAL,
            altitude_m  REAL,
            thermal_conf REAL,
            source_type TEXT,
            snapshot_path TEXT,
            extra_json  TEXT
        )
    """)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS sessions (
            session_id  TEXT PRIMARY KEY,
            started_at  TEXT NOT NULL,
            source      TEXT,
            model       TEXT,
            device      TEXT,
            notes       TEXT
        )
    """)
    conn.commit()
    logger.info("Database ready: %s", DB_PATH)
    return conn


def log_session(conn: sqlite3.Connection, session_id: str, source: str,
                model: str, device: str) -> None:
    conn.execute(
        "INSERT OR IGNORE INTO sessions VALUES (?,?,?,?,?,?)",
        (session_id, datetime.utcnow().isoformat(), source, model, device, ""),
    )
    conn.commit()


def log_detection(conn: sqlite3.Connection, **kwargs) -> int:
    """Insert a detection row and return its row id."""
    cols = [
        "session_id", "timestamp", "frame_id", "track_id", "confidence",
        "priority", "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2",
        "center_x", "center_y", "lat", "lon", "altitude_m",
        "thermal_conf", "source_type", "snapshot_path", "extra_json",
    ]
    vals = [kwargs.get(c) for c in cols]
    if isinstance(vals[-1], dict):
        vals[-1] = json.dumps(vals[-1])
    placeholders = ",".join(["?"] * len(cols))
    cur = conn.execute(
        f"INSERT INTO detections ({','.join(cols)}) VALUES ({placeholders})",
        vals,
    )
    conn.commit()
    return cur.lastrowid


def get_recent(conn: sqlite3.Connection, n: int = 20) -> list:
    cur = conn.execute(
        "SELECT * FROM detections ORDER BY id DESC LIMIT ?", (n,)
    )
    return cur.fetchall()
