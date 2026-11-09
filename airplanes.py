"""
airplanes.py — Dump1090 daemon controller & ADS-B decoder for AERØS.

Spawns/manages a background dump1090 process (or connects to one already
running), reads its raw-hex TCP stream, decodes Mode S / ADS-B frames with
pyModeS, and maintains a rolling table of visible aircraft with computed
range/bearing from the operator's GPS position.

This module only receives broadcast ADS-B signals (1090MHz), which every
aircraft transmits openly and continuously. Nothing here transmits.

Changes against the original controller/tracker (kept compatible):
  * pyModeS is probed at import. pyModeS 3.x removed the v2 API used here and
    raises on first use; the original would have swallowed that in its blanket
    `except` and silently shown zero aircraft. Now it is reported clearly and
    AERØS falls back to dump1090's own JSON / SBS output.
  * Frames must pass the Mode S CRC (pms.crc == 0) before they are trusted.
  * The aircraft table and CPR buffer are pruned (the original grew forever).
  * dump1090's stderr is drained and hardware errors are explained, and
    the launcher tries several flag sets so dump1090-fa, readsb,
    dump1090-mutability and classic dump1090 all work.
  * The raw stream carries no signal level, so RSSI is merged in from
    dump1090's aircraft.json when the decoder writes one (RssiSource).
"""

from __future__ import annotations

import collections
import json
import math
import os
import re
import shutil
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

PYMODES_ERROR = ""
try:
    import pyModeS as pms

    _ = pms.adsb.icao  # pyModeS >= 3 raises here (v2 API removed); v2.x passes
    HAVE_PYMODES = True
except Exception:  # not installed (ImportError) or pyModeS 3.x (V2APIRemovedError, an ImportError subclass)
    import importlib.util as _ilu

    pms = None  # type: ignore[assignment]
    HAVE_PYMODES = False
    if _ilu.find_spec("pyModeS") is None:
        PYMODES_ERROR = "pyModeS not installed (pip install 'pyModeS<3')"
    else:
        PYMODES_ERROR = "pyModeS 3.x installed but AERØS needs the v2 API (pip install 'pyModeS<3')"

DUMP1090_HOST = "127.0.0.1"
DUMP1090_RAW_PORT = 30002  # --net --raw output port
DUMP1090_SBS_PORT = 30003

HEX6 = re.compile(r"[0-9A-F]{6}")
COMMON_JSON_DIRS = ["/run/dump1090-fa", "/run/readsb", "/run/dump1090-mutability", "/run/dump1090",
                    "/var/run/dump1090-fa", "/var/run/readsb"]
EMERGENCY_SQUAWKS = {"7500": "HIJACK", "7600": "RADIO FAILURE", "7700": "EMERGENCY"}

# dump1090 launch ladder, tried in order until one comes up.
#   raw  -> TCP :30002 hex frames decoded by pyModeS (+ aircraft.json for RSSI if the decoder writes it)
#   json -> decoder writes aircraft.json (gives RSSI, decoder does the decoding)
#   sbs  -> TCP :30003 BaseStation feed (no RSSI)
LADDERS = [
    ("raw", ["--device-type", "rtlsdr", "--net", "--raw", "--quiet", "--write-json", "{json}", "--write-json-every", "1"]),
    ("raw", ["--net", "--raw", "--quiet", "--write-json", "{json}", "--write-json-every", "1"]),
    ("raw", ["--device-type", "rtlsdr", "--net", "--raw", "--quiet"]),
    ("raw", ["--net", "--raw", "--quiet"]),
    ("json", ["--device-type", "rtlsdr", "--quiet", "--write-json", "{json}", "--write-json-every", "1"]),
    ("json", ["--quiet", "--write-json", "{json}", "--write-json-every", "1"]),
    ("sbs", ["--net", "--quiet"]),
    ("sbs", ["--net"]),
]
HW_ERR = re.compile(
    r"no supported|no rtl|usb_open error|failed to open|unable to open|cannot open|"
    r"kernel driver is active|usb_claim_interface|error opening|resource busy|"
    r"no devices? found|rtlsdr_open|permission denied",
    re.I,
)


def to_float(value) -> Optional[float]:
    """float or None; NaN/inf, bools and junk become None."""
    try:
        if value is None or value == "" or isinstance(value, bool):
            return None
        out = float(value)
        return out if math.isfinite(out) else None
    except (TypeError, ValueError, OverflowError):
        return None


def to_int(value) -> Optional[int]:
    out = to_float(value)
    return None if out is None else int(out)


def explain_sdr_error(text: str) -> str:
    t = text.lower()
    if "kernel driver is active" in t or "usb_claim_interface" in t or "resource busy" in t:
        return "dongle busy: DVB driver loaded or another SDR app running (run --setup, then replug)"
    if "no supported" in t or "no rtl" in t or "no device" in t:
        return "no RTL-SDR dongle found: check USB connection / power / antenna"
    if "permission" in t or "usb_open error -3" in t:
        return "USB permission denied: add user to plugdev (run --setup, re-login) or use sudo"
    last = [ln for ln in text.splitlines() if ln.strip()]
    return last[-1][:110] if last else "decoder exited immediately"


# ─────────────────────────────────────────────────────────────────────────────
# Aircraft model
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Aircraft:
    icao: str
    callsign: str = ""
    altitude_ft: Optional[float] = None
    speed_kt: Optional[float] = None
    heading_deg: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    last_seen: float = field(default_factory=time.time)
    # AERØS additions
    squawk: str = ""
    on_ground: bool = False
    rssi: Optional[float] = None  # dBFS (relative), from aircraft.json only
    messages: int = 0
    first_seen: float = 0.0
    fresh: bool = False  # received something new since the logger last looked

    def distance_bearing_from(self, ref_lat: float, ref_lon: float):
        """Great-circle distance (nm) and initial bearing (deg) from ref point."""
        if self.lat is None or self.lon is None:
            return None, None
        R_NM = 3440.065
        lat1, lon1, lat2, lon2 = map(
            math.radians, (ref_lat, ref_lon, self.lat, self.lon)
        )
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.asin(min(1, math.sqrt(a)))
        distance = R_NM * c
        bearing = math.degrees(
            math.atan2(
                math.sin(dlon) * math.cos(lat2),
                math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon),
            )
        ) % 360
        return distance, bearing


class AircraftTable:
    """Thread-safe rolling table shared by every decoder backend (raw / json / sbs)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ac: Dict[str, Aircraft] = {}
        self.unique = 0
        self.rejected_positions = 0
        self._ref: Optional[tuple] = None
        self._max_km = 0.0

    @staticmethod
    def _km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        p1, p2 = math.radians(lat1), math.radians(lat2)
        a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
        return 12742.0176 * math.asin(min(1.0, math.sqrt(a)))

    def set_reference(self, lat: Optional[float], lon: Optional[float], max_km: float) -> None:
        """Receiver position used to reject impossible aircraft positions (a CPR decode across a zone
        boundary or a stale even/odd pair can land thousands of km away). None disables the filter."""
        with self._lock:
            if lat is None or lon is None or max_km <= 0:
                self._ref = None
                return
            self._ref, self._max_km = (lat, lon), max_km
            for ac in self._ac.values():  # purge anything accepted before the receiver position was known
                if ac.lat is not None and ac.lon is not None and self._km(lat, lon, ac.lat, ac.lon) > max_km:
                    ac.lat = ac.lon = None
                    self.rejected_positions += 1

    def upsert(self, icao: str, seen_at: float, fresh: bool, **fields) -> None:
        with self._lock:
            if self._ref is not None and fields.get("lat") is not None and fields.get("lon") is not None:
                if self._km(self._ref[0], self._ref[1], fields["lat"], fields["lon"]) > self._max_km:
                    fields = dict(fields, lat=None, lon=None)
                    self.rejected_positions += 1
            ac = self._ac.get(icao)
            if ac is None:
                ac = Aircraft(icao=icao, first_seen=seen_at, last_seen=seen_at)
                self._ac[icao] = ac
                self.unique += 1
            for k, v in fields.items():
                if v is None or (k in ("callsign", "squawk") and not v):
                    continue
                setattr(ac, k, v)
            if seen_at > ac.last_seen:
                ac.last_seen = seen_at
            if fresh:
                ac.fresh = True

    def update_existing(self, icao: str, **fields) -> None:
        """Set fields on an aircraft that is already known (never creates one, never touches last_seen)."""
        with self._lock:
            ac = self._ac.get(icao)
            if ac is None:
                return
            for k, v in fields.items():
                if v is not None:
                    setattr(ac, k, v)

    def mark_fresh(self, icao: str) -> None:
        """Offer an aircraft to the logger again on the next tick."""
        with self._lock:
            ac = self._ac.get(icao)
            if ac is not None:
                ac.fresh = True

    def drain_fresh(self) -> List[Aircraft]:
        with self._lock:
            out = [Aircraft(**{**a.__dict__}) for a in self._ac.values() if a.fresh]
            for a in self._ac.values():
                a.fresh = False
        return out

    def snapshot(self, max_age_s: float = 60.0) -> List[Aircraft]:
        now = time.time()
        with self._lock:
            for k in [k for k, a in self._ac.items() if now - a.last_seen > max(max_age_s * 5, 300)]:
                del self._ac[k]
            fresh = [Aircraft(**{**a.__dict__}) for a in self._ac.values() if now - a.last_seen <= max_age_s]
        return sorted(fresh, key=lambda a: a.last_seen, reverse=True)


# ─────────────────────────────────────────────────────────────────────────────
# dump1090 process control
# ─────────────────────────────────────────────────────────────────────────────
class Dump1090Controller:
    """Starts dump1090 as a background subprocess if not already running."""
    BINARIES = ["dump1090-aeros", "dump1090-fa", "readsb", "dump1090-mutability", "dump1090"]

    def __init__(self, binary: Optional[str] = None, json_dir: str = "", extra_args: Optional[List[str]] = None,
                 host: str = DUMP1090_HOST, raw_port: int = DUMP1090_RAW_PORT,
                 use_raw: bool = True, stop_evt: Optional[threading.Event] = None,
                 sbs_port: int = DUMP1090_SBS_PORT):
        self.binary = binary or self.find_binary()
        self.json_dir = json_dir
        self.extra_args = list(extra_args or [])
        self.host, self.raw_port, self.sbs_port = host, raw_port, sbs_port
        self.external_json_dir = ""
        self.use_raw = bool(use_raw and HAVE_PYMODES)
        self.stop_evt = stop_evt or threading.Event()
        self.process: Optional[subprocess.Popen] = None
        self.owns_process = False
        self.last_error = ""
        self.launched_binary = ""
        self.mode = ""
        self.argv: List[str] = []
        self.stderr_tail: collections.deque = collections.deque(maxlen=40)

    @classmethod
    def find_binary(cls, explicit: Optional[str] = None) -> Optional[str]:
        candidates = [explicit, os.environ.get("AEROS_DUMP1090"),
                      "/usr/local/bin/dump1090-aeros",
                      os.path.expanduser("~/.local/share/aeros/bin/dump1090")]
        for c in candidates:
            if c and os.path.isfile(c) and os.access(c, os.X_OK):
                return c
        for name in cls.BINARIES:
            p = shutil.which(name)
            if p:
                return p
        return None

    def is_port_open(self, port: Optional[int] = None) -> bool:
        try:
            with socket.create_connection((self.host, port or self.raw_port), timeout=0.5):
                return True
        except (OSError, socket.timeout):
            return False

    @staticmethod
    def find_external_json_dir() -> str:
        """A running decoder's aircraft.json that was updated in the last 10 s (e.g. the dump1090-fa service)."""
        for d in COMMON_JSON_DIRS:
            try:
                if time.time() - os.stat(os.path.join(d, "aircraft.json")).st_mtime < 10:
                    return d
            except OSError:
                continue
        return ""

    def ensure_running(self) -> bool:
        """Adopt a decoder that is already running (raw :30002, then aircraft.json, then SBS :30003),
        otherwise start our own. An existing service holds the dongle, so spawning a second one would fail."""
        if self.use_raw and self.is_port_open():
            self.mode = "raw"
            return True
        self.external_json_dir = self.find_external_json_dir()
        if self.external_json_dir:
            self.mode = "json"
            return True
        if self.is_port_open(self.sbs_port):
            self.mode = "sbs"
            return True
        return self.spawn()

    def external_alive(self) -> bool:
        """Is the adopted (not spawned) decoder still there?"""
        if self.mode == "raw":
            return self.is_port_open()
        if self.mode == "json":
            return bool(self.external_json_dir) and self.find_external_json_dir() == self.external_json_dir
        return self.is_port_open(self.sbs_port)

    def alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def _drain(self, proc: subprocess.Popen) -> None:
        try:
            for line in proc.stderr:  # type: ignore[union-attr]
                line = line.strip()
                if line:
                    self.stderr_tail.append(line)
        except (ValueError, OSError):
            pass

    def _wait_ready(self, proc: subprocess.Popen, mode: str) -> bool:
        start = time.monotonic()
        while time.monotonic() - start < 4.0:
            if self.stop_evt.wait(0.2):
                return False
            if proc.poll() is not None:
                return False
            if mode == "raw" and self.is_port_open():
                return True
            if mode != "raw" and time.monotonic() - start >= 3.0:
                return True
        return False

    @staticmethod
    def _kill(proc: subprocess.Popen) -> None:
        if proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except (subprocess.TimeoutExpired, OSError):
                try:
                    proc.kill()
                except OSError:
                    pass

    def spawn(self) -> bool:
        if not self.binary:
            self.last_error = "No dump1090 binary found on PATH."
            return False
        for mode, tmpl in LADDERS:
            if mode == "raw" and not self.use_raw:
                continue
            argv = [self.binary] + [a.replace("{json}", self.json_dir) for a in tmpl] + self.extra_args
            self.stderr_tail.clear()
            try:
                proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                        stderr=subprocess.PIPE, start_new_session=True,
                                        text=True, errors="replace")
            except OSError as exc:
                self.last_error = f"cannot execute {self.binary}: {exc}"
                return False
            reader = threading.Thread(target=self._drain, args=(proc,), daemon=True)
            reader.start()
            if self._wait_ready(proc, mode):
                self.process, self.owns_process = proc, True
                self.launched_binary, self.mode, self.argv = self.binary, mode, argv
                self.last_error = ""
                return True
            if self.stop_evt.is_set():
                self._kill(proc)
                return False
            self._kill(proc)
            reader.join(timeout=1.0)
            err = "\n".join(self.stderr_tail)
            self.last_error = err or f"exit code {proc.returncode}"
            if HW_ERR.search(err):  # hardware problem: other flag sets will not help
                return False
        return False

    def shutdown(self) -> None:
        if self.owns_process and self.process is not None:
            self._kill(self.process)
            self.process = None
            self.owns_process = False


# ─────────────────────────────────────────────────────────────────────────────
# Backend 1: raw hex stream decoded with pyModeS
# ─────────────────────────────────────────────────────────────────────────────
class AircraftTracker(threading.Thread):
    """Connects to dump1090 raw TCP stream and decodes frames with pyModeS."""

    def __init__(self, host: str = DUMP1090_HOST, port: int = DUMP1090_RAW_PORT,
                 table: Optional[AircraftTable] = None, stats=None,
                 stop_evt: Optional[threading.Event] = None):
        super().__init__(daemon=True, name="raw-feed")
        self.host = host
        self.port = port
        self.table = table if table is not None else AircraftTable()
        self.stats = stats
        self.stop_evt = stop_evt or threading.Event()
        self.halt = threading.Event()
        self.connected = False
        self.last_error = ""
        self._cpr_buffer: Dict[str, dict] = {}

    def _done(self) -> bool:
        return self.stop_evt.is_set() or self.halt.is_set()

    def start_tracking(self) -> None:  # alias kept from the original API
        self.start()

    def stop(self) -> None:
        self.halt.set()
        if self.is_alive():
            self.join(timeout=2)

    def _set_feed(self, ok: bool) -> None:
        self.connected = ok
        if self.stats is not None:
            self.stats.feed_kind = "raw"
            self.stats.feed_ok = ok

    def run(self) -> None:
        while not self._done():
            try:
                with socket.create_connection((self.host, self.port), timeout=5) as sock:
                    sock.settimeout(1.0)
                    self._set_feed(True)
                    buf = b""
                    while not self._done():
                        try:
                            chunk = sock.recv(4096)
                        except socket.timeout:
                            continue
                        if not chunk:
                            break
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            self._handle_raw_line(line.strip())
            except Exception as exc:  # connection problems and anything unexpected: reconnect
                self.last_error = str(exc)
            self._set_feed(False)
            self.halt.wait(3)

    def _handle_raw_line(self, line: bytes) -> None:
        if not HAVE_PYMODES or not line.startswith(b"*") or not line.endswith(b";"):
            return
        msg = line[1:-1].decode("ascii", errors="ignore")
        if len(msg) < 14:
            return
        try:
            if len(msg) != 28 or pms.df(msg) != 17:
                return
            if pms.crc(msg) != 0:  # corrupted frame
                return
            icao = str(pms.adsb.icao(msg) or "").upper()
            if not HEX6.fullmatch(icao):
                return
            now = time.time()
            fields: dict = {}
            tc = pms.adsb.typecode(msg)
            if tc is not None and 1 <= tc <= 4:
                callsign = pms.adsb.callsign(msg)
                if callsign:
                    fields["callsign"] = callsign.strip("_").strip()
            elif tc is not None and (9 <= tc <= 18 or 20 <= tc <= 22):
                alt = pms.adsb.altitude(msg)
                if alt is not None:
                    fields["altitude_ft"] = int(alt)
            elif tc == 28:  # aircraft status: squawk + emergency state (subtype 1 only)
                if pms.common.bin2int(pms.common.hex2bin(msg)[32:][5:8]) == 1:
                    fields["squawk"] = str(pms.bds.bds61.emergency_squawk(msg))
            elif tc == 19:
                vel = pms.adsb.velocity(msg)
                if vel is not None:
                    spd, hdg, _, _ = vel
                    if spd is not None:
                        fields["speed_kt"] = spd
                    if hdg is not None:
                        fields["heading_deg"] = hdg
            if tc is not None and (9 <= tc <= 18 or 20 <= tc <= 22):
                pos = self._buffer_and_decode_position(icao, msg, now)
                if pos is not None:
                    fields["lat"], fields["lon"] = pos
            self.table.upsert(icao, now, True, **fields)
            if self.stats is not None:
                self.stats.msgs_total += 1
        except Exception:
            return

    def _buffer_and_decode_position(self, icao: str, msg: str, now: float):
        try:
            oe_flag = pms.adsb.oe_flag(msg)
        except Exception:
            return None
        entry = self._cpr_buffer.setdefault(icao, {})
        entry["even" if oe_flag == 0 else "odd"] = (msg, now)
        if len(self._cpr_buffer) > 4000:  # prune stale CPR state
            for k in [k for k, v in self._cpr_buffer.items()
                      if all(now - t > 30 for _, t in v.values())]:
                del self._cpr_buffer[k]
        even, odd = entry.get("even"), entry.get("odd")
        if not even or not odd or abs(even[1] - odd[1]) > 10:
            return None
        try:
            position = pms.adsb.position(even[0], odd[0], even[1], odd[1])
        except Exception:
            return None
        if position is None:
            return None
        lat, lon = position
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            return None
        return lat, lon

    def snapshot(self, max_age_s: float = 60.0) -> List[Aircraft]:
        return self.table.snapshot(max_age_s)


# ─────────────────────────────────────────────────────────────────────────────
# Backend 2/3: decoder's own output (aircraft.json / SBS), also RSSI side channel
# ─────────────────────────────────────────────────────────────────────────────
class JsonSource(threading.Thread):
    """Polls the decoder's aircraft.json (dump1090-fa / readsb / mutability)."""

    def __init__(self, directory: str, table: AircraftTable, stats, stop: threading.Event):
        super().__init__(daemon=True, name="json-feed")
        self.path = os.path.join(directory, "aircraft.json")
        self.table, self.stats, self.stop_evt = table, stats, stop
        self.halt = threading.Event()
        self._last_msgs: Dict[str, int] = {}

    def run(self) -> None:
        self.stats.feed_kind = "json"
        while not (self.stop_evt.is_set() or self.halt.is_set()):
            data = None
            try:
                age = time.time() - os.stat(self.path).st_mtime
                with open(self.path, "rb") as fh:
                    data = json.loads(fh.read().decode("utf-8", "replace"))
                self.stats.feed_ok = age < 10
            except (OSError, ValueError):
                self.stats.feed_ok = False
            if isinstance(data, dict):
                try:
                    self._ingest(data)
                except Exception as exc:  # one odd record must never kill the feed thread
                    self.stats.feed_note = f"json feed: {type(exc).__name__}: {exc}"[:90]
            self.stop_evt.wait(1.0)

    def _ingest(self, data: dict) -> None:
        now = time.time()
        total = data.get("messages")
        if isinstance(total, int) and not isinstance(total, bool):
            self.stats.msgs_total = total
        seen_ids = set()
        records = data.get("aircraft")
        for a in (records if isinstance(records, list) else []):
            try:
                self._ingest_one(a, now, seen_ids)
            except Exception:
                continue
        for k in [k for k in self._last_msgs if k not in seen_ids]:
            del self._last_msgs[k]

    def _ingest_one(self, a: dict, now: float, seen_ids: set) -> None:
        icao = str(a.get("hex", "")).strip().upper()
        if not HEX6.fullmatch(icao):  # skips '~' non-ICAO (TIS-B) addresses
            return
        seen_ids.add(icao)
        msgs = to_int(a.get("messages")) or 0
        seen = to_float(a.get("seen")) or 0.0
        prev = self._last_msgs.get(icao)
        fresh = (seen < 2.0) if prev is None else (msgs > prev)
        self._last_msgs[icao] = msgs
        alt_raw = a.get("alt_baro", a.get("altitude", a.get("alt_geom")))
        on_ground = isinstance(alt_raw, str) and alt_raw.lower() == "ground"
        alt = None if on_ground else to_float(alt_raw)
        self.table.upsert(
            icao, now - seen, fresh,
            callsign=str(a.get("flight") or "").strip(),
            squawk=str(a.get("squawk") or ""),
            altitude_ft=alt, on_ground=on_ground,
            speed_kt=to_float(a.get("gs", a.get("speed"))),
            heading_deg=to_float(a.get("track")),
            lat=to_float(a.get("lat")), lon=to_float(a.get("lon")),
            rssi=to_float(a.get("rssi")), messages=msgs,
        )


class RssiSource(threading.Thread):
    """Raw mode has no signal level. If the decoder also writes aircraft.json, merge its RSSI in."""

    def __init__(self, directory: str, table: AircraftTable, stats, stop: threading.Event):
        super().__init__(daemon=True, name="rssi-feed")
        self.path = os.path.join(directory, "aircraft.json")
        self.table, self.stats, self.stop_evt = table, stats, stop
        self.halt = threading.Event()

    def run(self) -> None:
        while not (self.stop_evt.is_set() or self.halt.is_set()):
            got = False
            try:
                if time.time() - os.stat(self.path).st_mtime < 10:
                    with open(self.path, "rb") as fh:
                        data = json.loads(fh.read().decode("utf-8", "replace"))
                    for a in data.get("aircraft", []):
                        icao = str(a.get("hex", "")).strip().upper()
                        rssi = to_float(a.get("rssi"))
                        if rssi is not None and HEX6.fullmatch(icao):
                            self.table.update_existing(icao, rssi=rssi)
                            got = True
            except Exception:  # unreadable / half-written / odd JSON: just try again next second
                pass
            self.stats.rssi_ok = got
            self.stop_evt.wait(1.0)


class SbsSource(threading.Thread):
    """Reads the BaseStation/SBS-1 CSV feed on TCP (usually :30003). No RSSI available."""

    def __init__(self, host: str, port: int, table: AircraftTable, stats, stop: threading.Event):
        super().__init__(daemon=True, name="sbs-feed")
        self.host, self.port = host, port
        self.table, self.stats, self.stop_evt = table, stats, stop
        self.halt = threading.Event()

    def run(self) -> None:
        self.stats.feed_kind = "sbs"
        while not (self.stop_evt.is_set() or self.halt.is_set()):
            try:
                with socket.create_connection((self.host, self.port), timeout=5) as s:
                    s.settimeout(2.0)
                    self.stats.feed_ok = True
                    buf = b""
                    while not (self.stop_evt.is_set() or self.halt.is_set()):
                        try:
                            data = s.recv(8192)
                        except socket.timeout:
                            continue
                        if not data:
                            break
                        buf += data
                        *lines, buf = buf.split(b"\n")
                        for ln in lines:
                            try:
                                self._line(ln.decode("ascii", "ignore").strip())
                            except Exception:
                                continue
            except OSError:
                pass
            self.stats.feed_ok = False
            self.stop_evt.wait(2.0)

    def _line(self, line: str) -> None:
        f = line.split(",")
        if len(f) < 11 or f[0] != "MSG":
            return
        icao = f[4].strip().upper()
        if not HEX6.fullmatch(icao):
            return
        self.stats.msgs_total += 1

        def col(i: int) -> Optional[float]:
            return to_float(f[i]) if len(f) > i else None

        ground = len(f) > 21 and f[21].strip() == "-1"
        self.table.upsert(
            icao, time.time(), True,
            callsign=f[10].strip(), squawk=f[17].strip() if len(f) > 17 else "",
            altitude_ft=col(11), on_ground=ground, speed_kt=col(12), heading_deg=col(13),
            lat=col(14), lon=col(15),
        )
