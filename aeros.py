#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AERØS — passive aircraft tracking
=================================
Passive ADS-B / Mode S receiver + GPS wardriving logger.

AERØS wraps a dump1090-family decoder (dump1090-fa, readsb, dump1090-mutability
or the classic antirez/dump1090) driving an RTL-SDR dongle on 1090 MHz, listens
to a GPS receiver (gpsd or raw NMEA over serial/UART), and writes every
aircraft reception together with the *receiver's* GPS position to log files.

It is strictly receive-only: nothing is ever transmitted.

Credits : github.com/skuldexter-web  |  @s.k.7.l.d
License : MIT (see LICENSE)
"""
from __future__ import annotations

import argparse
import collections
import csv
import dataclasses
import datetime as dt
import getpass
import glob
import json
import math
import os
import re
import select
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from typing import Dict, List, Optional, Tuple
from xml.sax.saxutils import escape as xml_escape

try:  # optional: only needed for direct NMEA serial / UART GPS
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None

try:
    import termios
    import tty
except ImportError:  # pragma: no cover  (non-POSIX)
    termios = None
    tty = None

# ─────────────────────────────────────────────────────────────────────────────
# Identity / constants
# ─────────────────────────────────────────────────────────────────────────────
APP_NAME = "AERØS"
APP_ID = "AEROS"
VERSION = "1.0.1"
SUBTITLE = "passive aircraft tracking"
GITHUB = "github.com/skuldexter-web"
HANDLE = "@s.k.7.l.d"

PURPLE = "\033[95m"   # banner / header
CYAN = "\033[96m"     # live feed + table
RED = "\033[91m"      # faults only
DIM = "\033[2m"
RESET = "\033[0m"

# 5-row block glyphs, every row exactly 6 columns wide.
GLYPHS = {
    "A": [" ████ ", "█    █", "██████", "█    █", "█    █"],
    "E": ["██████", "█     ", "█████ ", "█     ", "██████"],
    "R": ["█████ ", "█    █", "█████ ", "█   █ ", "█    █"],
    "Ø": [" ████ ", "█   ██", "█  █ █", "██   █", " ████ "],
    "S": [" █████", "█     ", " ████ ", "     █", "█████ "],
}


def banner_lines(ascii_only: bool = False) -> List[str]:
    rows = ["  ".join(GLYPHS[ch][r] for ch in "AERØS") for r in range(5)]
    if ascii_only:
        rows = [r.replace("█", "#") for r in rows]
    return rows


UNIT_FACTORS = {"km": 1.0, "nm": 0.539957, "mi": 0.621371}
HEX6 = re.compile(r"[0-9A-F]{6}")

# dump1090 launch ladder: tried in order until one stays alive.
#   json -> decoder writes aircraft.json (gives RSSI)   sbs -> TCP :30003 feed (no RSSI)
LADDERS = [
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
BINARY_NAMES = ["dump1090-aeros", "dump1090-fa", "readsb", "dump1090-mutability", "dump1090"]
COMMON_JSON_DIRS = ["/run/dump1090-fa", "/run/readsb", "/run/dump1090-mutability", "/run/dump1090",
                    "/var/run/dump1090-fa", "/var/run/readsb"]

APT_PACKAGES = [
    "rtl-sdr", "librtlsdr-dev", "gpsd", "gpsd-clients", "python3-pip", "build-essential",
    "git", "pkg-config", "libusb-1.0-0-dev", "libncurses-dev", "python3-serial",
]
BUILD_TARGETS = [
    ("dump1090-fa", "https://github.com/flightaware/dump1090.git",
     ["dump1090", "RTLSDR=yes", "BLADERF=no", "HACKRF=no", "LIMESDR=no", "DUMP1090_VERSION=aeros"]),
    ("dump1090-classic", "https://github.com/antirez/dump1090.git", []),
]
FRIENDLY_DISTROS = {
    "kali": "Kali Linux", "ubuntu": "Ubuntu", "linuxmint": "Linux Mint", "raspbian": "Raspberry Pi OS",
    "debian": "Debian", "pop": "Pop!_OS", "parrot": "Parrot OS", "elementary": "elementary OS",
    "zorin": "Zorin OS", "devuan": "Devuan",
}
RTL_USB_IDS = {("0bda", "2832"), ("0bda", "2838"), ("0bda", "2831"), ("0bda", "2834"), ("0bda", "2837"),
               ("0bda", "2839"), ("0bda", "2870")}


# ─────────────────────────────────────────────────────────────────────────────
# Small helpers
# ─────────────────────────────────────────────────────────────────────────────
def haversine_km(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in kilometres."""
    r = 6371.0088
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def parse_iso8601(text: str) -> Optional[float]:
    m = re.match(r"(\d{4})-(\d\d)-(\d\d)T(\d\d):(\d\d):(\d\d)(\.\d+)?Z?$", str(text))
    if not m:
        return None
    try:
        y, mo, d, h, mi, s = [int(x) for x in m.groups()[:6]]
        frac = float(m.group(7)) if m.group(7) else 0.0
        return dt.datetime(y, mo, d, h, mi, s, tzinfo=dt.timezone.utc).timestamp() + frac
    except ValueError:
        return None


def to_float(value) -> Optional[float]:
    try:
        if value is None or value == "":
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def clip(text: str, width: int) -> str:
    return text if len(text) <= width else text[: max(0, width)]


def say(msg: str) -> None:
    print(f"{PURPLE}[{APP_NAME}]{RESET} {msg}", flush=True)


# ─────────────────────────────────────────────────────────────────────────────
# OS detection / system setup
# ─────────────────────────────────────────────────────────────────────────────
@dataclasses.dataclass
class Distro:
    ident: str
    like: List[str]
    pretty: str
    friendly: str
    family: str  # "debian" | "unknown"


def detect_distro() -> Distro:
    info: Dict[str, str] = {}
    for path in ("/etc/os-release", "/usr/lib/os-release"):
        try:
            with open(path, encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if "=" in line and not line.startswith("#"):
                        k, v = line.split("=", 1)
                        info[k] = v.strip().strip('"').strip("'")
            break
        except OSError:
            continue
    ident = info.get("ID", "unknown").lower()
    like = info.get("ID_LIKE", "").lower().split()
    pretty = info.get("PRETTY_NAME") or ident
    is_debian = ident in FRIENDLY_DISTROS or "debian" in like or "ubuntu" in like
    friendly = FRIENDLY_DISTROS.get(ident, pretty)
    if os.path.exists("/etc/rpi-issue"):
        friendly = "Raspberry Pi OS"
    return Distro(ident, like, pretty, friendly, "debian" if is_debian else "unknown")


def rtlsdr_devices() -> List[str]:
    found = []
    for d in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(d, "idVendor")) as f:
                vid = f.read().strip().lower()
            with open(os.path.join(d, "idProduct")) as f:
                pid = f.read().strip().lower()
        except OSError:
            continue
        if (vid, pid) in RTL_USB_IDS:
            found.append(f"{vid}:{pid}")
    return found


def dvb_driver_loaded() -> bool:
    try:
        with open("/proc/modules") as f:
            return "dvb_usb_rtl28xxu" in f.read()
    except OSError:
        return False


UBLOX_PRODUCTS = {"01a5": "u-blox 5", "01a6": "u-blox 6", "01a7": "u-blox 7",
                  "01a8": "u-blox 8/M8", "01a9": "u-blox 9"}


def usb_info_for_tty(port: str) -> Optional[Tuple[str, str, str]]:
    """(vendor_id, product_id, label) of the USB device behind a tty node, else None (UART etc.)."""
    name = os.path.basename(os.path.realpath(port))
    path = os.path.realpath(f"/sys/class/tty/{name}/device")
    for _ in range(6):
        if os.path.exists(os.path.join(path, "idVendor")):
            try:
                with open(os.path.join(path, "idVendor")) as f:
                    vid = f.read().strip().lower()
                with open(os.path.join(path, "idProduct")) as f:
                    pid = f.read().strip().lower()
            except OSError:
                return None
            label = UBLOX_PRODUCTS.get(pid, "u-blox") if vid == "1546" else ""
            if not label:
                try:
                    with open(os.path.join(path, "product")) as f:
                        label = f.read().strip()
                except OSError:
                    label = f"{vid}:{pid}"
            return vid, pid, label
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    return None


def is_ublox(port: str) -> bool:
    info = usb_info_for_tty(port)
    return bool(info and info[0] == "1546")


def find_dump1090(explicit: Optional[str] = None) -> Optional[str]:
    candidates = [explicit, os.environ.get("AEROS_DUMP1090"),
                  "/usr/local/bin/dump1090-aeros",
                  os.path.expanduser("~/.local/share/aeros/bin/dump1090")]
    for c in candidates:
        if c and os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    for name in BINARY_NAMES:
        p = shutil.which(name)
        if p:
            return p
    return None


def _priv(cmd: List[str]) -> List[str]:
    return cmd if os.geteuid() == 0 else ["sudo"] + cmd


def _run(cmd: List[str], **kw) -> subprocess.CompletedProcess:
    print(f"{DIM}  $ {' '.join(cmd)}{RESET}", flush=True)
    try:
        return subprocess.run(cmd, **kw)
    except FileNotFoundError:
        return subprocess.CompletedProcess(cmd, 127)


def _sudo_write(path: str, content: str) -> bool:
    r = _run(_priv(["tee", path]), input=content.encode(), stdout=subprocess.DEVNULL)
    return r.returncode == 0


def _apt_install(packages: List[str]) -> List[str]:
    """Install packages; returns list of packages that could not be installed."""
    _run(_priv(["apt-get", "update"]))
    base = _priv(["env", "DEBIAN_FRONTEND=noninteractive", "apt-get", "install", "-y"])
    if _run(base + packages).returncode == 0:
        return []
    say("bulk install failed — retrying package by package")
    missing = []
    for pkg in packages:
        if _run(base + [pkg]).returncode != 0:
            missing.append(pkg)
    return missing


def _install_binary(built: str) -> Optional[str]:
    target = "/usr/local/bin/dump1090-aeros"
    if _run(_priv(["install", "-m", "0755", built, target])).returncode == 0:
        return target
    local_dir = os.path.expanduser("~/.local/share/aeros/bin")
    os.makedirs(local_dir, exist_ok=True)
    target = os.path.join(local_dir, "dump1090")
    shutil.copy2(built, target)
    os.chmod(target, 0o755)
    return target


def build_dump1090() -> Optional[str]:
    """Clone + compile a dump1090 variant; returns the installed binary path."""
    cache = os.path.expanduser("~/.cache/aeros")
    os.makedirs(cache, exist_ok=True)
    jobs = str(min(os.cpu_count() or 2, 4))
    for name, url, make_args in BUILD_TARGETS:
        src = os.path.join(cache, name)
        if os.path.isdir(src):
            shutil.rmtree(src, ignore_errors=True)
        say(f"building {name} from {url}")
        if _run(["git", "clone", "--depth", "1", url, src]).returncode != 0:
            say(f"{RED}clone failed for {name}{RESET}")
            continue
        if _run(["make", "-C", src, "-j", jobs] + make_args).returncode != 0:
            say(f"{RED}build failed for {name}{RESET}")
            continue
        built = os.path.join(src, "dump1090")
        if os.path.isfile(built):
            path = _install_binary(built)
            if path:
                say(f"installed {name} -> {path}")
                return path
    return None


def setup_system(args: argparse.Namespace) -> int:
    """Detect the distro, install dependencies, configure udev/gpsd, build dump1090."""
    d = detect_distro()
    say(f"detected OS: {d.friendly} ({d.pretty}) — package family: {d.family}")
    if d.family != "debian":
        say(f"{RED}automatic setup only supports apt-based distributions.{RESET}")
        say("install manually: rtl-sdr librtlsdr-dev gpsd gpsd-clients python3-serial git make gcc pkg-config")
        say("then build dump1090-fa (github.com/flightaware/dump1090) and pass --dump1090 /path/to/binary")
        return 1
    if os.geteuid() != 0 and not shutil.which("sudo"):
        say(f"{RED}need root or sudo to install packages.{RESET}")
        return 1

    # 1. packages
    missing = _apt_install(APT_PACKAGES)
    if missing:
        say(f"{RED}could not install: {', '.join(missing)}{RESET} (continuing)")

    # 2. RTL-SDR access: keep the DVB-T kernel driver off the dongle, grant USB access
    _sudo_write("/etc/modprobe.d/aeros-rtlsdr-blacklist.conf",
                "# Written by AERØS: stop the DVB-T driver from claiming RTL-SDR dongles\n"
                "blacklist dvb_usb_rtl28xxu\nblacklist rtl2832\nblacklist rtl2830\n")
    _sudo_write("/etc/udev/rules.d/20-aeros-rtlsdr.rules",
                '# Written by AERØS\n'
                'SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2838", GROUP="plugdev", MODE="0660"\n'
                'SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="2832", GROUP="plugdev", MODE="0660"\n')
    _sudo_write("/etc/udev/rules.d/21-aeros-ublox.rules",
                "# Written by AERØS: u-blox GPS (u-blox 5/6/7/8). Stop ModemManager from probing the port\n"
                "# with AT commands (breaks NMEA) and let the 'dialout' group read it.\n"
                'ATTRS{idVendor}=="1546", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"\n'
                'SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", GROUP="dialout", MODE="0660"\n')
    _run(_priv(["udevadm", "control", "--reload-rules"]))
    _run(_priv(["udevadm", "trigger"]))
    if dvb_driver_loaded():
        _run(_priv(["modprobe", "-r", "dvb_usb_rtl28xxu"]))
    user = os.environ.get("SUDO_USER") or getpass.getuser()
    if user != "root":
        for grp in ("plugdev", "dialout"):
            if subprocess.run(["getent", "group", grp], stdout=subprocess.DEVNULL).returncode == 0:
                _run(_priv(["usermod", "-aG", grp, user]))

    # 3. gpsd
    if not getattr(args, "skip_gpsd_config", False):
        devices = args.gps_serial or ""
        if os.path.exists("/etc/default/gpsd"):
            _run(_priv(["cp", "-n", "/etc/default/gpsd", "/etc/default/gpsd.aeros.bak"]))
        _sudo_write("/etc/default/gpsd",
                    "# Written by AERØS setup (original saved as /etc/default/gpsd.aeros.bak)\n"
                    'START_DAEMON="true"\nUSBAUTO="true"\n'
                    f'DEVICES="{devices}"\nGPSD_OPTIONS="-n"\n')
        if shutil.which("systemctl"):
            _run(_priv(["systemctl", "enable", "gpsd.socket"]))
            _run(_priv(["systemctl", "restart", "gpsd.socket"]))
            _run(_priv(["systemctl", "restart", "gpsd"]))

    # 4. dump1090
    binary = find_dump1090(getattr(args, "dump1090", None))
    if binary:
        say(f"decoder already present: {binary}")
    else:
        binary = build_dump1090()
    if not binary:
        say(f"{RED}no dump1090 decoder available — build failed. See messages above.{RESET}")
        return 2

    say("setup complete.")
    say("log out/in (or reboot) so the new group memberships and the DVB blacklist apply, then run: python3 aeros.py")
    return 0


# ─────────────────────────────────────────────────────────────────────────────
# GPS
# ─────────────────────────────────────────────────────────────────────────────
@dataclasses.dataclass
class GpsFix:
    source: str = "none"
    status: str = "NO DEVICE"
    error: str = ""
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None
    mode: int = 0
    sats_used: Optional[int] = None
    sats_seen: Optional[int] = None
    hdop: Optional[float] = None
    eph: Optional[float] = None
    speed_ms: Optional[float] = None
    track: Optional[float] = None
    utc_epoch: Optional[float] = None
    utc_mono: Optional[float] = None
    static: bool = False
    age: float = 1e9

    @property
    def has_fix(self) -> bool:
        return self.mode >= 2 and self.lat is not None and self.lon is not None and self.age < 5.0

    def utc_now(self) -> float:
        if self.utc_epoch is not None and self.utc_mono is not None:
            return self.utc_epoch + (time.monotonic() - self.utc_mono)
        return time.time()

    def accuracy_m(self) -> float:
        if self.eph is not None:
            return self.eph
        if self.hdop is not None:
            return self.hdop * 5.0
        return 10.0


def nmea_valid(line: str) -> bool:
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, chk = line[1:].partition("*")
    try:
        want = int(chk[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def nmea_latlon(value: str, hemi: str, deg_digits: int) -> Optional[float]:
    try:
        deg = int(value[:deg_digits])
        minutes = float(value[deg_digits:])
    except ValueError:
        return None
    res = deg + minutes / 60.0
    return -res if hemi in ("S", "W") else res


GPS_STATUS = {3: "3D FIX", 2: "2D FIX"}


class GpsSupervisor(threading.Thread):
    """Keeps a current GpsFix up to date from gpsd, a serial NMEA port, or a fixed position."""

    def __init__(self, args: argparse.Namespace, stop: threading.Event):
        super().__init__(daemon=True, name="gps")
        self.args = args
        self.stop_evt = stop
        self._lock = threading.Lock()
        self._fix = GpsFix()
        self._updated = 0.0
        self._sv: Dict[str, int] = {}
        self._note = ""

    # -- state ---------------------------------------------------------------
    def _update(self, **kw) -> None:
        with self._lock:
            for k, v in kw.items():
                setattr(self._fix, k, v)
            self._updated = time.monotonic()

    def _status(self, status: str, error: str = "") -> None:
        with self._lock:
            self._fix.status = status
            self._fix.error = error

    def snapshot(self) -> GpsFix:
        with self._lock:
            f = dataclasses.replace(self._fix)
            f.age = 0.0 if f.static else (time.monotonic() - self._updated if self._updated else 1e9)
        return f

    # -- main loop -----------------------------------------------------------
    def run(self) -> None:
        a = self.args
        if a.fixed_pos:
            lat, lon, alt = a.fixed_pos
            while True:
                self._update(source="fixed", status="FIXED POSITION", lat=lat, lon=lon, alt=alt,
                             mode=3, static=True, eph=5.0)
                if self.stop_evt.wait(1.0):
                    return
        if a.gps == "none":
            self._status("DISABLED", "GPS disabled (--gps none)")
            return
        while not self.stop_evt.is_set():
            connected = False
            if a.gps in ("auto", "gpsd"):
                connected = self._gpsd_session()
            if not connected and a.gps in ("auto", "serial"):
                port, baud = a.gps_serial, a.gps_baud
                if not port:
                    found = self._autodetect_serial()
                    port, baud = found if found else (None, None)
                if port:
                    self._serial_session(port, baud)
                    connected = True
            if not connected:
                self._status("NO DEVICE", self._note or "no GPS found (gpsd / serial)")
            self.stop_evt.wait(3.0 if connected else 10.0)

    # -- gpsd ----------------------------------------------------------------
    def _gpsd_session(self) -> bool:
        a = self.args
        name = f"gpsd {a.gpsd_host}:{a.gpsd_port}"
        try:
            sock = socket.create_connection((a.gpsd_host, a.gpsd_port), timeout=3)
        except OSError as exc:
            self._note = f"gpsd unavailable ({exc})"
            return False
        got_data = False
        start = time.monotonic()
        with sock:
            sock.settimeout(1.0)
            try:
                sock.sendall(b'?WATCH={"enable":true,"json":true};\n')
            except OSError:
                return False
            buf = b""
            while not self.stop_evt.is_set():
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    if not got_data and time.monotonic() - start > 6:
                        self._note = "gpsd running but no GPS device attached"
                        return False
                    continue
                except OSError:
                    return got_data
                if not data:
                    return got_data
                buf += data
                *lines, buf = buf.split(b"\n")
                for raw in lines:
                    if self._handle_gpsd(raw, name):
                        got_data = True
        return got_data

    def _handle_gpsd(self, raw: bytes, name: str) -> bool:
        try:
            msg = json.loads(raw.decode("utf-8", "replace"))
        except ValueError:
            return False
        cls = msg.get("class")
        if cls == "TPV":
            mode = int(msg.get("mode", 0) or 0)
            kw = {"mode": mode, "status": GPS_STATUS.get(mode, "NO FIX"), "error": "", "source": name}
            for key, dest in (("lat", "lat"), ("lon", "lon"), ("speed", "speed_ms"),
                              ("track", "track"), ("eph", "eph")):
                v = msg.get(key)
                if isinstance(v, (int, float)):
                    kw[dest] = float(v)
            for key in ("altMSL", "alt", "altHAE"):
                if isinstance(msg.get(key), (int, float)):
                    kw["alt"] = float(msg[key])
                    break
            t = parse_iso8601(msg.get("time", ""))
            if t is not None:
                kw["utc_epoch"], kw["utc_mono"] = t, time.monotonic()
            self._update(**kw)
            return True
        if cls == "SKY":
            kw = {}
            sats = msg.get("satellites")
            if isinstance(sats, list):
                kw["sats_seen"] = len(sats)
                kw["sats_used"] = sum(1 for s in sats if isinstance(s, dict) and s.get("used"))
            if isinstance(msg.get("uSat"), int):
                kw["sats_used"] = msg["uSat"]
            if isinstance(msg.get("nSat"), int):
                kw["sats_seen"] = msg["nSat"]
            if isinstance(msg.get("hdop"), (int, float)):
                kw["hdop"] = float(msg["hdop"])
            if kw:
                self._update(**kw)
            return False
        if cls == "DEVICES":
            return bool(msg.get("devices"))
        return False

    # -- serial NMEA ---------------------------------------------------------
    def _serial_candidates(self) -> List[str]:
        ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        real = {os.path.realpath(p) for p in ports}
        for extra in ("/dev/serial0", "/dev/ttyAMA0"):  # Raspberry Pi GPIO UART
            if os.path.exists(extra) and os.path.realpath(extra) not in real:
                ports.append(extra)
                real.add(os.path.realpath(extra))
        ports.sort(key=lambda p: 0 if is_ublox(p) else 1)  # probe u-blox receivers first
        return ports

    @staticmethod
    def _open_serial(port: str, baud: int, timeout: float):
        """Open the port exclusively when pyserial supports it, so we never split bytes with gpsd."""
        try:
            return serial.Serial(port, baud, timeout=timeout, exclusive=True)
        except TypeError:  # pyserial < 3.3
            return serial.Serial(port, baud, timeout=timeout)

    def _probe_nmea(self, port: str, baud: int, dwell: float = 2.5) -> bool:
        try:
            with self._open_serial(port, baud, 0.5) as ser:
                end = time.monotonic() + dwell
                while time.monotonic() < end and not self.stop_evt.is_set():
                    line = ser.readline().decode("ascii", "ignore").strip()
                    if line and nmea_valid(line):
                        return True
        except (OSError, ValueError, getattr(serial, "SerialException", OSError)):
            return False
        return False

    def _autodetect_serial(self) -> Optional[Tuple[str, int]]:
        if serial is None:
            self._note = "pyserial missing (apt install python3-serial) — serial GPS unavailable"
            return None
        ports = self._serial_candidates()
        if not ports:
            self._note = "no gpsd and no /dev/ttyUSB*, /dev/ttyACM*, /dev/serial0 found"
            return None
        self._status("SCANNING", f"probing {len(ports)} serial port(s) for NMEA")
        for baud in (9600, 115200, 38400, 57600, 4800):
            for port in ports:
                if self.stop_evt.is_set():
                    return None
                if self._probe_nmea(port, baud):
                    return port, baud
        self._note = "serial ports present but none speaks NMEA (try --gps-serial PORT --gps-baud N)"
        return None

    def _serial_session(self, port: str, baud: int) -> None:
        if serial is None:
            self._note = "pyserial missing (apt install python3-serial)"
            return
        info = usb_info_for_tty(port)
        name = f"serial {port}@{baud}" + (f" ({info[2]})" if info else "")
        try:
            with self._open_serial(port, baud, 1) as ser:
                self._update(source=name, status="NO FIX", error="")
                last_ok = time.monotonic()
                while not self.stop_evt.is_set():
                    raw = ser.readline()
                    if raw:
                        line = raw.decode("ascii", "ignore").strip()
                        if nmea_valid(line):
                            last_ok = time.monotonic()
                            self._handle_nmea(line, name)
                    if time.monotonic() - last_ok > 10:
                        self._status("NO DATA", f"{port} open but no valid NMEA (check baud/wiring)")
        except (OSError, getattr(serial, "SerialException", OSError)) as exc:
            hint = " — add your user to the 'dialout' group" if "ermission" in str(exc) else ""
            if "lock" in str(exc).lower():
                hint = " — port is in use (gpsd?); use --gps gpsd or stop gpsd"
            self._note = f"{port}: {exc}{hint}"
            self._status("DISCONNECTED", self._note)

    def _handle_nmea(self, line: str, name: str) -> None:
        body = line[1:line.index("*")]
        f = body.split(",")
        talker, kind = f[0][:2], f[0][2:]
        try:
            if kind == "GGA" and len(f) >= 10:
                quality = int(f[6] or 0)
                kw = {"source": name}
                if quality > 0 and f[2] and f[4]:
                    kw["lat"] = nmea_latlon(f[2], f[3], 2)
                    kw["lon"] = nmea_latlon(f[4], f[5], 3)
                    kw["mode"] = 3 if f[9] else 2
                else:
                    kw["mode"] = 0
                kw["status"] = GPS_STATUS.get(kw["mode"], "NO FIX")
                kw["error"] = ""
                if f[7]:
                    kw["sats_used"] = int(f[7])
                if f[8]:
                    kw["hdop"] = float(f[8])
                if f[9]:
                    kw["alt"] = float(f[9])
                self._update(**kw)
            elif kind == "RMC" and len(f) >= 10 and f[2] == "A":
                kw = {}
                if f[7]:
                    kw["speed_ms"] = float(f[7]) * 0.514444
                if f[8]:
                    kw["track"] = float(f[8])
                if f[1] and f[9]:
                    hh, mm, ss = int(f[1][0:2]), int(f[1][2:4]), float(f[1][4:])
                    d, mo, y = int(f[9][0:2]), int(f[9][2:4]), 2000 + int(f[9][4:6])
                    kw["utc_epoch"] = dt.datetime(y, mo, d, hh, mm, 0, tzinfo=dt.timezone.utc).timestamp() + ss
                    kw["utc_mono"] = time.monotonic()
                if kw:
                    self._update(**kw)
            elif kind == "GSV" and len(f) >= 4 and f[3]:
                self._sv[talker] = int(f[3])
                self._update(sats_seen=sum(self._sv.values()))
        except (ValueError, IndexError):
            pass


# ─────────────────────────────────────────────────────────────────────────────
# Aircraft model + sources
# ─────────────────────────────────────────────────────────────────────────────
@dataclasses.dataclass
class Aircraft:
    icao: str
    callsign: str = ""
    squawk: str = ""
    alt_ft: Optional[float] = None
    on_ground: bool = False
    speed_kt: Optional[float] = None
    track: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    rssi: Optional[float] = None
    messages: int = 0
    first_seen: float = 0.0
    last_seen: float = 0.0
    fresh: bool = False


class AircraftTable:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ac: Dict[str, Aircraft] = {}
        self.unique = 0

    def upsert(self, icao: str, seen_at: float, fresh: bool, **fields) -> None:
        with self._lock:
            ac = self._ac.get(icao)
            if ac is None:
                ac = Aircraft(icao=icao, first_seen=seen_at)
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

    def drain_fresh(self) -> List[Aircraft]:
        with self._lock:
            out = [dataclasses.replace(a) for a in self._ac.values() if a.fresh]
            for a in self._ac.values():
                a.fresh = False
        return out

    def snapshot(self, max_age: float) -> List[Aircraft]:
        now = time.time()
        with self._lock:
            for k in [k for k, a in self._ac.items() if now - a.last_seen > max(max_age * 5, 300)]:
                del self._ac[k]
            return [dataclasses.replace(a) for a in self._ac.values() if now - a.last_seen <= max_age]


class Stats:
    def __init__(self) -> None:
        self.sdr_state = "INIT"
        self.sdr_detail = ""
        self.feed_kind = "-"
        self.feed_ok = False
        self.msgs_total = 0
        self.msgs_rate = 0.0
        self.log_rows = 0
        self.logging_note = ""


class JsonSource(threading.Thread):
    """Polls the decoder's aircraft.json (dump1090-fa / readsb / mutability)."""

    def __init__(self, directory: str, table: AircraftTable, stats: Stats, stop: threading.Event):
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
                self._ingest(data)
            self.stop_evt.wait(1.0)

    def _ingest(self, data: dict) -> None:
        now = time.time()
        total = data.get("messages")
        if isinstance(total, int):
            self.stats.msgs_total = total
        seen_ids = set()
        for a in data.get("aircraft", []):
            icao = str(a.get("hex", "")).strip().upper()
            if not HEX6.fullmatch(icao):  # skips '~' non-ICAO (TIS-B) addresses
                continue
            seen_ids.add(icao)
            msgs = int(a.get("messages", 0) or 0)
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
                alt_ft=alt, on_ground=on_ground,
                speed_kt=to_float(a.get("gs", a.get("speed"))),
                track=to_float(a.get("track")),
                lat=to_float(a.get("lat")), lon=to_float(a.get("lon")),
                rssi=to_float(a.get("rssi")), messages=msgs,
            )
        for k in [k for k in self._last_msgs if k not in seen_ids]:
            del self._last_msgs[k]


class SbsSource(threading.Thread):
    """Reads the BaseStation/SBS-1 CSV feed on TCP (usually :30003). No RSSI available."""

    def __init__(self, host: str, port: int, table: AircraftTable, stats: Stats, stop: threading.Event):
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
                            self._line(ln.decode("ascii", "ignore").strip())
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
            alt_ft=col(11), on_ground=ground, speed_kt=col(12), track=col(13),
            lat=col(14), lon=col(15),
        )


def explain_sdr_error(text: str) -> str:
    t = text.lower()
    if "kernel driver is active" in t or "usb_claim_interface" in t or "resource busy" in t:
        return "dongle busy: DVB driver loaded or another SDR app running (run --setup, then replug)"
    if "no supported" in t or "no rtl" in t or "no device" in t:
        return "no RTL-SDR dongle found: check USB connection / power / antenna"
    if "permission" in t or "usb_open error -3" in t:
        return "USB permission denied: add user to plugdev (run --setup, re-login) or use sudo"
    last = [ln for ln in text.splitlines() if ln.strip()]
    return (last[-1][:110] if last else "decoder exited immediately")


class SdrSupervisor(threading.Thread):
    """Starts dump1090 (trying flag sets until one works), watches it, restarts on failure."""

    def __init__(self, binary: str, args: argparse.Namespace, json_dir: str,
                 stats: Stats, stop: threading.Event, on_started):
        super().__init__(daemon=True, name="sdr")
        self.binary, self.args, self.json_dir = binary, args, json_dir
        self.stats, self.stop_evt, self.on_started = stats, stop, on_started
        self.proc: Optional[subprocess.Popen] = None
        self.tail: collections.deque = collections.deque(maxlen=40)

    def _variants(self):
        extra: List[str] = []
        if self.args.gain is not None:
            extra += ["--gain", str(self.args.gain)]
        if self.args.device_index is not None:
            extra += ["--device-index", str(self.args.device_index)]
        if self.args.dump1090_args:
            extra += shlex.split(self.args.dump1090_args)
        for mode, tmpl in LADDERS:
            yield mode, [self.binary] + [x.replace("{json}", self.json_dir) for x in tmpl] + extra

    def _drain(self, proc: subprocess.Popen) -> None:
        try:
            for line in proc.stderr:  # type: ignore[union-attr]
                line = line.strip()
                if line:
                    self.tail.append(line)
        except (ValueError, OSError):
            pass

    def _try_start(self) -> Tuple[Optional[subprocess.Popen], str, str]:
        last_err = ""
        for mode, argv in self._variants():
            self.tail.clear()
            try:
                proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                        stderr=subprocess.PIPE, start_new_session=True,
                                        text=True, errors="replace")
            except OSError as exc:
                return None, "", f"cannot execute {self.binary}: {exc}"
            reader = threading.Thread(target=self._drain, args=(proc,), daemon=True)
            reader.start()
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and proc.poll() is None and not self.stop_evt.is_set():
                time.sleep(0.2)
            if proc.poll() is None:
                return proc, mode, ""
            reader.join(timeout=1.0)
            err = "\n".join(self.tail)
            last_err = err or f"exit code {proc.returncode}"
            if HW_ERR.search(err):  # hardware problem — other flag sets won't help
                return None, "", last_err
        return None, "", last_err

    def run(self) -> None:
        name = os.path.basename(self.binary)
        while not self.stop_evt.is_set():
            self.stats.sdr_state, self.stats.sdr_detail = "STARTING", name
            proc, mode, err = self._try_start()
            if proc is None:
                if self.stop_evt.is_set():
                    break
                self.stats.sdr_state = "ERROR"
                self.stats.sdr_detail = explain_sdr_error(err)
                self.stats.feed_ok = False
                self.stop_evt.wait(10.0)
                continue
            self.proc = proc
            self.stats.sdr_state, self.stats.sdr_detail = "RUNNING", f"{name} [{mode}]"
            self.on_started(mode)
            while not self.stop_evt.is_set() and proc.poll() is None:
                self.stop_evt.wait(1.0)
            if self.stop_evt.is_set():
                break
            self.stats.sdr_state = "STOPPED"
            self.stats.sdr_detail = f"{name} exited ({proc.returncode}); restarting"
            self.stop_evt.wait(5.0)
        self.terminate()

    def terminate(self) -> None:
        proc = self.proc
        if proc and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except (subprocess.TimeoutExpired, OSError):
                try:
                    proc.kill()
                except OSError:
                    pass


# ─────────────────────────────────────────────────────────────────────────────
# Logging (WiGLE-style CSV, full-detail .log, KML)
# ─────────────────────────────────────────────────────────────────────────────
@dataclasses.dataclass
class Sighting:
    utc: float
    rx_lat: float
    rx_lon: float
    rx_alt: Optional[float]
    rx_acc: float
    sats: Optional[int]
    icao: str
    callsign: str
    squawk: str
    alt_ft: Optional[float]
    speed_kt: Optional[float]
    track: Optional[float]
    ac_lat: Optional[float]
    ac_lon: Optional[float]
    rssi: Optional[float]
    dist_km: Optional[float]


FULL_COLUMNS = ["utc", "rx_lat", "rx_lon", "rx_alt_m", "rx_acc_m", "gps_sats", "icao", "callsign", "squawk",
                "ac_alt_ft", "ac_speed_kt", "ac_track", "ac_lat", "ac_lon", "rssi_dbfs", "distance_km"]
WIGLE_COLUMNS = ["MAC", "SSID", "AuthMode", "FirstSeen", "Channel", "RSSI", "CurrentLatitude",
                 "CurrentLongitude", "AltitudeMeters", "AccuracyMeters", "Type"]


def _fmt(v: Optional[float], spec: str) -> str:
    return "" if v is None else format(v, spec)


class WardriveLogger:
    def __init__(self, outdir: str, stamp: str, wigle_type: str):
        os.makedirs(outdir, exist_ok=True)
        base = os.path.join(outdir, f"{APP_ID}_{stamp}")
        self.base = base
        self.log_path, self.csv_path, self.kml_path = base + ".log", base + ".csv", base + ".kml"
        self.wigle_type = wigle_type
        self.rows = 0
        self._lock = threading.Lock()
        self._log = open(self.log_path, "w", newline="", encoding="utf-8")
        self._lw = csv.writer(self._log, lineterminator="\n")
        self._lw.writerow(FULL_COLUMNS)
        self._wig = open(self.csv_path, "w", newline="", encoding="utf-8")
        model = f"{APP_ID}-{VERSION}"
        self._wig.write(f"WigleWifi-1.4,appRelease={model},model={APP_ID},release={os.uname().release},"
                        f"device={APP_ID},display=,board=,brand=skuldexter-web\n")
        self._ww = csv.writer(self._wig, lineterminator="\n")
        self._ww.writerow(WIGLE_COLUMNS)
        self._flush()

    def _flush(self) -> None:
        self._log.flush()
        self._wig.flush()

    def add(self, s: Sighting) -> None:
        when = dt.datetime.fromtimestamp(s.utc, dt.timezone.utc)
        with self._lock:
            self._lw.writerow([
                when.strftime("%Y-%m-%dT%H:%M:%SZ"), f"{s.rx_lat:.7f}", f"{s.rx_lon:.7f}",
                _fmt(s.rx_alt, ".1f"), f"{s.rx_acc:.1f}", "" if s.sats is None else s.sats,
                s.icao, s.callsign, s.squawk, _fmt(s.alt_ft, ".0f"), _fmt(s.speed_kt, ".0f"),
                _fmt(s.track, ".0f"), _fmt(s.ac_lat, ".6f"), _fmt(s.ac_lon, ".6f"),
                _fmt(s.rssi, ".1f"), _fmt(s.dist_km, ".2f"),
            ])
            self._ww.writerow([
                s.icao, s.callsign, "[ADS-B]", when.strftime("%Y-%m-%d %H:%M:%S"), 1090,
                int(round(s.rssi)) if s.rssi is not None else 0,
                f"{s.rx_lat:.7f}", f"{s.rx_lon:.7f}", f"{(s.rx_alt or 0.0):.1f}", f"{s.rx_acc:.1f}",
                self.wigle_type,
            ])
            self.rows += 1
            self._flush()

    def write_kml(self) -> None:
        """Rebuild the KML from the .log (receiver observations + aircraft tracks)."""
        with self._lock:
            self._flush()
            try:
                with open(self.log_path, newline="", encoding="utf-8") as fh:
                    rows = list(csv.DictReader(fh))
            except OSError:
                return
        tmp = self.kml_path + ".tmp"
        tracks: Dict[str, List[str]] = collections.OrderedDict()
        with open(tmp, "w", encoding="utf-8") as out:
            out.write('<?xml version="1.0" encoding="UTF-8"?>\n'
                      '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>\n'
                      f"<name>{xml_escape(os.path.basename(self.base))}</name>\n"
                      '<Style id="rx"><IconStyle><color>ffff55aa</color><scale>0.7</scale><Icon>'
                      "<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>"
                      "</Icon></IconStyle></Style>\n"
                      '<Style id="trk"><LineStyle><color>fff0e000</color><width>2</width></LineStyle></Style>\n'
                      "<Folder><name>Receiver observations</name>\n")
            for r in rows:
                try:
                    rx_lat, rx_lon = float(r["rx_lat"]), float(r["rx_lon"])
                except (KeyError, ValueError):
                    continue
                rx_alt = r.get("rx_alt_m") or "0"
                label = r["icao"] + (f" {r['callsign']}" if r.get("callsign") else "")
                desc = (f"ICAO {r['icao']} callsign {r.get('callsign', '')} alt {r.get('ac_alt_ft', '')} ft "
                        f"speed {r.get('ac_speed_kt', '')} kt RSSI {r.get('rssi_dbfs', '')} dBFS "
                        f"distance {r.get('distance_km', '')} km")
                out.write(f"<Placemark><name>{xml_escape(label)}</name><styleUrl>#rx</styleUrl>"
                          f"<description>{xml_escape(desc)}</description>"
                          f"<TimeStamp><when>{r['utc']}</when></TimeStamp>"
                          f"<Point><coordinates>{rx_lon},{rx_lat},{rx_alt}</coordinates></Point></Placemark>\n")
                if r.get("ac_lat") and r.get("ac_lon"):
                    alt_m = (float(r["ac_alt_ft"]) * 0.3048) if r.get("ac_alt_ft") else 0.0
                    tracks.setdefault(r["icao"], []).append(f"{r['ac_lon']},{r['ac_lat']},{alt_m:.0f}")
            out.write("</Folder>\n<Folder><name>Aircraft tracks</name>\n")
            for icao, pts in tracks.items():
                if len(pts) >= 2:
                    out.write(f"<Placemark><name>{xml_escape(icao)}</name><styleUrl>#trk</styleUrl>"
                              f"<LineString><altitudeMode>absolute</altitudeMode>"
                              f"<coordinates>{' '.join(pts)}</coordinates></LineString></Placemark>\n")
            out.write("</Folder>\n</Document></kml>\n")
        os.replace(tmp, self.kml_path)

    def close(self) -> None:
        with self._lock:
            for fh in (self._log, self._wig):
                try:
                    fh.flush()
                    fh.close()
                except OSError:
                    pass


class WardriveEngine(threading.Thread):
    """Once per second: correlate freshly received aircraft with the current GPS fix and log them."""

    def __init__(self, table: AircraftTable, gps: GpsSupervisor, logger: WardriveLogger,
                 stats: Stats, stop: threading.Event, interval: float, move_m: float):
        super().__init__(daemon=True, name="engine")
        self.table, self.gps, self.logger = table, gps, logger
        self.stats, self.stop_evt = stats, stop
        self.interval, self.move_m = interval, move_m
        self._last: Dict[str, Tuple[float, float, float]] = {}

    def run(self) -> None:
        last_total, last_t = 0, time.monotonic()
        while not self.stop_evt.wait(1.0):
            now = time.monotonic()
            total = self.stats.msgs_total
            if now > last_t:
                self.stats.msgs_rate = max(0.0, (total - last_total) / (now - last_t))
            last_total, last_t = total, now
            fresh = self.table.drain_fresh()
            fix = self.gps.snapshot()
            if not fix.has_fix:
                self.stats.logging_note = "waiting for GPS fix — not logging"
                continue
            self.stats.logging_note = ""
            for ac in fresh:
                self._maybe_log(ac, fix, now)
            if len(self._last) > 5000:
                self._last = {k: v for k, v in self._last.items() if now - v[0] < 600}

    def _maybe_log(self, ac: Aircraft, fix: GpsFix, now: float) -> None:
        last = self._last.get(ac.icao)
        if last is not None:
            moved_m = haversine_km(last[1], last[2], fix.lat, fix.lon) * 1000.0
            if now - last[0] < self.interval and moved_m < self.move_m:
                return
        dist = None
        if ac.lat is not None and ac.lon is not None:
            dist = haversine_km(fix.lat, fix.lon, ac.lat, ac.lon)
        self.logger.add(Sighting(
            utc=fix.utc_now(), rx_lat=fix.lat, rx_lon=fix.lon, rx_alt=fix.alt, rx_acc=fix.accuracy_m(),
            sats=fix.sats_used, icao=ac.icao, callsign=ac.callsign, squawk=ac.squawk, alt_ft=ac.alt_ft,
            speed_kt=ac.speed_kt, track=ac.track, ac_lat=ac.lat, ac_lon=ac.lon, rssi=ac.rssi, dist_km=dist,
        ))
        self._last[ac.icao] = (now, fix.lat, fix.lon)
        self.stats.log_rows = self.logger.rows


# ─────────────────────────────────────────────────────────────────────────────
# Application / terminal UI
# ─────────────────────────────────────────────────────────────────────────────
ROW_FMT = "{:<6}  {:<8}  {:>9}  {:>7}  {:>7}  {:>4}  {:>7}  {:>5}"


class App:
    def __init__(self, args: argparse.Namespace, binary: Optional[str], distro: Distro):
        self.args, self.binary, self.distro = args, binary, distro
        self.stop = threading.Event()
        self.stats = Stats()
        self.table = AircraftTable()
        self.gps = GpsSupervisor(args, self.stop)
        self.logger: Optional[WardriveLogger] = None
        self.sdr: Optional[SdrSupervisor] = None
        self.source = None
        self.source_mode = ""
        self.json_dir = ""
        self._src_lock = threading.Lock()
        self.banner = banner_lines(args.ascii)
        self.sep = "-" if args.ascii else "─"
        self.dot = "|" if args.ascii else "·"

    # -- data source management ---------------------------------------------
    def _start_source(self, mode: str, directory: str = "") -> None:
        with self._src_lock:
            if self.source is not None and self.source_mode == mode:
                return
            if self.source is not None:
                self.source.halt.set()
            if mode == "json":
                self.source = JsonSource(directory or self.json_dir, self.table, self.stats, self.stop)
            else:
                self.source = SbsSource(self.args.sbs_host, self.args.sbs_port, self.table, self.stats, self.stop)
            self.source_mode = mode
            self.source.start()

    # -- UI ------------------------------------------------------------------
    def render(self) -> str:
        cols, rows = shutil.get_terminal_size((100, 32))
        cols = max(cols, 40)
        fix = self.gps.snapshot()
        st = self.stats
        now = time.time()
        out: List[str] = []

        def add(text: str, color: str = CYAN) -> None:
            out.append(color + clip(text, cols - 1) + RESET)

        # persistent header (always pinned: whole frame is redrawn from the top)
        if rows >= 24:
            for line in self.banner:
                add("  " + line, PURPLE)
            add(f"  {SUBTITLE}  {self.dot}  v{VERSION}", PURPLE)
            add(f"  {GITHUB}  {self.dot}  {HANDLE}", PURPLE)
        else:
            add(f" {APP_NAME} {self.dot} {SUBTITLE} {self.dot} {GITHUB} {self.dot} {HANDLE}", PURPLE)
        add(self.sep * (cols - 1), PURPLE)

        # receiver status
        bad = st.sdr_state in ("ERROR", "STOPPED")
        add(f" SDR  {st.sdr_state:<9} {st.sdr_detail}", RED if bad else CYAN)
        feed = f"feed {st.feed_kind} {'OK' if st.feed_ok else '--'}   msgs {st.msgs_total}   {st.msgs_rate:.0f}/s"
        add(f"      {feed}")
        sats = "-" if fix.sats_used is None else str(fix.sats_used)
        seen = "-" if fix.sats_seen is None else str(fix.sats_seen)
        hdop = "-" if fix.hdop is None else f"{fix.hdop:.1f}"
        gps_bad = fix.status in ("NO DEVICE", "NO DATA", "DISABLED")
        add(f" GPS  {fix.status:<9} sats {sats}/{seen}  hdop {hdop}  {fix.source if fix.source != 'none' else ''}"
            + (f"  [{fix.error}]" if fix.error else ""), RED if gps_bad else CYAN)
        if fix.lat is not None and fix.lon is not None and fix.mode >= 2:
            alt = "-" if fix.alt is None else f"{fix.alt:.0f} m"
            utc = time.strftime("%H:%M:%S", time.gmtime(fix.utc_now()))
            add(f" POS  {fix.lat:.6f}, {fix.lon:.6f}  alt {alt}  UTC {utc}")
        else:
            add(" POS  no position yet")
        live = self.table.snapshot(self.args.max_age)
        name = os.path.basename(self.logger.base) if self.logger else "-"
        note = f"  [{st.logging_note}]" if st.logging_note else ""
        add(f" LOG  {name}  rows {st.log_rows}  aircraft now {len(live)} / total {self.table.unique}{note}")
        add(self.sep * (cols - 1), PURPLE)

        # aircraft table
        u = self.args.units
        add(ROW_FMT.format("ICAO", "CALLSIGN", f"DIST({u})", "ALT(ft)", "SPD(kt)", "HDG", "RSSI", "SEEN"))
        entries = []
        for ac in live:
            dist = None
            if fix.has_fix and ac.lat is not None and ac.lon is not None:
                dist = haversine_km(fix.lat, fix.lon, ac.lat, ac.lon)
            entries.append((ac, dist))
        key = self.args.sort
        if key == "dist":
            entries.sort(key=lambda e: (e[1] is None, e[1] or 0.0))
        elif key == "rssi":
            entries.sort(key=lambda e: (e[0].rssi is None, -(e[0].rssi or 0.0)))
        else:
            entries.sort(key=lambda e: -e[0].last_seen)
        room = max(0, rows - len(out) - 1)  # keep one line for the footer
        shown = entries[:room]
        if len(entries) > room and room > 0:
            shown = entries[:room - 1]
        for ac, dist in shown:
            d = "-" if dist is None else f"{dist * UNIT_FACTORS[u]:.1f}"
            alt = "GND" if ac.on_ground else ("-" if ac.alt_ft is None else f"{ac.alt_ft:.0f}")
            spd = "-" if ac.speed_kt is None else f"{ac.speed_kt:.0f}"
            hdg = "-" if ac.track is None else f"{ac.track:.0f}"
            rssi = "n/a" if ac.rssi is None else f"{ac.rssi:.1f}"
            age = f"{max(0, now - ac.last_seen):.0f}s"
            add(ROW_FMT.format(ac.icao, ac.callsign or "-", d, alt, spd, hdg, rssi, age))
        if len(entries) > len(shown):
            add(f"  ... +{len(entries) - len(shown)} more (enlarge the terminal)")
        footer = f" [q] quit & save   [s] save KML now   RSSI = dBFS (relative)"
        while len(out) < rows - 1:
            out.append("")
        add(footer, PURPLE)
        return "\033[H" + "\n".join(line + "\033[K" for line in out[:rows]) + "\033[J"

    def _ui_loop(self) -> None:
        fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        old = None
        if fd is not None and termios is not None:
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        sys.stdout.write("\033[?1049h\033[?25l\033[2J")
        sys.stdout.flush()
        next_save = time.monotonic() + self.args.save_interval
        try:
            while not self.stop.is_set():
                sys.stdout.write(self.render())
                sys.stdout.flush()
                if old is not None:
                    ready, _, _ = select.select([sys.stdin], [], [], 0.5)
                    if ready:
                        for ch in os.read(fd, 32).decode("utf-8", "ignore").lower():
                            if ch == "q":
                                self.stop.set()
                            elif ch == "s":
                                self._safe_save()
                else:
                    self.stop.wait(0.5)
                if time.monotonic() >= next_save:
                    self._safe_save()
                    next_save = time.monotonic() + self.args.save_interval
        finally:
            if old is not None:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            sys.stdout.write("\033[?25h\033[?1049l")
            sys.stdout.flush()

    def _headless_loop(self) -> None:
        next_save = time.monotonic() + self.args.save_interval
        while not self.stop.wait(10.0):
            fix = self.gps.snapshot()
            print(f"[{time.strftime('%H:%M:%S')}] sdr={self.stats.sdr_state} gps={fix.status} "
                  f"aircraft={len(self.table.snapshot(self.args.max_age))} rows={self.stats.log_rows} "
                  f"msgs/s={self.stats.msgs_rate:.0f}", flush=True)
            if time.monotonic() >= next_save:
                self._safe_save()
                next_save = time.monotonic() + self.args.save_interval

    def _safe_save(self) -> None:
        try:
            if self.logger:
                self.logger.write_kml()
        except Exception as exc:  # never let a save problem kill the UI
            self.stats.logging_note = f"KML save failed: {exc}"

    # -- lifecycle -----------------------------------------------------------
    def run(self) -> int:
        a = self.args
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.logger = WardriveLogger(a.output_dir, stamp, a.wigle_type)
        self.json_dir = a.json_dir or tempfile.mkdtemp(prefix="aeros_")
        own_tmp = not a.json_dir

        def handler(signum, frame):  # noqa: ARG001
            self.stop.set()

        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, handler)

        engine = WardriveEngine(self.table, self.gps, self.logger, self.stats, self.stop,
                                a.log_interval, a.log_move)
        self.gps.start()
        engine.start()

        if a.no_spawn:
            self.stats.sdr_state, self.stats.sdr_detail = "EXTERNAL", "using an already-running decoder"
            if a.json_dir:
                self._start_source("json", a.json_dir)
            else:
                found = next((d for d in COMMON_JSON_DIRS if os.path.exists(os.path.join(d, "aircraft.json"))), "")
                if found:
                    self._start_source("json", found)
                else:
                    self._start_source("sbs")
        else:
            self.sdr = SdrSupervisor(self.binary or "", a, self.json_dir, self.stats, self.stop,
                                     lambda mode: self._start_source(mode))
            self.sdr.start()

        interactive = sys.stdout.isatty() and not a.no_ui
        try:
            if interactive:
                self._ui_loop()
            else:
                self._headless_loop()
        finally:
            self.stop.set()
            if self.sdr:
                self.sdr.terminate()
            for t in (engine, self.gps):
                t.join(timeout=2.0)
            self._safe_save()
            self.logger.close()
            if own_tmp:
                shutil.rmtree(self.json_dir, ignore_errors=True)
        say(f"session ended — {self.logger.rows} rows, {self.table.unique} unique aircraft")
        say(f"  {self.logger.csv_path}   (WiGLE-style CSV)")
        say(f"  {self.logger.log_path}   (full detail)")
        say(f"  {self.logger.kml_path}   (Google Earth / KML)")
        return 0


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
def fixed_pos_type(text: str) -> Tuple[float, float, Optional[float]]:
    parts = [p.strip() for p in text.split(",")]
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError("use LAT,LON or LAT,LON,ALT_M")
    try:
        lat, lon = float(parts[0]), float(parts[1])
        alt = float(parts[2]) if len(parts) == 3 else None
    except ValueError:
        raise argparse.ArgumentTypeError("LAT,LON,ALT must be numbers")
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise argparse.ArgumentTypeError("latitude/longitude out of range")
    return lat, lon, alt


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="aeros.py",
        description=f"{APP_NAME} — {SUBTITLE}. Passive ADS-B / Mode S receiver with GPS wardriving logs.",
        epilog=f"{GITHUB}  |  {HANDLE}")
    g = p.add_argument_group("GPS")
    g.add_argument("--gps", choices=["auto", "gpsd", "serial", "none"], default="auto",
                   help="GPS source (default: auto = gpsd first, then scan serial ports)")
    g.add_argument("--gpsd-host", default="127.0.0.1")
    g.add_argument("--gpsd-port", type=int, default=2947)
    g.add_argument("--gps-serial", metavar="PORT", help="NMEA serial port, e.g. /dev/ttyUSB0, /dev/ttyACM0, /dev/serial0")
    g.add_argument("--gps-baud", type=int, default=9600, help="baud rate for --gps-serial (default 9600)")
    g.add_argument("--fixed-pos", type=fixed_pos_type, metavar="LAT,LON[,ALT_M]",
                   help="use a fixed receiver position instead of GPS (bench testing / static site)")
    d = p.add_argument_group("SDR / decoder")
    d.add_argument("--dump1090", metavar="PATH", help="path to a dump1090-family binary")
    d.add_argument("--no-spawn", action="store_true", help="do not start a decoder; read an existing one")
    d.add_argument("--json-dir", metavar="DIR", help="directory holding aircraft.json (with --no-spawn)")
    d.add_argument("--sbs-host", default="127.0.0.1")
    d.add_argument("--sbs-port", type=int, default=30003)
    d.add_argument("--gain", metavar="DB", help="tuner gain in dB (default: decoder's auto gain)")
    d.add_argument("--device-index", type=int, metavar="N", help="RTL-SDR device index")
    d.add_argument("--dump1090-args", metavar="STR", help="extra raw arguments passed to the decoder")
    d.add_argument("--no-auto-setup", action="store_true", help="never try to install/build the decoder automatically")
    o = p.add_argument_group("logging")
    o.add_argument("--output-dir", default="logs", metavar="DIR", help="where AEROS_<timestamp>.* files go (default ./logs)")
    o.add_argument("--log-interval", type=float, default=5.0, help="min seconds between rows per aircraft (default 5)")
    o.add_argument("--log-move", type=float, default=50.0, help="also log when the receiver moved this many metres (default 50)")
    o.add_argument("--save-interval", type=float, default=60.0, help="seconds between KML rebuilds (default 60)")
    o.add_argument("--wigle-type", default="ADSB",
                   help="value for the CSV 'Type' column (default ADSB; see README before changing)")
    u = p.add_argument_group("display")
    u.add_argument("--units", choices=list(UNIT_FACTORS), default="km", help="distance units (default km)")
    u.add_argument("--sort", choices=["dist", "seen", "rssi"], default="dist")
    u.add_argument("--max-age", type=float, default=60.0, help="hide aircraft not heard for N seconds (default 60)")
    u.add_argument("--ascii", action="store_true", help="plain ASCII banner/lines (no box-drawing characters)")
    u.add_argument("--no-ui", action="store_true", help="headless: log only, print a status line every 10 s")
    s = p.add_argument_group("maintenance")
    s.add_argument("--setup", action="store_true", help="install dependencies, configure udev/gpsd, build dump1090, then exit")
    s.add_argument("--skip-gpsd-config", action="store_true", help="with --setup: leave /etc/default/gpsd alone")
    s.add_argument("--check", action="store_true", help="print a hardware/dependency diagnostic and exit")
    s.add_argument("--version", action="version", version=f"{APP_NAME} {VERSION}")
    return p


def run_check(args: argparse.Namespace, distro: Distro, binary: Optional[str]) -> int:
    say(f"OS          : {distro.friendly} ({distro.pretty}) [{distro.family}]")
    say(f"decoder     : {binary or 'NOT FOUND (run: python3 aeros.py --setup)'}")
    devs = rtlsdr_devices()
    say(f"RTL-SDR USB : {', '.join(devs) if devs else 'not detected'}")
    if dvb_driver_loaded():
        say(f"{RED}dvb_usb_rtl28xxu kernel driver is loaded — it will block the dongle (run --setup){RESET}")
    try:
        socket.create_connection((args.gpsd_host, args.gpsd_port), timeout=2).close()
        say(f"gpsd        : reachable at {args.gpsd_host}:{args.gpsd_port}")
    except OSError as exc:
        say(f"gpsd        : not reachable ({exc})")
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") +
                   [p for p in ("/dev/serial0", "/dev/ttyAMA0") if os.path.exists(p)])
    say(f"serial ports: {', '.join(ports) if ports else 'none'}   pyserial: {'yes' if serial else 'no'}")
    ublox_seen = False
    for port in ports:
        info = usb_info_for_tty(port)
        if info:
            ublox_seen = ublox_seen or info[0] == "1546"
            say(f"  {port}: {info[2]} ({info[0]}:{info[1]})")
        else:
            say(f"  {port}: not USB (UART / other)")
    if ublox_seen and shutil.which("systemctl"):
        mm = subprocess.run(["systemctl", "is-active", "--quiet", "ModemManager"]).returncode == 0
        if mm:
            say(f"{RED}ModemManager is active and may grab the u-blox port; "
                f"run --setup to install the ignore rule (then replug the GPS){RESET}")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if not sys.platform.startswith("linux"):
        say(f"{RED}AERØS targets Linux (Ubuntu, Kali, Mint, Raspberry Pi OS, Debian).{RESET}")
    if args.setup:
        return setup_system(args)
    distro = detect_distro()
    binary = find_dump1090(args.dump1090)
    if args.check:
        return run_check(args, distro, binary)

    say(f"{APP_NAME} v{VERSION} — {distro.friendly}")
    if not args.no_spawn and binary is None:
        if args.no_auto_setup or distro.family != "debian":
            say(f"{RED}no dump1090 decoder found.{RESET} Run: python3 aeros.py --setup  (or pass --dump1090 PATH)")
            return 2
        say("no dump1090 decoder found — running automatic setup (sudo password may be requested)")
        rc = setup_system(args)
        binary = find_dump1090(args.dump1090)
        if binary is None:
            say(f"{RED}setup did not produce a decoder (exit {rc}). Aborting.{RESET}")
            return 2
    if binary:
        say(f"decoder: {binary}")
    if not args.no_spawn:
        devs = rtlsdr_devices()
        if not devs:
            say(f"{RED}warning: no RTL-SDR dongle seen on USB — will keep retrying{RESET}")
        if dvb_driver_loaded():
            say(f"{RED}warning: DVB kernel driver loaded; it can block the dongle (python3 aeros.py --setup){RESET}")
    if args.gps == "serial" and not args.gps_serial and not args.fixed_pos:
        say("--gps serial without --gps-serial: ports will be auto-scanned")
    time.sleep(0.8)  # let the preflight lines be readable before the full-screen UI starts
    return App(args, binary, distro).run()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
