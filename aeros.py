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
import curses
import dataclasses
import datetime as dt
import getpass
import glob
import json
import locale
import math
import os
import re
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

from airplanes import (  # noqa: E402  (decoder side lives in airplanes.py)
    HAVE_PYMODES, PYMODES_ERROR, AircraftTable, AircraftTracker, Dump1090Controller,
    JsonSource, RssiSource, SbsSource, explain_sdr_error,
)

# ─────────────────────────────────────────────────────────────────────────────
# Identity / constants
# ─────────────────────────────────────────────────────────────────────────────
APP_NAME = "AERØS"
APP_ID = "AEROS"
VERSION = "1.2.1"
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

COMMON_JSON_DIRS = ["/run/dump1090-fa", "/run/readsb", "/run/dump1090-mutability", "/run/dump1090",
                    "/var/run/dump1090-fa", "/var/run/readsb"]

APT_PACKAGES = [
    "rtl-sdr", "librtlsdr-dev", "gpsd", "gpsd-clients", "python3-pip", "build-essential",
    "git", "pkg-config", "libusb-1.0-0-dev", "libncurses-dev", "python3-serial",
    "python3-numpy", "python3-zmq",
]
BUILD_TARGETS = [
    ("dump1090-fa", "https://github.com/flightaware/dump1090.git",
     ["dump1090", "RTLSDR=yes", "BLADERF=no", "HACKRF=no", "LIMESDR=no", "DUMP1090_VERSION=aeros",
      "DUMP1090_CFLAGS=-std=c11 -fno-common -Wall -Wmissing-declarations -Wformat-signedness -W"]),
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


def detect_distro(paths: Tuple[str, ...] = ("/etc/os-release", "/usr/lib/os-release")) -> Distro:
    info: Dict[str, str] = {}
    for path in paths:
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


def detect_virt() -> str:
    """Name of the hypervisor when running in a VM (typical for Kali), else ''."""
    try:
        out = subprocess.run(["systemd-detect-virt"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             universal_newlines=True, timeout=3).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""
    return "" if out in ("", "none") else out


VM_USB_HINT = ("USB devices must be attached to the VM itself (VirtualBox: Devices > USB, USB 2.0/3.0 needs the "
               "Extension Pack; VMware: VM > Removable Devices > Connect)")


def dvb_driver_loaded() -> bool:
    try:
        with open("/proc/modules") as f:
            return "dvb_usb_rtl28xxu" in f.read()
    except OSError:
        return False


UBLOX_VID = "1546"                 # USB vendor ID of u-blox AG
GPSD_IDENT_GRACE = 12.0            # seconds gpsd gets to identify the receiver as u-blox
UBX_POLL_MON_VER = bytes([0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34])  # read-only version query
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
            label = UBLOX_PRODUCTS.get(pid, "u-blox") if vid == UBLOX_VID else ""
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
    return bool(info and info[0] == UBLOX_VID)


@dataclasses.dataclass
class PortInfo:
    device: str
    vid: Optional[str] = None
    pid: Optional[str] = None
    label: str = ""
    usb: bool = False

    @property
    def ublox(self) -> bool:
        return self.vid == UBLOX_VID


def discover_serial_ports() -> List[PortInfo]:
    """Enumerate USB serial ports dynamically: serial.tools.list_ports first, then every
    /dev/ttyACM* and /dev/ttyUSB* node. No path is assumed; ports may appear, vanish or be renumbered."""
    found: Dict[str, PortInfo] = {}
    if serial is not None:
        try:
            from serial.tools import list_ports
            for lp in list_ports.comports():
                if lp.vid is None:
                    continue  # not a USB device (legacy UARTs are only used when named with --gps-serial)
                vid, pid = f"{lp.vid:04x}", f"{(lp.pid or 0):04x}"
                label = UBLOX_PRODUCTS.get(pid, "u-blox") if vid == UBLOX_VID else (lp.product or lp.description or f"{vid}:{pid}")
                found[os.path.realpath(lp.device)] = PortInfo(lp.device, vid, pid, label, True)
        except Exception:  # pragma: no cover - list_ports quirks must never break scanning
            pass
    for dev in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
        real = os.path.realpath(dev)
        if real in found:
            continue
        info = usb_info_for_tty(dev)
        found[real] = PortInfo(dev, info[0], info[1], info[2], True) if info else PortInfo(dev)
    ports = list(found.values())
    ports.sort(key=lambda p: (not p.ublox, not p.device.startswith("/dev/ttyACM"), p.device))
    return ports


def classify_stream(buf: bytes) -> str:
    """'nmea' if a checksum-valid NMEA sentence is present, 'ubx' for a checksum-valid UBX frame, else ''."""
    for line in buf.split(b"\n"):
        text = line.decode("ascii", "ignore").strip()
        if text.startswith("$") and nmea_valid(text):
            return "nmea"
    i = 0
    while True:
        i = buf.find(b"\xb5\x62", i)
        if i < 0 or len(buf) < i + 8:
            return ""
        length = buf[i + 4] | (buf[i + 5] << 8)
        end = i + 6 + length + 2
        if length <= 2048 and len(buf) >= end and ubx_checksum(buf[i + 2:i + 6 + length]) == buf[i + 6 + length:end]:
            return "ubx"
        i += 2


def verify_stream(ser, stop_evt: threading.Event, timeout: float = 4.0) -> str:
    """Passively read until a valid NMEA sentence or UBX frame shows up (writes nothing)."""
    buf = b""
    end = time.monotonic() + timeout
    while time.monotonic() < end and not stop_evt.is_set():
        chunk = ser.read(256)
        if chunk:
            buf = (buf + chunk)[-8192:]
            kind = classify_stream(buf)
            if kind:
                return kind
    return ""


def cdc_acm_state(proc_modules: str = "/proc/modules", sys_module: str = "/sys/module/cdc_acm",
                  release: Optional[str] = None) -> str:
    """State of the cdc_acm kernel driver (u-blox USB GPS modules enumerate as /dev/ttyACM*):
    'loaded' | 'builtin' | 'available' (module exists, not loaded) | 'missing'."""
    try:
        with open(proc_modules) as fh:
            if any(line.split(" ", 1)[0] == "cdc_acm" for line in fh):
                return "loaded"
    except OSError:
        pass
    if os.path.isdir(sys_module):
        return "loaded"
    release = release or os.uname().release
    try:
        with open(f"/lib/modules/{release}/modules.builtin") as fh:
            if "cdc-acm.ko" in fh.read():
                return "builtin"
    except OSError:
        pass
    try:
        if subprocess.run(["modinfo", "-n", "cdc_acm"], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0:
            return "available"
    except OSError:
        pass
    return "missing"


def _priv_auto(cmd: List[str]) -> List[str]:
    """sudo for system changes; never hang on a password prompt when there is no terminal."""
    if os.geteuid() == 0:
        return cmd
    return ["sudo"] + ([] if sys.stdin.isatty() else ["-n"]) + cmd


def ensure_cdc_acm(fix: bool = True) -> Tuple[str, str]:
    state = cdc_acm_state()
    if state in ("loaded", "builtin"):
        return state, f"cdc_acm kernel driver ok ({state})"
    if state == "available":
        if not fix:
            return state, "cdc_acm is not loaded (run: sudo modprobe cdc_acm)"
        if _run(_priv_auto(["modprobe", "cdc_acm"])).returncode == 0:
            _run(_priv_auto(["tee", "/etc/modules-load.d/aeros-cdc_acm.conf"]),
                 input=b"# Written by AERO\xc3\x98S: u-blox USB GPS modules need cdc_acm\ncdc_acm\n",
                 stdout=subprocess.DEVNULL)
            return cdc_acm_state(), "loaded cdc_acm and set it to load at boot"
        return state, "modprobe cdc_acm failed (sudo needed): run it manually"
    return state, ("kernel has no cdc_acm module: reboot if the kernel was just updated, or on Ubuntu "
                   "install linux-modules-extra-$(uname -r)")


def dialout_state() -> Tuple[bool, bool]:
    """(user listed in the dialout group, group active in this login session)."""
    try:
        import grp
        import pwd
    except ImportError:  # pragma: no cover
        return True, True
    if os.geteuid() == 0:
        return True, True
    try:
        g = grp.getgrnam("dialout")
    except KeyError:
        return True, True  # distro without dialout: do not nag
    user = os.environ.get("SUDO_USER") or getpass.getuser()
    try:
        primary = pwd.getpwnam(user).pw_gid
    except KeyError:
        primary = -1
    return (user in g.gr_mem or primary == g.gr_gid), (g.gr_gid in os.getgroups() or os.getegid() == g.gr_gid)


def reexec_with_dialout() -> None:
    """Re-launch this program once under `sg dialout` so a freshly added group works without re-login."""
    if os.environ.get("AEROS_SG") or not shutil.which("sg"):
        return
    env = dict(os.environ, AEROS_SG="1")
    cmd = " ".join(shlex.quote(x) for x in [sys.executable] + sys.argv)
    os.execvpe("sg", ["sg", "dialout", "-c", cmd], env)


def prepare_usb_serial(args: argparse.Namespace, fix: bool = True) -> None:
    """Startup hardware prep for u-blox USB GPS: kernel driver + serial-port permissions."""
    state, msg = ensure_cdc_acm(fix)
    say(f"USB-serial driver: {msg}")
    in_db, in_session = dialout_state()
    user = os.environ.get("SUDO_USER") or getpass.getuser()
    if not in_db:
        if fix and _run(_priv_auto(["usermod", "-aG", "dialout", user])).returncode == 0:
            say(f"added {user} to the 'dialout' group")
            in_db = True
        else:
            say(f"{RED}warning: {user} is not in 'dialout'; serial ports may be unreadable "
                f"(sudo usermod -aG dialout {user}){RESET}")
    if in_db and not in_session:
        blocked = [p.device for p in discover_serial_ports()
                   if p.ublox and not os.access(p.device, os.R_OK | os.W_OK)]
        if blocked:
            say(f"no permission on {', '.join(blocked)} in this session: re-launching with the dialout group")
            reexec_with_dialout()
            say("could not apply the group automatically: log out/in or run `newgrp dialout`")


def ubx_checksum(data: bytes) -> bytes:
    a = b = 0
    for x in data:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return bytes([a, b])


def parse_ubx_mon_ver(buf: bytes) -> Optional[Tuple[str, str]]:
    """Find a checksum-valid UBX-MON-VER reply in buf -> (swVersion, hwVersion), else None."""
    i = 0
    while True:
        i = buf.find(b"\xb5\x62\x0a\x04", i)
        if i < 0 or len(buf) < i + 6:
            return None
        length = buf[i + 4] | (buf[i + 5] << 8)
        end = i + 6 + length + 2
        if length >= 40 and len(buf) >= end and ubx_checksum(buf[i + 2:i + 6 + length]) == buf[i + 6 + length:end]:
            payload = buf[i + 6:i + 6 + length]
            sw = payload[0:30].split(b"\0")[0].decode("ascii", "replace")
            hw = payload[30:40].split(b"\0")[0].decode("ascii", "replace")
            return sw, hw
        i += 2


def ublox_handshake(ser, stop_evt: threading.Event, timeout: float = 3.0) -> Optional[Tuple[str, str]]:
    """Ask the receiver for UBX-MON-VER (a read-only query). Only a u-blox answers with a valid frame."""
    buf = b""
    end = time.monotonic() + timeout
    next_poll = 0.0
    while time.monotonic() < end and not stop_evt.is_set():
        if time.monotonic() >= next_poll:
            ser.write(UBX_POLL_MON_VER)
            ser.flush()
            next_poll = time.monotonic() + 1.0
        chunk = ser.read(512)
        if chunk:
            buf = (buf + chunk)[-4096:]
            res = parse_ubx_mon_ver(buf)
            if res:
                return res
    return None


def find_dump1090(explicit: Optional[str] = None) -> Optional[str]:
    return Dump1090Controller.find_binary(explicit)


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


def install_pymodes() -> bool:
    """pyModeS must stay on the 2.x API (3.x removed it). Retries with --break-system-packages on PEP 668 distros."""
    cmd = [sys.executable, "-m", "pip", "install", "--user", "pyModeS>=2.13,<3"]
    sudo_user = os.environ.get("SUDO_USER")
    if os.geteuid() == 0 and sudo_user and sudo_user != "root":
        cmd = ["sudo", "-u", sudo_user] + cmd  # `sudo ./install.sh`: install for the real user, not for root
    if _run(cmd).returncode == 0:
        return True
    return _run(cmd + ["--break-system-packages"]).returncode == 0


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

    if HAVE_PYMODES:
        say("pyModeS (v2 API) already available")
    elif not install_pymodes():
        say(f"{RED}pyModeS install failed: AERØS will use the decoder's own JSON/SBS output instead{RESET}")

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

    # 2b. kernel driver for u-blox USB GPS (cdc_acm) + persistence
    state, msg = ensure_cdc_acm(True)
    if state == "missing" and ("ubuntu" in [d.ident] + d.like):
        _apt_install(["linux-modules-extra-" + os.uname().release])
        state, msg = ensure_cdc_acm(True)
    say(f"USB-serial driver: {msg}")

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
        self._rejected: Dict[str, Tuple[str, float]] = {}   # ports that failed the u-blox check (re-checked after 120 s)
        self._scan_no = 0
        self._gdev: Dict[str, dict] = {}      # gpsd devices seen this session

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
                self._update(source="fixed", status="FIXED", lat=lat, lon=lon, alt=alt,
                             mode=3, static=True, eph=5.0)
                if self.stop_evt.wait(1.0):
                    return
        if a.gps == "none":
            self._status("DISABLED", "GPS disabled (--gps none)")
            return
        while not self.stop_evt.is_set():
            locked = False
            if a.gps in ("auto", "gpsd"):
                locked = self._gpsd_session()
            if not locked and a.gps in ("auto", "serial"):
                self._scan_no += 1
                locked = self._serial_scan()
            if not locked and a.gps == "gpsd":
                self._status("NO DEVICE", self._note or "gpsd has no GPS")
            self.stop_evt.wait(2.0)  # keep scanning until a responding u-blox is found (also handles hot-plug)

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
        self._gdev = {}
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

    def _gpsd_note_device(self, d: dict) -> None:
        path = d.get("path")
        if not path:
            return
        rec = self._gdev.setdefault(path, {"first": time.monotonic(), "driver": ""})
        if d.get("driver"):
            rec["driver"] = str(d["driver"])

    def _gpsd_verdict(self, path: Optional[str]) -> Optional[bool]:
        """True = u-blox confirmed, False = rejected, None = still identifying."""
        if path is None and len(self._gdev) == 1:
            path = next(iter(self._gdev))
        rec = self._gdev.get(path) if path else None
        if rec is None:
            return None
        if "u-blox" in rec["driver"].lower() or is_ublox(path):
            return True
        if time.monotonic() - rec["first"] > GPSD_IDENT_GRACE:
            return False
        return None

    def _gpsd_gate(self, msg: dict) -> bool:
        """Only data from a gpsd device that is a u-blox receiver may reach the fix state."""
        verdict = self._gpsd_verdict(msg.get("device"))
        if verdict is True:
            return True
        path = msg.get("device") or (next(iter(self._gdev)) if len(self._gdev) == 1 else "?")
        if verdict is False:
            drv = self._gdev.get(path, {}).get("driver") or "unknown"
            self._status("REJECTED", f"gpsd device {path} is '{drv}', not a u-blox receiver (unsupported)")
        else:
            self._status("IDENTIFYING", "waiting for gpsd to identify a u-blox receiver")
        return False

    def _handle_gpsd(self, raw: bytes, name: str) -> bool:
        try:
            msg = json.loads(raw.decode("utf-8", "replace"))
        except ValueError:
            return False
        cls = msg.get("class")
        if cls == "DEVICES":
            devices = msg.get("devices") or []
            for d in devices:
                if isinstance(d, dict):
                    self._gpsd_note_device(d)
            return bool(devices)
        if cls == "DEVICE":
            self._gpsd_note_device(msg)
            return True
        if cls == "TPV":
            if not self._gpsd_gate(msg):
                return True
            mode = int(msg.get("mode", 0) or 0)
            kw = {"mode": mode, "status": GPS_STATUS.get(mode, "NO FIX"), "error": "",
                  "source": f"{name} (u-blox)"}
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
            if not self._gpsd_gate(msg):
                return False
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
        return False

    # -- serial NMEA ---------------------------------------------------------
    @staticmethod
    def _open_serial(port: str, baud: int, timeout: float):
        """Open the port exclusively when pyserial supports it, so we never split bytes with gpsd."""
        try:
            return serial.Serial(port, baud, timeout=timeout, exclusive=True)
        except TypeError:  # pyserial < 3.3
            return serial.Serial(port, baud, timeout=timeout)

    def _serial_scan(self) -> bool:
        """One scan pass over whatever serial ports exist right now. Returns True once a u-blox was
        verified and used (the call blocks while it stays connected)."""
        a = self.args
        n = self._scan_no
        if serial is None:
            self._status("NO DEVICE", "pyserial missing (apt install python3-serial): serial GPS unavailable")
            return False
        if a.gps_serial:
            info = usb_info_for_tty(a.gps_serial)
            cands = [PortInfo(a.gps_serial, info[0], info[1], info[2], True) if info else PortInfo(a.gps_serial)]
            skipped: List[str] = []
        else:
            ports = discover_serial_ports()
            cands = [p for p in ports if p.ublox or (a.scan_bridges and p.usb)]
            skipped = [p.device for p in ports if p not in cands]
        if not cands:
            note = f"scan #{n}: no u-blox USB GPS found"
            if skipped:
                note += f" ({', '.join(skipped)} skipped: not u-blox)"
            self._status("SCANNING", note)
            return False
        for pi in cands:
            if self.stop_evt.is_set():
                return False
            rej = self._rejected.get(pi.device)
            if rej and time.monotonic() - rej[1] < 120:
                self._status("REJECTED", rej[0])
                continue
            if self._serial_session(pi.device, a.gps_baud, pi):
                return True
        return False

    def _serial_session(self, port: str, baud: int, pinfo: PortInfo) -> bool:
        n = self._scan_no
        label = pinfo.label or "serial"
        locked = False
        try:
            with self._open_serial(port, baud, 0.5) as ser:
                self._status("VERIFYING", f"scan #{n}: checking {port} ({label})")
                ident = label
                if not pinfo.ublox:
                    # USB-serial bridge / UART / unknown: prove it is a u-blox with one read-only UBX query
                    res = ublox_handshake(ser, self.stop_evt)
                    if res is None:
                        if self.stop_evt.is_set():
                            return False
                        msg = (f"{port} did not answer UBX-MON-VER: not a u-blox receiver, or its RX pin "
                               f"is not wired. Unsupported")
                        self._rejected[port] = (msg, time.monotonic())
                        self._status("REJECTED", msg)
                        return False
                    ident = f"u-blox sw {res[0]} hw {res[1]}"
                kind = verify_stream(ser, self.stop_evt, 4.0)  # passive: valid NMEA or UBX must arrive
                if not kind:
                    if not self.stop_evt.is_set():
                        self._status("SCANNING", f"scan #{n}: {port} ({label}) is silent (no NMEA/UBX in 4 s); retrying")
                    return False
                ubx_only = kind == "ubx"
                name = f"serial {port}@{baud} ({ident})"
                self._update(source=name, status="NO FIX", mode=0,
                             error="UBX-only output: enable NMEA or use gpsd" if ubx_only else "")
                locked = True
                ser.timeout = 1
                last_ok = time.monotonic()
                while not self.stop_evt.is_set():
                    raw = ser.readline()
                    if raw:
                        line = raw.decode("ascii", "ignore").strip()
                        if nmea_valid(line):
                            last_ok = time.monotonic()
                            self._handle_nmea(line, name)
                    if time.monotonic() - last_ok > 10:
                        self._status("NO DATA", f"{port}: " + ("only UBX frames, no NMEA (enable NMEA output)"
                                                                if ubx_only else "no valid NMEA (check baud/wiring)"))
                return True
        except (OSError, getattr(serial, "SerialException", OSError)) as exc:
            text = str(exc)
            if "ermission" in text:
                self._status("NO ACCESS", f"{port}: permission denied: join the 'dialout' group (re-login or `sg dialout`)")
            elif "lock" in text.lower():
                self._status("SCANNING", f"scan #{n}: {port} is in use (gpsd?): use --gps gpsd or stop gpsd")
            elif locked:
                self._status("DISCONNECTED", f"{port} vanished; scanning for a u-blox again")
            else:
                self._status("SCANNING", f"scan #{n}: {port}: {text[:80]}")
            return locked
        return False

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
# Shared stats + decoder supervision (process control itself lives in airplanes.py)
# ─────────────────────────────────────────────────────────────────────────────
class Stats:
    def __init__(self) -> None:
        self.sdr_state = "INIT"
        self.sdr_detail = ""
        self.feed_kind = "-"
        self.feed_ok = False
        self.rssi_ok = False
        self.msgs_total = 0
        self.msgs_rate = 0.0
        self.log_rows = 0
        self.logging_note = ""


def extra_decoder_args(args: argparse.Namespace) -> List[str]:
    extra: List[str] = []
    if args.gain is not None:
        extra += ["--gain", str(args.gain)]
    if args.device_index is not None:
        extra += ["--device-index", str(args.device_index)]
    if args.dump1090_args:
        extra += shlex.split(args.dump1090_args)
    return extra


class SdrSupervisor(threading.Thread):
    """Keeps the decoder up: starts it (or adopts one already serving :30002), restarts it if it dies."""

    def __init__(self, ctl: Dump1090Controller, stats: Stats, stop: threading.Event, on_started):
        super().__init__(daemon=True, name="sdr")
        self.ctl, self.stats, self.stop_evt, self.on_started = ctl, stats, stop, on_started

    def run(self) -> None:
        ctl = self.ctl
        name = os.path.basename(ctl.binary or "dump1090")
        while not self.stop_evt.is_set():
            self.stats.sdr_state, self.stats.sdr_detail = "STARTING", name
            if not ctl.ensure_running():
                if self.stop_evt.is_set():
                    break
                self.stats.sdr_state = "ERROR"
                self.stats.sdr_detail = explain_sdr_error(ctl.last_error)
                self.stats.feed_ok = False
                self.stop_evt.wait(10.0)
                continue
            external = ctl.process is None
            if external:
                self.stats.sdr_state = "EXTERNAL"
                self.stats.sdr_detail = f"existing decoder on :{ctl.raw_port} [raw]"
            else:
                self.stats.sdr_state, self.stats.sdr_detail = "RUNNING", f"{name} [{ctl.mode}]"
            self.on_started(ctl.mode, external)
            while not self.stop_evt.is_set() and (ctl.is_port_open() if external else ctl.alive()):
                self.stop_evt.wait(1.0)
            if self.stop_evt.is_set():
                break
            code = "" if external else f" ({ctl.process.returncode})" if ctl.process else ""
            self.stats.sdr_state = "STOPPED"
            self.stats.sdr_detail = f"{name} exited{code}; restarting"
            ctl.shutdown()
            self.stop_evt.wait(5.0)
        self.terminate()

    def terminate(self) -> None:
        self.ctl.shutdown()


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

    def _maybe_log(self, ac, fix: GpsFix, now: float) -> None:
        # raw mode merges RSSI from aircraft.json about a second late: give a brand-new aircraft up to 3 s
        # so its first logged row carries the signal level
        if ac.rssi is None and self.stats.rssi_ok and time.time() - ac.first_seen < 3.0:
            self.table.mark_fresh(ac.icao)
            return
        last = self._last.get(ac.icao)
        if last is not None:
            moved_m = haversine_km(last[1], last[2], fix.lat, fix.lon) * 1000.0
            if now - last[0] < self.interval and moved_m < self.move_m:
                return
        dist = None
        d_nm, _ = ac.distance_bearing_from(fix.lat, fix.lon)
        if d_nm is not None:
            dist = d_nm * 1.852
        self.logger.add(Sighting(
            utc=fix.utc_now(), rx_lat=fix.lat, rx_lon=fix.lon, rx_alt=fix.alt, rx_acc=fix.accuracy_m(),
            sats=fix.sats_used, icao=ac.icao, callsign=ac.callsign, squawk=ac.squawk, alt_ft=ac.altitude_ft,
            speed_kt=ac.speed_kt, track=ac.heading_deg, ac_lat=ac.lat, ac_lon=ac.lon, rssi=ac.rssi, dist_km=dist,
        ))
        self._last[ac.icao] = (now, fix.lat, fix.lon)
        self.stats.log_rows = self.logger.rows


# ─────────────────────────────────────────────────────────────────────────────
# Application / curses terminal UI
# ─────────────────────────────────────────────────────────────────────────────
ROW_FMT = "{:<6}  {:<8}  {:>9}  {:>7}  {:>7}  {:>4}  {:>7}  {:>5}"
TABLE_WIDTH = 67
RANGE_STEPS_KM = [10, 25, 50, 100, 200, 300, 450]
ARROWS = ["↑", "↗", "→", "↘", "↓", "↙", "←", "↖"]
ARROWS_ASCII = ["^", "/", ">", "\\", "v", "/", "<", "\\"]


class App:
    def __init__(self, args: argparse.Namespace, binary: Optional[str], distro: Distro):
        self.args, self.binary, self.distro = args, binary, distro
        self.stop = threading.Event()
        self.stats = Stats()
        self.table = AircraftTable()
        self.gps = GpsSupervisor(args, self.stop)
        self.logger: Optional[WardriveLogger] = None
        self.sdr: Optional[SdrSupervisor] = None
        self.ctl: Optional[Dump1090Controller] = None
        self.sources: List = []
        self.source_mode = ""
        self.json_dir = ""
        self._src_lock = threading.Lock()
        self.banner = banner_lines(args.ascii)
        self.sep = "-" if args.ascii else "─"
        self.dot = "|" if args.ascii else "·"
        self.ring = "." if args.ascii else "·"
        self.arrows = ARROWS_ASCII if args.ascii else ARROWS
        self.view = "auto"            # auto | table | radar
        self.range_km: Optional[float] = None  # None = auto-range
        self.A_PURPLE = 0
        self.A_CYAN = 0

    # -- data source management ---------------------------------------------
    def _rssi_dir(self, external: bool) -> str:
        if not external:
            return self.json_dir
        if self.args.json_dir:
            return self.args.json_dir
        return next((d for d in COMMON_JSON_DIRS if os.path.exists(os.path.join(d, "aircraft.json"))), "")

    def _start_sources(self, mode: str, external: bool = False) -> None:
        a = self.args
        with self._src_lock:
            if self.sources and self.source_mode == mode:
                return
            for src in self.sources:
                src.halt.set()
            self.sources = []
            if mode == "raw":
                self.sources.append(AircraftTracker(a.raw_host, a.raw_port, self.table, self.stats, self.stop))
                rssi_dir = self._rssi_dir(external)
                if rssi_dir:
                    self.sources.append(RssiSource(rssi_dir, self.table, self.stats, self.stop))
            elif mode == "json":
                self.sources.append(JsonSource(self._rssi_dir(external) or self.json_dir, self.table, self.stats, self.stop))
            else:
                self.sources.append(SbsSource(a.sbs_host, a.sbs_port, self.table, self.stats, self.stop))
            self.source_mode = mode
            for src in self.sources:
                src.start()

    # -- per-frame data ------------------------------------------------------
    def _collect(self):
        fix = self.gps.snapshot()
        entries = []
        for ac in self.table.snapshot(self.args.max_age):
            dist_km = brg = None
            if fix.has_fix:
                d_nm, brg = ac.distance_bearing_from(fix.lat, fix.lon)
                dist_km = None if d_nm is None else d_nm * 1.852
            entries.append((ac, dist_km, brg))
        key = self.args.sort
        if key == "dist":
            entries.sort(key=lambda e: (e[1] is None, e[1] or 0.0))
        elif key == "rssi":
            entries.sort(key=lambda e: (e[0].rssi is None, -(e[0].rssi or 0.0)))
        else:
            entries.sort(key=lambda e: -e[0].last_seen)
        return fix, entries

    # -- curses drawing ------------------------------------------------------
    def _init_colors(self, scr) -> None:
        curses.start_color()
        if curses.COLORS >= 16:
            purple, cyan, extra = 13, 14, 0      # bright magenta / bright cyan == \033[95m / \033[96m
        else:
            purple, cyan, extra = curses.COLOR_MAGENTA, curses.COLOR_CYAN, curses.A_BOLD
        curses.init_pair(1, purple, curses.COLOR_BLACK)
        curses.init_pair(2, cyan, curses.COLOR_BLACK)
        self.A_PURPLE = curses.color_pair(1) | extra
        self.A_CYAN = curses.color_pair(2) | extra
        scr.bkgd(" ", self.A_CYAN)             # black background everywhere

    @staticmethod
    def _put(scr, y: int, x: int, text: str, attr: int) -> None:
        rows, cols = scr.getmaxyx()
        if y < 0 or y >= rows or x < 0 or x >= cols - 1 or not text:
            return
        try:
            scr.addnstr(y, x, text, cols - x - 1, attr)
        except curses.error:
            pass

    def _effective_range(self, entries) -> Tuple[float, bool]:
        if self.range_km:
            return self.range_km, False
        far = max((d for _, d, _ in entries if d is not None), default=0.0)
        for step in RANGE_STEPS_KM:
            if far * 1.1 <= step:
                return float(step), True
        return float(RANGE_STEPS_KM[-1]), True

    def _step_range(self, entries, direction: int) -> None:
        cur, _ = self._effective_range(entries)
        steps = RANGE_STEPS_KM
        idx = min(range(len(steps)), key=lambda i: abs(steps[i] - cur))
        self.range_km = float(steps[max(0, min(len(steps) - 1, idx + direction))])

    def _draw(self, scr, entries_out: list) -> None:
        scr.erase()
        rows, cols = scr.getmaxyx()
        fix, entries = self._collect()
        entries_out[:] = entries
        st, a = self.stats, self.args
        P, C = self.A_PURPLE, self.A_CYAN
        ALERT = C | curses.A_REVERSE | curses.A_BOLD
        now = time.time()

        # ── pinned header (purple) — redrawn from row 0 on every frame
        y = 0
        if rows >= 24 and cols >= 44:
            for line in self.banner:
                self._put(scr, y, 2, line, P)
                y += 1
            self._put(scr, y, 2, f"{SUBTITLE}  {self.dot}  v{VERSION}", P)
            self._put(scr, y + 1, 2, f"{GITHUB}  {self.dot}  {HANDLE}", P)
            y += 2
        else:
            self._put(scr, y, 1, f"{APP_NAME} {self.dot} {SUBTITLE} {self.dot} {GITHUB} {self.dot} {HANDLE}", P)
            y += 1
        self._put(scr, y, 0, self.sep * (cols - 1), P)
        y += 1

        # ── status block (cyan)
        rssi_src = "json" if (st.rssi_ok or st.feed_kind == "json") else "n/a"
        bad = st.sdr_state in ("ERROR", "STOPPED")
        self._put(scr, y, 1, "SDR  ", C)
        self._put(scr, y, 6, f"{st.sdr_state:<12}", ALERT if bad else C)
        self._put(scr, y, 19, f"{st.sdr_detail}   feed {st.feed_kind} {'OK' if st.feed_ok else '--'}   "
                              f"msgs {st.msgs_total} ({st.msgs_rate:.0f}/s)   rssi {rssi_src}", C)
        y += 1
        sats = "-" if fix.sats_used is None else str(fix.sats_used)
        seen = "-" if fix.sats_seen is None else str(fix.sats_seen)
        hdop = "-" if fix.hdop is None else f"{fix.hdop:.1f}"
        gps_bad = fix.status in ("NO DEVICE", "NO DATA", "DISABLED", "REJECTED", "DISCONNECTED", "NO ACCESS")
        self._put(scr, y, 1, "GPS  ", C)
        self._put(scr, y, 6, f"{fix.status:<12}", ALERT if gps_bad else C)
        self._put(scr, y, 19, f"sats {sats}/{seen}  hdop {hdop}  {fix.source if fix.source != 'none' else ''}"
                              + (f"  [{fix.error}]" if fix.error else ""), C)
        y += 1
        if fix.lat is not None and fix.lon is not None and fix.mode >= 2:
            alt = "-" if fix.alt is None else f"{fix.alt:.0f} m"
            utc = time.strftime("%H:%M:%S", time.gmtime(fix.utc_now()))
            self._put(scr, y, 1, f"POS  {fix.lat:.6f}, {fix.lon:.6f}  alt {alt}  UTC {utc}", C)
        else:
            self._put(scr, y, 1, "POS  no position yet", C)
        y += 1
        name = os.path.basename(self.logger.base) if self.logger else "-"
        note = f"  [{st.logging_note}]" if st.logging_note else ""
        self._put(scr, y, 1, f"LOG  {name}  rows {st.log_rows}  aircraft now {len(entries)} / total "
                             f"{self.table.unique}{note}", C)
        y += 1
        self._put(scr, y, 0, self.sep * (cols - 1), C)
        y += 1

        # ── body: table and/or radar scope
        body_h = rows - y - 1
        both_ok = cols >= 104 and body_h >= 11
        show_table, show_radar = {"table": (True, False), "radar": (False, True)}.get(self.view, (True, both_ok))
        if show_table:
            self._draw_table(scr, y, body_h, fix, entries, now)
        if show_radar:
            x0 = TABLE_WIDTH + 2 if show_table else 0
            self._draw_radar(scr, y, x0, cols - 1 - x0, body_h, fix, entries, now)
        footer = (f" [q] quit & save  [s] save KML  [v] view:{self.view}  [+/-] range  [a] auto range"
                  f"   RSSI=dBFS")
        self._put(scr, rows - 1, 0, footer, C)
        scr.refresh()

    def _draw_table(self, scr, y0: int, body_h: int, fix, entries, now: float) -> None:
        C = self.A_CYAN
        u = self.args.units
        self._put(scr, y0, 0, ROW_FMT.format("ICAO", "CALLSIGN", f"DIST({u})", "ALT(ft)", "SPD(kt)",
                                              "HDG", "RSSI", "SEEN"), C | curses.A_BOLD)
        room = max(0, body_h - 1)
        shown = entries[:room - 1] if len(entries) > room and room > 0 else entries[:room]
        for i, (ac, dist, _) in enumerate(shown):
            d = "-" if dist is None else f"{dist * UNIT_FACTORS[u]:.1f}"
            alt = "GND" if ac.on_ground else ("-" if ac.altitude_ft is None else f"{ac.altitude_ft:.0f}")
            spd = "-" if ac.speed_kt is None else f"{ac.speed_kt:.0f}"
            hdg = "-" if ac.heading_deg is None else f"{ac.heading_deg:.0f}"
            rssi = "n/a" if ac.rssi is None else f"{ac.rssi:.1f}"
            age = f"{max(0, now - ac.last_seen):.0f}s"
            self._put(scr, y0 + 1 + i, 0, ROW_FMT.format(ac.icao, ac.callsign or "-", d, alt, spd, hdg, rssi, age), C)
        if len(entries) > len(shown) and room > 0:
            self._put(scr, y0 + 1 + len(shown), 0, f"  ... +{len(entries) - len(shown)} more ([v] for full view)", C)

    def _draw_radar(self, scr, y0: int, x0: int, w: int, h: int, fix, entries, now: float) -> None:
        C = self.A_CYAN
        u = self.args.units
        rng, auto = self._effective_range(entries)
        self._put(scr, y0, x0, f"RADAR  north-up  range {rng * UNIT_FACTORS[u]:.0f}{u}"
                               f"{' (auto)' if auto else ''}", C | curses.A_BOLD)
        sh = h - 2                      # scope rows (title above, legend below)
        r_rows = (sh - 1) // 2
        r_cols = min(2 * r_rows, (w - 1) // 2 - 1)   # terminal cells are ~2x taller than wide
        if r_rows < 2 or r_cols < 6:
            self._put(scr, y0 + 1, x0, "(terminal too small for the scope: enlarge it or press [v])", C)
            return
        cy, cx = y0 + 1 + r_rows, x0 + w // 2
        n = max(60, int(3 * (r_rows + r_cols)))
        for frac in (1 / 3, 2 / 3, 1.0):
            for i in range(n):
                th = 2 * math.pi * i / n
                self._put(scr, cy - int(round(r_rows * frac * math.cos(th))),
                          cx + int(round(r_cols * frac * math.sin(th))), self.ring, C | curses.A_DIM)
        for label, dy, dx in (("N", -r_rows, 0), ("S", r_rows, 0), ("E", 0, r_cols), ("W", 0, -r_cols)):
            self._put(scr, cy + dy, cx + dx, label, C | curses.A_BOLD)
        sweep = math.radians((now * 90) % 360)   # rotating scope sweep
        for k in range(1, 11):
            f = k / 10.0
            self._put(scr, cy - int(round(r_rows * f * math.cos(sweep))),
                      cx + int(round(r_cols * f * math.sin(sweep))), self.ring, C)
        self._put(scr, cy, cx, "+", C | curses.A_BOLD)
        if not fix.has_fix:
            self._put(scr, cy + 2, cx - 6, "NO GPS FIX", C | curses.A_REVERSE)
            self._put(scr, cy + 3, cx - 6, "scope idle", C)
            return
        plotted = 0
        for ac, dist, brg in entries:
            if dist is None or brg is None or dist > rng:
                continue
            frac = dist / rng
            th = math.radians(brg)
            py = cy - int(round(r_rows * frac * math.cos(th)))
            px = cx + int(round(r_cols * frac * math.sin(th)))
            sym = "*" if ac.heading_deg is None else self.arrows[int(((ac.heading_deg + 22.5) % 360) // 45)]
            self._put(scr, py, px, sym, C | curses.A_BOLD)
            label = (ac.callsign or ac.icao)[:8]
            if px + 2 + len(label) < x0 + w:
                self._put(scr, py, px + 2, label, C)
            plotted += 1
        self._put(scr, y0 + h - 1, x0, f"{plotted} plotted, {len(entries)} heard   rings at "
                                       f"{rng / 3 * UNIT_FACTORS[u]:.0f}/{rng * 2 / 3 * UNIT_FACTORS[u]:.0f}/"
                                       f"{rng * UNIT_FACTORS[u]:.0f}{u}", C)

    def _curses_main(self, scr) -> None:
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        self._init_colors(scr)
        scr.timeout(500)
        entries: list = []
        next_save = time.monotonic() + self.args.save_interval
        views = ["auto", "table", "radar"]
        while not self.stop.is_set():
            self._draw(scr, entries)
            ch = scr.getch()
            if ch in (ord("q"), ord("Q")):
                self.stop.set()
            elif ch in (ord("s"), ord("S")):
                self._safe_save()
            elif ch in (ord("v"), ord("V"), 9):
                self.view = views[(views.index(self.view) + 1) % len(views)]
            elif ch in (ord("+"), ord("=")):
                self._step_range(entries, +1)
            elif ch in (ord("-"), ord("_")):
                self._step_range(entries, -1)
            elif ch in (ord("a"), ord("A")):
                self.range_km = None
            if time.monotonic() >= next_save:
                self._safe_save()
                next_save = time.monotonic() + self.args.save_interval

    def _headless_loop(self) -> None:
        next_save = time.monotonic() + self.args.save_interval
        while not self.stop.wait(10.0):
            fix = self.gps.snapshot()
            print(f"[{time.strftime('%H:%M:%S')}] sdr={self.stats.sdr_state} gps={fix.status} "
                  f"aircraft={len(self.table.snapshot(self.args.max_age))} rows={self.stats.log_rows} "
                  f"msgs/s={self.stats.msgs_rate:.0f} feed={self.stats.feed_kind}", flush=True)
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

        self.ctl = Dump1090Controller(self.binary, self.json_dir, extra_decoder_args(a), a.raw_host, a.raw_port,
                                      use_raw=not a.no_raw, stop_evt=self.stop)
        if a.no_spawn:
            self.stats.sdr_state, self.stats.sdr_detail = "EXTERNAL", "using an already-running decoder"
            if self.ctl.use_raw and self.ctl.is_port_open():
                self._start_sources("raw", external=True)
            elif a.json_dir or self._rssi_dir(True):
                self._start_sources("json", external=True)
            else:
                self._start_sources("sbs")
        else:
            self.sdr = SdrSupervisor(self.ctl, self.stats, self.stop,
                                     lambda mode, external: self._start_sources(mode, external))
            self.sdr.start()

        interactive = sys.stdout.isatty() and sys.stdin.isatty() and not a.no_ui
        try:
            if interactive:
                locale.setlocale(locale.LC_ALL, "")
                try:
                    curses.wrapper(self._curses_main)
                except curses.error as exc:
                    say(f"curses unavailable ({exc}); continuing headless")
                    self._headless_loop()
            else:
                self._headless_loop()
        finally:
            self.stop.set()
            for src in self.sources:
                src.halt.set()
            if self.sdr:
                self.sdr.terminate()
            elif self.ctl:
                self.ctl.shutdown()
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
                   help="GPS source, u-blox receivers only (default: auto = gpsd first, then u-blox USB devices)")
    g.add_argument("--gpsd-host", default="127.0.0.1")
    g.add_argument("--gpsd-port", type=int, default=2947)
    g.add_argument("--gps-serial", metavar="PORT",
                   help="u-blox serial port, e.g. /dev/ttyACM0 or /dev/serial0. Anything that is not a u-blox "
                        "USB device is verified with a read-only UBX-MON-VER poll and rejected if it fails")
    g.add_argument("--gps-baud", type=int, default=9600, help="baud rate for --gps-serial (default 9600)")
    g.add_argument("--fixed-pos", type=fixed_pos_type, metavar="LAT,LON[,ALT_M]",
                   help="use a fixed receiver position instead of GPS (bench testing / static site)")
    d = p.add_argument_group("SDR / decoder")
    d.add_argument("--dump1090", metavar="PATH", help="path to a dump1090-family binary")
    d.add_argument("--no-spawn", action="store_true", help="do not start a decoder; read an existing one")
    d.add_argument("--json-dir", metavar="DIR", help="directory holding aircraft.json (with --no-spawn)")
    d.add_argument("--raw-host", default="127.0.0.1", help="host of the raw-hex feed (default 127.0.0.1)")
    d.add_argument("--raw-port", type=int, default=30002, help="raw-hex feed port decoded with pyModeS (default 30002)")
    d.add_argument("--no-raw", action="store_true",
                   help="skip the pyModeS raw decoder; use the decoder's own aircraft.json/SBS output (native RSSI)")
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
    u.add_argument("--no-ui", action="store_true", help="headless: log only, print a status line every 10 s (no curses)")
    s = p.add_argument_group("maintenance")
    g.add_argument("--scan-bridges", action="store_true",
                   help="also scan non-u-blox USB-serial ports (e.g. /dev/ttyUSB*) and accept a port only if it "
                        "answers a read-only UBX-MON-VER query (writes 8 bytes to every such port)")
    s.add_argument("--no-auto-fix", action="store_true",
                   help="never run sudo modprobe/usermod at startup; only report driver/permission problems")
    s.add_argument("--setup", action="store_true", help="install dependencies, configure udev/gpsd, build dump1090, then exit")
    s.add_argument("--skip-gpsd-config", action="store_true", help="with --setup: leave /etc/default/gpsd alone")
    s.add_argument("--check", action="store_true", help="print a hardware/dependency diagnostic and exit")
    s.add_argument("--version", action="version", version=f"{APP_NAME} {VERSION}")
    return p


def run_check(args: argparse.Namespace, distro: Distro, binary: Optional[str]) -> int:
    say(f"OS          : {distro.friendly} ({distro.pretty}) [{distro.family}]")
    say(f"decoder     : {binary or 'NOT FOUND (run: python3 aeros.py --setup)'}")
    say(f"pyModeS     : {'OK (v2 API)' if HAVE_PYMODES else PYMODES_ERROR + ' -> fallback to decoder JSON/SBS'}")
    devs = rtlsdr_devices()
    say(f"RTL-SDR USB : {', '.join(devs) if devs else 'not detected'}")
    virt = detect_virt()
    if virt:
        say(f"virtual machine: {virt}" + ("" if devs else f"  ->  {VM_USB_HINT}"))
    if dvb_driver_loaded():
        say(f"{RED}dvb_usb_rtl28xxu kernel driver is loaded — it will block the dongle (run --setup){RESET}")
    try:
        socket.create_connection((args.gpsd_host, args.gpsd_port), timeout=2).close()
        say(f"gpsd        : reachable at {args.gpsd_host}:{args.gpsd_port}")
    except OSError as exc:
        say(f"gpsd        : not reachable ({exc})")
    in_db, in_sess = dialout_state()
    say(f"cdc_acm     : {cdc_acm_state()}   dialout: member={in_db} active-this-session={in_sess}   "
        f"pyserial: {'yes' if serial else 'no'}")
    ublox_seen = False
    ports = discover_serial_ports()
    if not ports:
        say("serial ports: none present right now (AERØS keeps scanning at runtime)")
    for pi in ports:
        access = "ok" if os.access(pi.device, os.R_OK | os.W_OK) else "NO ACCESS"
        if pi.ublox:
            ublox_seen = True
            say(f"  {pi.device}: {pi.label} ({pi.vid}:{pi.pid})  -> u-blox, usable, access {access}")
        elif pi.usb:
            say(f"  {pi.device}: {pi.label} ({pi.vid}:{pi.pid})  -> not u-blox: skipped by auto scan "
                f"(--scan-bridges verifies via UBX-MON-VER)")
        else:
            say(f"  {pi.device}: not USB  -> skipped; name it with --gps-serial to verify via UBX-MON-VER")
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
    if args.gps in ("auto", "serial") and not args.fixed_pos:
        prepare_usb_serial(args, fix=not args.no_auto_fix)

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
    if args.no_raw:
        say("raw decoding disabled (--no-raw): using the decoder's own output")
    elif not HAVE_PYMODES:
        say(f"{PYMODES_ERROR}; using the decoder's own JSON/SBS output instead")
    if not args.no_spawn:
        devs = rtlsdr_devices()
        if not devs:
            say(f"{RED}warning: no RTL-SDR dongle seen on USB — will keep retrying{RESET}")
            if detect_virt():
                say(f"running in a VM ({detect_virt()}): {VM_USB_HINT}")
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
