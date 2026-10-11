#!/usr/bin/env bash
# AERØS installer — detects the Linux distribution, installs prerequisites, then
# hands over to `aeros.py --setup` (packages, udev/DVB blacklist, gpsd, dump1090 build).
#
# Supported: apt-based systems (Ubuntu, Kali Linux, Linux Mint, Raspberry Pi OS, Debian, Pop!_OS...)
# Usage:     ./install.sh [--skip-gpsd-config]
set -euo pipefail

PURPLE=$'\033[95m'; RED=$'\033[91m'; RESET=$'\033[0m'
say() { printf '%s[AERØS]%s %s\n' "$PURPLE" "$RESET" "$*"; }
die() { printf '%s[AERØS] %s%s\n' "$RED" "$*" "$RESET" >&2; exit 1; }

cd "$(dirname "$(readlink -f "$0")")"

# --- OS detection -------------------------------------------------------------
[ "$(uname -s)" = "Linux" ] || die "AERØS supports Linux only."
ID="unknown"; ID_LIKE=""; PRETTY_NAME="Linux"
if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
fi
NAME_OUT="${PRETTY_NAME}"
[ -f /etc/rpi-issue ] && NAME_OUT="Raspberry Pi OS (${PRETTY_NAME})"
say "detected: ${NAME_OUT}  [id=${ID} like=${ID_LIKE:-none}]"

case " ${ID} ${ID_LIKE} " in
  *" debian "*|*" ubuntu "*|*" kali "*|*" raspbian "*|*" linuxmint "*) ;;
  *) die "unsupported distribution family. Install rtl-sdr, gpsd, python3-serial, git, build tools manually (see README)." ;;
esac
command -v apt-get >/dev/null 2>&1 || die "apt-get not found."

# --- privileges ---------------------------------------------------------------
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  command -v sudo >/dev/null 2>&1 || die "run as root or install sudo."
  SUDO="sudo"
  say "not root — using sudo (password may be requested)"
fi

# --- minimal bootstrap: python + pyserial + git -------------------------------
say "installing bootstrap packages (python3, python3-serial, git)"
$SUDO apt-get update || say "apt-get update reported errors (continuing)"
$SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y python3 python3-serial git \
  || die "could not install bootstrap packages."

chmod +x aeros.py

# --- full setup (RTL-SDR, gpsd, udev, dump1090 build) -------------------------
say "running full setup"
python3 aeros.py --setup "$@"

say "done. Start with:  python3 aeros.py      (diagnostics: python3 aeros.py --check)"