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

# --- USB-serial support for u-blox GPS modules ---------------------------------
# u-blox USB receivers enumerate as CDC-ACM devices (/dev/ttyACM*), which needs the cdc_acm kernel driver.
KVER="$(uname -r)"
cdc_acm_state() {
  if grep -q '^cdc_acm ' /proc/modules 2>/dev/null || [ -d /sys/module/cdc_acm ]; then echo loaded
  elif grep -q 'cdc-acm\.ko' "/lib/modules/${KVER}/modules.builtin" 2>/dev/null; then echo builtin
  elif modinfo -n cdc_acm >/dev/null 2>&1; then echo available
  else echo missing; fi
}

state="$(cdc_acm_state)"
if [ "$state" = "missing" ] && { [ "${ID}" = "ubuntu" ] || [[ " ${ID_LIKE} " == *" ubuntu "* ]]; }; then
  say "cdc_acm not found; trying linux-modules-extra-${KVER}"
  $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y "linux-modules-extra-${KVER}" || true
  state="$(cdc_acm_state)"
fi
case "$state" in
  loaded|builtin) say "cdc_acm kernel driver: ok (${state})" ;;
  available)
    say "loading cdc_acm kernel driver"
    if $SUDO modprobe cdc_acm; then
      printf '# Written by AERØS: u-blox USB GPS modules need cdc_acm\ncdc_acm\n' \
        | $SUDO tee /etc/modules-load.d/aeros-cdc_acm.conf >/dev/null
      say "cdc_acm loaded and set to load at boot"
    else
      say "WARNING: modprobe cdc_acm failed"
    fi ;;
  *) say "WARNING: this kernel (${KVER}) has no cdc_acm module. Reboot if the kernel was just updated." ;;
esac

# serial-port permissions: /dev/ttyACM* and /dev/ttyUSB* belong to group dialout
TARGET_USER="${SUDO_USER:-$(id -un)}"
if [ "$TARGET_USER" != "root" ] && getent group dialout >/dev/null 2>&1; then
  if id -nG "$TARGET_USER" | tr ' ' '\n' | grep -qx dialout; then
    say "user ${TARGET_USER} is already in the dialout group"
  else
    $SUDO usermod -aG dialout "$TARGET_USER" && say "added ${TARGET_USER} to dialout (log out/in to apply; AERØS re-launches itself with sg when needed)"
  fi
fi

# --- full setup (RTL-SDR, gpsd, udev, dump1090 build) -------------------------
say "running full setup"
python3 aeros.py --setup "$@"

say "done. Start with:  python3 aeros.py      (diagnostics: python3 aeros.py --check)"
