# AERØS — passive aircraft tracking

> Receive-only ADS-B / Mode S wardriving. RTL-SDR + GPS in, geotagged logs out.

**GitHub:** [github.com/skuldexter-web](https://github.com/skuldexter-web) · **Handle:** `@s.k.7.l.d`

AERØS wraps a `dump1090`-family decoder, listens to a GPS receiver, and records every aircraft it
hears together with **where you were standing when you heard it**. A purple banner stays pinned to
the top of the terminal while a cyan live table updates underneath.

```
  ████   ██████  █████    ████    █████
 █    █  █       █    █  █   ██  █
 ██████  █████   █████   █  █ █   ████
 █    █  █       █   █   ██   █       █
 █    █  ██████  █    █   ████   █████
 passive aircraft tracking · v1.1.0
```

## Features

- Starts and supervises `dump1090-fa`, `readsb`, `dump1090-mutability` or classic `dump1090`
  (auto-detected; restarts if it dies; retries if the dongle is unplugged).
- **Raw-hex decoding with pyModeS** (`airplanes.py`): AERØS connects to dump1090's raw stream (`:30002`), CRC-checks each DF17 frame and decodes callsign, altitude, speed, heading and CPR position itself. If pyModeS is missing, or dump1090 cannot serve the raw port, it falls back to the decoder's own `aircraft.json` / SBS output.
- **u-blox GPS modules only** (USB, UART or via gpsd). Other GPS hardware is detected and refused, see below. A fixed position (`--fixed-pos`) is available for bench tests.
- **curses UI**: purple banner pinned on top; everything live is cyan on black. A live table (ICAO, callsign, Haversine distance from *you*, altitude, speed, heading, RSSI, last seen) sits next to a **north-up radar scope** with range rings, a sweep line and heading arrows.
- Logging only happens with a valid GPS fix. Rows are streamed to disk immediately (crash-safe);
  KML is rebuilt every 60 s and on exit (Ctrl+C / `q`).
- OS detection (Ubuntu, Kali, Linux Mint, Raspberry Pi OS, Debian…) and automatic `apt` setup,
  including cloning and compiling dump1090 if none is installed.

## Honest limitations (read these)

| Topic | Reality |
|---|---|
| **RSSI** | dump1090's raw `:30002` stream carries **no signal level**. In raw mode AERØS merges RSSI from the decoder's `aircraft.json` when the decoder writes one (dump1090-fa, readsb); otherwise the column shows `n/a`. `--no-raw` uses the decoder's own output with native RSSI. Values are **dBFS** (relative), not calibrated dBm. |
| **Range** | ADS-B is line-of-sight. A stock whip antenna gives tens of km; a tuned 1090 MHz antenna with a clear horizon gives 150+ km. |
| **GPS hardware** | **u-blox only.** This is a guard against using the wrong device by mistake, not copy protection: the code is MIT-licensed and anyone can edit the check. |
| **GPS lock** | Indoors there is usually no fix, and **nothing is logged without one** (use `--fixed-pos` for a static site). |
| **Privacy** | Your logs are a precise record of where you were and when. They are git-ignored by default. Do not publish them unfiltered. |
| **Legality** | AERØS only *receives* unencrypted public broadcasts. Receiving ADS-B is legal in most countries, but rules differ. You are responsible for local law. |
| **Tested** | Parsing, logging, KML, decoder launch/fallback and the UI were exercised against simulated decoder + gpsd feeds. It has **not** been run on every distro or every GPS/dongle model. Report issues. |

## Hardware

- **RTL-SDR dongle** (RTL2832U; e.g. RTL-SDR Blog V3/V4, Nooelec NESDR) + a 1090 MHz antenna.
- **u-blox GPS receiver**, either:
  - **USB** (u-blox 6/7/8, e.g. VK-172) → appears as `/dev/ttyACM0` (USB vendor `1546`); or
  - **UART module** on a Raspberry Pi (NEO-6M / NEO-7M / NEO-M8N). **Both TX and RX must be wired**, because AERØS proves the module is a u-blox by asking it for its version (see below):

    | GPS module | Raspberry Pi |
    |---|---|
    | VCC | 3.3 V (pin 1) — or 5 V (pin 2) if the module has a regulator |
    | GND | GND (pin 6) |
    | TX | GPIO15 / RXD (pin 10) |
    | RX | GPIO14 / TXD (pin 8) |

    Enable the serial *hardware* port and disable the serial login shell (`sudo raspi-config` →
    Interface Options → Serial Port). Then use `--gps-serial /dev/serial0`.

### How the u-blox-only rule works

| Path | What AERØS does |
|---|---|
| `--gps auto` (default) | Uses gpsd, or any **USB serial device with vendor ID `1546`**. Every other port (ESP32, Arduino, FTDI, CP210x, CNC controllers…) is **never opened and never written to**. |
| gpsd | Accepts data only from a gpsd device whose driver is `u-blox` (or a `1546` USB device). Anything else shows `REJECTED` and no position is used. gpsd gets 12 s to identify the receiver. |
| `--gps-serial PORT` (UART, USB-serial bridge, unknown) | Sends **one read-only `UBX-MON-VER` query** and trusts the port only if a checksum-valid reply comes back. No reply → `REJECTED`, nothing is parsed, and it is not retried. |

The `UBX-MON-VER` query reads the version string and changes no receiver setting. It is the only thing
AERØS ever writes to a GPS, and only for ports you name yourself that are not u-blox USB devices.

### u-blox 7 (VK-172 / VK-162 / G-Mouse style USB modules)

- Shows up as a USB CDC device, normally `/dev/ttyACM0` (USB ID `1546:01a7`). The baud rate is irrelevant
  over USB; for a UART-wired NEO-7M use `--gps-baud 9600` (factory default).
- Factory output is 1 Hz NMEA (GGA, RMC, GSA, GSV, GLL, VTG). AERØS parses GGA/RMC/GSV and accepts both
  `GP` and `GN` talkers, and sums GPS + GLONASS satellites in view.
- Without a fix the module still sends NMEA with empty position fields. AERØS shows `NO FIX` and logs nothing.
  Cold start outdoors is typically under a minute; behind glass or indoors it may never lock.
- `python3 aeros.py --check` prints which port is the u-blox. USB u-blox modules are recognised by vendor ID, so nothing is written to them.
- **ModemManager** (installed by default on Ubuntu, Mint, Kali desktop) can grab `/dev/ttyACM*` and send AT
  commands to it, which garbles NMEA. `--setup` installs `/etc/udev/rules.d/21-aeros-ublox.rules` telling it to
  ignore `1546:*` devices; **unplug and replug the GPS afterwards**.
- Use either gpsd or direct serial on one port, not both. `--gps auto` prefers gpsd.
- AERØS never changes receiver configuration. Your u-blox settings stay untouched (see the rule table above for the one read-only query).

## Installation

```bash
git clone https://github.com/skuldexter-web/AEROS.git
cd AEROS
./install.sh
```

`install.sh` detects your distribution and then runs `python3 aeros.py --setup`, which:

1. installs `rtl-sdr librtlsdr-dev gpsd gpsd-clients python3-pip build-essential git pkg-config libusb-1.0-0-dev libncurses-dev python3-serial` via `apt`;
2. blacklists the DVB-T kernel driver (`dvb_usb_rtl28xxu`) that otherwise steals the dongle, adds a udev rule (`plugdev`) and adds you to `plugdev` + `dialout`;
3. writes `/etc/default/gpsd` (`USBAUTO=true`, `-n`; the original is kept as `/etc/default/gpsd.aeros.bak`) — skip with `--skip-gpsd-config`;
4. clones and builds `flightaware/dump1090` (falls back to `antirez/dump1090`) and installs it as `/usr/local/bin/dump1090-aeros`.

**Log out and back in (or reboot) afterwards** so group membership and the driver blacklist take effect.

Python dependencies: `pyModeS<3` (raw decoding) and optional `pyserial` (direct serial GPS). `--setup` installs pyModeS for you, and `apt` supplies numpy/pyzmq. **pyModeS 3.x removed the API AERØS uses**, so it is pinned below 3 (`pip install -r requirements.txt`). Without it AERØS still runs on the decoder's own output.
(`apt install python3-serial` or `pip install -r requirements.txt`).
Non-apt distributions: install the packages above manually and pass `--dump1090 /path/to/binary`.

Run `python3 aeros.py --check` for a diagnostic of dongle, decoder, gpsd and serial ports.

## Usage

```bash
python3 aeros.py                                   # auto: gpsd first, then scan serial ports
python3 aeros.py --gps serial --gps-serial /dev/ttyUSB0 --gps-baud 9600
python3 aeros.py --gps-serial /dev/serial0         # Raspberry Pi UART GPS
python3 aeros.py --gps gpsd --gpsd-host 192.168.1.20
python3 aeros.py --fixed-pos 52.3702,4.8952,5      # static site, no GPS
python3 aeros.py --no-spawn --json-dir /run/dump1090-fa   # use a decoder that is already running
python3 aeros.py --no-ui --output-dir /srv/aeros   # headless / service
python3 aeros.py --units nm --sort rssi --gain 40
```

Keys: `q` quit & save · `s` write KML now · `v` cycle view (auto / table / radar) · `+` `-` radar range · `a` auto range. `Ctrl+C` also saves cleanly.

The table and scope sit side by side on terminals at least 104 columns wide; on smaller ones use `v` to switch.

### CLI arguments

| Argument | Default | Meaning |
|---|---|---|
| `--gps {auto,gpsd,serial,none}` | `auto` | GPS source (u-blox receivers only) |
| `--gpsd-host` / `--gpsd-port` | `127.0.0.1` / `2947` | gpsd endpoint |
| `--gps-serial PORT` | – | u-blox port, e.g. `/dev/ttyACM0`, `/dev/serial0`; non-u-blox-USB ports must answer `UBX-MON-VER` |
| `--gps-baud N` | `9600` | baud rate for `--gps-serial` |
| `--fixed-pos LAT,LON[,ALT_M]` | – | fixed receiver position (no GPS) |
| `--dump1090 PATH` | auto | decoder binary (also env `AEROS_DUMP1090`) |
| `--no-spawn` | off | do not start a decoder; read an existing feed |
| `--json-dir DIR` | auto | directory containing `aircraft.json` |
| `--raw-host` / `--raw-port` | `127.0.0.1` / `30002` | raw-hex feed decoded with pyModeS |
| `--no-raw` | off | skip pyModeS; use the decoder's own `aircraft.json` / SBS output (native RSSI) |
| `--sbs-host` / `--sbs-port` | `127.0.0.1` / `30003` | SBS-1 feed fallback |
| `--gain DB` / `--device-index N` | decoder default | passed to the decoder |
| `--dump1090-args "…"` | – | extra raw decoder arguments |
| `--output-dir DIR` | `logs` | where `AEROS_<YYYYMMDD_HHMMSS>.*` are written |
| `--log-interval S` | `5` | min seconds between rows per aircraft |
| `--log-move M` | `50` | also log when you moved this many metres |
| `--save-interval S` | `60` | KML rebuild interval |
| `--wigle-type T` | `ADSB` | value of the CSV `Type` column |
| `--units {km,nm,mi}` | `km` | distance units |
| `--sort {dist,seen,rssi}` | `dist` | table order |
| `--max-age S` | `60` | hide aircraft not heard for S seconds |
| `--ascii` / `--no-ui` | off | ASCII-only drawing / headless mode |
| `--setup` / `--skip-gpsd-config` / `--check` / `--no-auto-setup` | – | maintenance |

## Output files

Each run creates three files named `AEROS_<YYYYMMDD_HHMMSS>.*`:

| File | Content |
|---|---|
| `.csv` | WiGLE-1.4-style CSV: `MAC`=ICAO hex, `SSID`=callsign, `AuthMode`=`[ADS-B]`, `FirstSeen` (UTC), `Channel`=1090 (MHz), `RSSI` (dBFS, int), receiver lat/lon/alt/accuracy, `Type`=`ADSB` |
| `.log` | Full detail CSV: receiver position + satellites, ICAO, callsign, squawk, aircraft altitude/speed/track/lat/lon, RSSI, distance (km) |
| `.kml` | Google Earth: receiver observation points + aircraft tracks |

The position in every row is **your receiver's GPS position at the moment of reception**; the
aircraft's own broadcast position is kept in the `.log` / KML tracks.

## Uploading

### WiGLE — please read before uploading

WiGLE's data model is Wi-Fi, Bluetooth, BLE and cellular networks. Its CSV `Type` column has no
ADS-B value. AERØS therefore writes `Type=ADSB`, which WiGLE is **not** expected to accept as a
real network type. Do **not** relabel the rows as `WIFI` / `BT` to force them through: that would
put fake networks (ICAO codes posing as MAC addresses) into a community database. Use `.csv` as a
format reference or for tools that read WiGLE-style files, and the `.kml`/`.log` for your own mapping.
The format here follows WiGLE's public CSV description (two header lines, then rows); check
`github.com/wiglenet/wigle-wifi-wardriving` for the current spec.

### wdgwars.pl

The upload requirements of wdgwars.pl could not be verified while this tool was written, so no
site-specific format is emitted. Open the site's upload page, check which file types and `Type`
values it accepts, and if it takes WiGLE-style CSV try the `.csv` (use `--wigle-type` to match
whatever value the site expects). If it needs another layout, the `.log` file has every field
needed for a trivial conversion script.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `no RTL-SDR dongle found` | Check USB/power; `lsusb` should show `0bda:2838`. |
| `dongle busy: DVB driver loaded` | `python3 aeros.py --setup`, unplug/replug, or `sudo modprobe -r dvb_usb_rtl28xxu`. |
| `USB permission denied` | Re-login after `--setup` (plugdev), or run with `sudo`. |
| `GPS REJECTED` | The device is not a u-blox, or (UART) its RX pin is not wired so it can't answer the version query. Only u-blox modules are supported. |
| `GPS NO DEVICE` | `python3 aeros.py --check`; start gpsd (`sudo systemctl start gpsd.socket`) or pass `--gps-serial`. |
| `GPS NO FIX` / "not logging" | Move the antenna outdoors/with sky view; cold start can take several minutes. |
| `GPS DISCONNECTED … port is in use` | gpsd holds the port: use `--gps gpsd`, or stop gpsd for direct serial. |
| `serial … Permission denied` | `sudo usermod -aG dialout $USER`, then re-login. |
| RSSI shows `n/a` | Raw mode needs the decoder to also write `aircraft.json` (dump1090-fa / readsb do), or use `--no-raw`. |
| `pyModeS not installed` / `3.x installed` | `pip install --user 'pyModeS<3'` (add `--break-system-packages` on Debian 12 / Ubuntu 24.04 if pip refuses). |

## Repository layout

```
aeros.py          application: curses UI + radar, GPS (u-blox only), logging, setup
airplanes.py      decoder side: dump1090 controller, pyModeS raw decoder, aircraft table, JSON/SBS fallbacks
install.sh        OS detection + bootstrap, then `aeros.py --setup`
requirements.txt  optional Python dependency (pyserial)
README.md  LICENSE  .gitignore
```

## Credits

Built by `@s.k.7.l.d` — [github.com/skuldexter-web](https://github.com/skuldexter-web).
The raw decoder (`airplanes.py`) is built on [pyModeS](https://github.com/junzis/pyModeS). The decoding front end is the excellent [dump1090-fa](https://github.com/flightaware/dump1090) project.
Released under the MIT License.
