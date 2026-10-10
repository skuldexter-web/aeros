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
 passive aircraft tracking · v1.0.0
```

## Features

- Starts and supervises `dump1090-fa`, `readsb`, `dump1090-mutability` or classic `dump1090`
  (auto-detected; restarts if it dies; retries if the dongle is unplugged).
- Reads `aircraft.json` (gives per-aircraft **RSSI**) and falls back to the SBS-1 TCP feed (no RSSI).
- GPS via **gpsd**, **raw NMEA serial** (`/dev/ttyUSB*`, `/dev/ttyACM*`, Pi UART `/dev/serial0`) or a fixed position.
- Live table: ICAO, callsign, Haversine distance from *you*, altitude, speed, heading, RSSI, last seen.
- Logging only happens with a valid GPS fix. Rows are streamed to disk immediately (crash-safe);
  KML is rebuilt every 60 s and on exit (Ctrl+C / `q`).
- OS detection (Ubuntu, Kali, Linux Mint, Raspberry Pi OS, Debian…) and automatic `apt` setup,
  including cloning and compiling dump1090 if none is installed.

## Honest limitations (read these)

| Topic | Reality |
|---|---|
| **RSSI** | Only available in `aircraft.json` mode (dump1090-fa / readsb). The value is **dBFS** (relative to ADC full-scale, e.g. `-18.5`), *not* calibrated dBm. In SBS fallback mode RSSI is empty / `0` in the CSV. |
| **Range** | ADS-B is line-of-sight. A stock whip antenna gives tens of km; a tuned 1090 MHz antenna with a clear horizon gives 150+ km. |
| **GPS lock** | Indoors there is usually no fix, and **nothing is logged without one** (use `--fixed-pos` for a static site). |
| **Privacy** | Your logs are a precise record of where you were and when. They are git-ignored by default. Do not publish them unfiltered. |
| **Legality** | AERØS only *receives* unencrypted public broadcasts. Receiving ADS-B is legal in most countries, but rules differ. You are responsible for local law. |
| **Tested** | Parsing, logging, KML, decoder launch/fallback and the UI were exercised against simulated decoder + gpsd feeds. It has **not** been run on every distro or every GPS/dongle model. Report issues. |

## Hardware

- **RTL-SDR dongle** (RTL2832U; e.g. RTL-SDR Blog V3/V4, Nooelec NESDR) + a 1090 MHz antenna.
- **GPS receiver**, either:
  - **USB** (u-blox 6/7/8 "VK-162/VK-172" style) → appears as `/dev/ttyACM0` or `/dev/ttyUSB0`; or
  - **UART module** on a Raspberry Pi (NEO-6M / NEO-M8N):

    | GPS module | Raspberry Pi |
    |---|---|
    | VCC | 3.3 V (pin 1) — or 5 V (pin 2) if the module has a regulator |
    | GND | GND (pin 6) |
    | TX | GPIO15 / RXD (pin 10) |
    | RX | GPIO14 / TXD (pin 8) |

    Enable the serial *hardware* port and disable the serial login shell (`sudo raspi-config` →
    Interface Options → Serial Port). Then use `--gps-serial /dev/serial0`.

Other serial devices (ESP32 / Arduino dev boards on `/dev/ttyACM*`) can be probed and even reset by
auto-detection. If you have any plugged in, name your GPS explicitly with `--gps-serial`.

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

Python dependencies: only the standard library, plus optional `pyserial` for direct serial GPS
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

Keys: `q` quit & save · `s` write KML now. `Ctrl+C` also saves cleanly.

### CLI arguments

| Argument | Default | Meaning |
|---|---|---|
| `--gps {auto,gpsd,serial,none}` | `auto` | GPS source |
| `--gpsd-host` / `--gpsd-port` | `127.0.0.1` / `2947` | gpsd endpoint |
| `--gps-serial PORT` | – | NMEA port, e.g. `/dev/ttyUSB0`, `/dev/ttyACM0`, `/dev/serial0` |
| `--gps-baud N` | `9600` | baud rate for `--gps-serial` |
| `--fixed-pos LAT,LON[,ALT_M]` | – | fixed receiver position (no GPS) |
| `--dump1090 PATH` | auto | decoder binary (also env `AEROS_DUMP1090`) |
| `--no-spawn` | off | do not start a decoder; read an existing feed |
| `--json-dir DIR` | auto | directory containing `aircraft.json` |
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
| `GPS NO DEVICE` | `python3 aeros.py --check`; start gpsd (`sudo systemctl start gpsd.socket`) or pass `--gps-serial`. |
| `GPS NO FIX` / "not logging" | Move the antenna outdoors/with sky view; cold start can take several minutes. |
| `serial … Permission denied` | `sudo usermod -aG dialout $USER`, then re-login. |
| RSSI shows `n/a` | Decoder fell back to SBS mode; use `dump1090-fa` or `readsb`. |

## Repository layout

```
aeros.py          application (UI, GPS, decoder supervision, logging, setup)
install.sh        OS detection + bootstrap, then `aeros.py --setup`
requirements.txt  optional Python dependency (pyserial)
README.md  LICENSE  .gitignore
```

## Credits

Built by `@s.k.7.l.d` — [github.com/skuldexter-web](https://github.com/skuldexter-web).
Decoding is done by the excellent [dump1090-fa](https://github.com/flightaware/dump1090) project.
Released under the MIT License.
