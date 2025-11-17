# Tundra Pi Tuner (2006 Toyota Tundra, 2UZ-FE VVT-i)

A Raspberry Pi project to **log** and **analyze** engine data on a 2006 Tundra (VVT-i + ACIS) using an ELM327 and (optionally) a wideband O2 sensor via ADS1115.  
Phase 1 & 2 only (no ECU modification).

## Features
- Live dashboard (Flask) + CSV logging
- Core OBD-II PIDs: RPM, speed, load, MAF, timing advance, IAT, fuel trims
- VVT-i context PIDs: **Throttle position**, **Fuel status (open/closed loop)**, **Commanded equivalence ratio λ (PID 0x44)**, **Commanded AFR**
- Wideband AFR via **ADS1115** (0–5 V) with configurable volts→AFR mapping
- Heuristic **ACIS_est** flag (based on RPM & TPS) to mark likely short/long runner operation

> ⚠️ Safety: This project is read-only. It does not modify ECU signals.

---

## Hardware
- Raspberry Pi 4/5 (Bookworm/Bullseye)
- **ELM327 USB** OBD interface (reliable brand; avoid cheap BT clones)
- (Optional) **ADS1115 I²C ADC** for wideband 0–5 V
- (Optional later) MCP4725 DAC + SPDT relay for MAF piggyback (Phase 3, not included here)

## Install
```bash
sudo apt update && sudo apt install -y python3-pip python3-venv
cd tundra-pi-tuner
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Running
```bash
python tundra_pi_tuner.py --port COM3 --hz 2.0 --csv logs\tundra_log.csv --host 0.0.0.0 --port-web 5000
```

## Argumetents
###Core OBD / Logging Arguments
--port

ELM327 device port.
Windows example: COM3
Linux/RPi example: /dev/ttyUSB0

--baud

Baud rate for the ELM327 device.
Typically you leave this unset and python-OBD auto-detects.

--hz

Sampling rate (samples per second).
Default: 2.0
Example:

--hz 5


(samples data 5 times per second)

--csv

Path to save CSV log file. Folder is auto-created.
Example:

--csv logs\tundra.csv

Web Server Arguments
--host

Host interface for Flask/Waitress.
Default: 0.0.0.0 (accessible over LAN)

--port-web

Port for the web dashboard.
Default: 5000
Example:

--port-web 8080

Wideband AFR / ADS1115 Arguments

Only relevant if you pass --ads.

--ads

Enables ADS1115 wideband AFR reader.
If ADS hardware isn’t present, it will warn and continue safely.

--ads-addr

I2C address for ADS1115.
Default: 0x48

--ads-chan

ADS1115 analog input channel.
Choices: A0, A1, A2, A3
Default: A0

--afr-slope

Slope of volts → AFR linear conversion.
Default: 2.0

--afr-intercept

Intercept of volts → AFR conversion.
Default: 10.0

--afr-min

Clamp AFR minimum value.
Example:

--afr-min 10.0

--afr-max

Clamp AFR maximum value.
Example:

--afr-max 20.0

ACIS (Toyota Intake Runner Control) Estimate Arguments
--acis-disable

Turn OFF ACIS estimation logic.

--acis-rpm

RPM threshold to consider ACIS “on.”
Default: 4300.0

--acis-tps

Throttle percent threshold for ACIS “on.”
Default: 35.0

So ACIS is considered ON when BOTH:

RPM ≥ acis_rpm
TPS ≥ acis_tps

Example Commands
Basic Windows run:
python tundra.py --port COM3

Logging + higher sample rate:
python tundra.py --port COM3 --hz 5 --csv logs\tundra.csv

Wideband AFR enabled:
python tundra.py --port COM3 --ads --afr-slope 2.1 --afr-intercept 9.6

Custom web port:
python tundra.py --port COM3 --port-web 8080

## Notes
![alt text](doc/wid.PNG)

- THA --> Air Temperature Signal
- E2G --> Signal GND
- VG --> MAF Signal
- +B --> Power
- E2 --> GND

![alt text](doc/manual.PNG)