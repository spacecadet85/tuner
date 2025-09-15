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
