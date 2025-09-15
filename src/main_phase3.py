#!/usr/bin/env python3
"""
Tundra Pi Tuner — Phase 3 (MAF piggyback with safety gates)
- Reads OEM MAF 0–5 V via ADS1115
- Drives adjusted MAF to ECU via MCP4725 DAC
- Controls a bypass relay via GPIO (NC = hard bypass; only energize when SAFE)
- Engages only in conservative closed-loop steady-state with λ≈1, outside ACIS switch band
- Slew-limits DAC output and clamps trims (default ±3%)

Hardware:
- ADS1115: read OEM MAF voltage (tap the MAF signal; high-impedance)
- MCP4725: DAC output to ECU MAF input (through RC filter; recommended 1k + 0.1uF)
- SPDT relay: NC = OEM direct pass-through; NO = DAC (tuned) path
- GPIO: controls relay coil (active HIGH by default; configurable)

NOTE: You are responsible for safe wiring. Start with --dry-run to validate logic without driving the DAC/relay.
"""

import argparse, time, csv, threading, os
from collections import deque
import math

import obd
from flask import Flask, jsonify, render_template_string

# --- I2C / GPIO optionals ---
ADS_OK = False
DAC_OK = False
GPIO_OK = False

try:
    import board, busio
    from adafruit_ads1x15.ads1115 import ADS1115, Mode as ADSMode
    from adafruit_ads1x15.analog_in import AnalogIn
    ADS_OK = True
except Exception:
    ADS_OK = False

try:
    import adafruit_mcp4725
    DAC_OK = True
except Exception:
    DAC_OK = False

# GPIO (works on Pi)
try:
    import RPi.GPIO as GPIO
    GPIO_OK = True
except Exception:
    GPIO_OK = False

# ---------------- UI (same vibe as phase 2) ----------------
HTML = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Tundra Pi Tuner — Phase 3</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { font-family: system-ui, sans-serif; margin: 20px; }
  .grid { display: grid; grid-template-columns: repeat(4, minmax(200px, 1fr)); gap: 12px; }
  .card { padding: 12px; border: 1px solid #ddd; border-radius: 10px; }
  h1 { margin-top: 0; }
  small { color:#666; }
  table { width: 100%; border-collapse: collapse; font-size: 14px; }
  th, td { border-bottom: 1px solid #eee; padding: 6px; text-align: left; }
  .ok{color:#0a0}.warn{color:#a60}.bad{color:#a00}
</style>
</head>
<body>
  <h1>Phase 3 — MAF Piggyback <small id="proto"></small></h1>
  <div class="grid" id="cards"></div>
  <h2>Recent Samples</h2>
  <table>
    <thead>
      <tr>
        <th>t</th><th>RPM</th><th>TPS%</th><th>Load%</th><th>MAF g/s</th><th>OEM MAF V</th><th>DAC Out V</th>
        <th>STFT1%</th><th>LTFT1%</th><th>STFT2%</th><th>LTFT2%</th>
        <th>λ_cmd</th><th>AFR</th><th>λ</th><th>Fuel</th><th>ACIS</th><th>Bypass</th><th>Trim%</th>
      </tr>
    </thead>
    <tbody id="rows"></tbody>
  </table>
<script>
function fmt(x, d=2){return (x===null||x===undefined)?'—':(typeof x==='number'?x.toFixed(d):x);}
async function refresh() {
  const r = await fetch('/api/latest'); const j = await r.json();
  document.getElementById('proto').textContent = j.protocol ? "("+j.protocol+")" : "";
  const c = document.getElementById('cards'); c.innerHTML="";
  const items = [
    ['RPM', j.rpm], ['TPS %', j.throttle_pos], ['Load %', j.load], ['MAF g/s', j.maf],
    ['OEM MAF V', j.maf_volts], ['DAC Out V', j.dac_volts], ['Trim %', j.trim_pct],
    ['Cmd λ', j.lmbd_cmd], ['AFR', j.afr], ['λ', j.lmbd],
    ['Fuel', j.fuel_status], ['ACIS', j.acis_est?'ON':'OFF'], ['Bypass', j.bypass?'ON':'OFF'],
    ['Safe', j.safe?'YES':'NO']
  ];
  for (const [k,v] of items){
    const div = document.createElement('div'); div.className='card';
    const val = (typeof v === 'number') ? v.toFixed(2) : (v ?? '—');
    div.innerHTML = `<strong>${k}</strong><div>${val}</div>`; c.appendChild(div);
  }
  const rows = document.getElementById('rows'); rows.innerHTML="";
  (j.recent||[]).forEach(row=>{
    const tr = document.createElement('tr');
    tr.innerHTML =
      `<td>${fmt(row.t)}</td><td>${fmt(row.rpm)}</td><td>${fmt(row.throttle_pos)}</td><td>${fmt(row.load)}</td><td>${fmt(row.maf)}</td>`+
      `<td>${fmt(row.maf_volts)}</td><td>${fmt(row.dac_volts)}</td>`+
      `<td>${fmt(row.stft1)}</td><td>${fmt(row.ltft1)}</td><td>${fmt(row.stft2)}</td><td>${fmt(row.ltft2)}</td>`+
      `<td>${fmt(row.lmbd_cmd)}</td><td>${fmt(row.afr)}</td><td>${fmt(row.lmbd)}</td>`+
      `<td>${row.fuel_status??'—'}</td><td>${row.acis_est?'ON':'OFF'}</td><td>${row.bypass?'ON':'OFF'}</td><td>${fmt(row.trim_pct)}</td>`;
    rows.appendChild(tr);
  });
}
setInterval(refresh, 800); refresh();
</script>
</body>
</html>
"""

# --- Helpers to access python-OBD commands safely ---
def _cmd(name):
    return getattr(obd.commands, name, None)

PIDS = {
    "rpm": _cmd("RPM"),
    "speed": _cmd("SPEED"),
    "load": _cmd("ENGINE_LOAD"),
    "maf": _cmd("MAF"),
    "timing_advance": _cmd("TIMING_ADVANCE"),
    "iat": _cmd("INTAKE_TEMP"),
    "stft1": _cmd("SHORT_FUEL_TRIM_1"),
    "ltft1": _cmd("LONG_FUEL_TRIM_1"),
    "stft2": _cmd("SHORT_FUEL_TRIM_2"),
    "ltft2": _cmd("LONG_FUEL_TRIM_2"),
    "throttle_pos": _cmd("THROTTLE_POS"),
    "lmbd_cmd": _cmd("EQUIV_RATIO"),
    "fuel_status": _cmd("FUEL_STATUS"),
}

STOICH_AFR = 14.7


# ----------------- ADS1115 Reader -----------------
class ADSReader:
    """Generic ADS1115 single-ended channel reader at ~10 Hz averaged output."""
    def __init__(self, enabled, addr=0x48, channel='A0', gain=2/3):
        self.enabled = enabled and ADS_OK
        self.addr = addr
        self.channel = channel
        self.gain = gain
        self._adc = None
        self._ain = None
        self._lock = threading.Lock()
        self._volts = None
        self._stop = threading.Event()

    def start(self):
        if not self.enabled: return
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self._adc = ADS1115(i2c, address=self.addr)
            self._adc.mode = ADSMode.CONTINUOUS
            self._adc.gain = self.gain
            chmap = {'A0':0,'A1':1,'A2':2,'A3':3}
            pin = chmap.get(self.channel.upper(), 0)
            self._ain = AnalogIn(self._adc, getattr(self._adc, f'P{pin}'))
            t = threading.Thread(target=self._loop, daemon=True)
            t.start()
        except Exception:
            self.enabled = False

    def _loop(self):
        acc, n = 0.0, 0
        last = time.time()
        while not self._stop.is_set():
            try:
                v = float(self._ain.voltage)
                acc += v; n += 1
            except Exception:
                pass
            now = time.time()
            if now - last >= 0.1:
                avg = (acc/n) if n else None
                with self._lock: self._volts = avg
                acc, n, last = 0.0, 0, now
            time.sleep(0.01)

    def stop(self): self._stop.set()

    def read(self):
        with self._lock:
            return self._volts


# ----------------- MCP4725 DAC Driver -----------------
class DACDriver:
    def __init__(self, enabled, addr=0x60, vref=5.00, dry_run=False):
        self.enabled = enabled and DAC_OK
        self.addr = addr
        self.vref = vref
        self._dac = None
        self._last_v = None
        self.dry = dry_run

    def start(self):
        if not self.enabled: return
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self._dac = adafruit_mcp4725.MCP4725(i2c, address=self.addr)
            self._last_v = 0.0
        except Exception:
            self.enabled = False

    def write_volts(self, vout):
        v = max(0.0, min(self.vref, float(vout if vout is not None else 0.0)))
        self._last_v = v
        if not self.enabled or self.dry: return v
        # Convert to 12-bit code
        code = int((v / self.vref) * 4095.0 + 0.5)
        try:
            self._dac.raw_value = code
        except Exception:
            pass
        return v

    @property
    def last_v(self):
        return self._last_v


# ----------------- Relay Control -----------------
class Relay:
    def __init__(self, enabled, pin=18, active_high=True, dry_run=False):
        self.enabled = enabled and GPIO_OK
        self.pin = pin
        self.active_high = active_high
        self.state = False  # False = de-energized (bypass)
        self.dry = dry_run
        if self.enabled:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW if self.active_high else GPIO.HIGH)
            except Exception:
                self.enabled = False

    def set(self, energized: bool):
        self.state = bool(energized)
        if not self.enabled or self.dry: return
        try:
            if self.active_high:
                GPIO.output(self.pin, GPIO.HIGH if energized else GPIO.LOW)
            else:
                GPIO.output(self.pin, GPIO.LOW if energized else GPIO.HIGH)
        except Exception:
            pass

    def cleanup(self):
        if self.enabled:
            try: GPIO.cleanup(self.pin)
            except Exception: pass


# ----------------- Utility -----------------
def _to_float(val):
    try:
        return float(val.magnitude if hasattr(val, "magnitude") else val)
    except Exception:
        return None

def _fuel_status_to_text(value):
    try:
        if isinstance(value, (list, tuple)):
            return ",".join(str(x).split(".")[-1] for x in value)
        return str(value).split(".")[-1]
    except Exception:
        return None


# ----------------- Controller / Logger -----------------
class Controller:
    def __init__(self, args):
        self.args = args
        self.conn = None
        self.protocol = None

        # Readers
        self.wb_reader = ADSReader(args.ads, addr=args.ads_addr, channel=args.ads_chan, gain=2/3) if args.ads else None  # AFR volts
        self.maf_reader = ADSReader(args.maf_read, addr=args.maf_ads_addr, channel=args.maf_ads_chan, gain=2/3) if args.maf_read else None

        # DAC + Relay
        self.dac = DACDriver(args.maf_write, addr=args.dac_addr, vref=args.dac_vref, dry_run=args.dry_run) if args.maf_write else None
        self.relay = Relay(args.relay_enable, pin=args.relay_pin, active_high=(not args.relay_active_low), dry_run=args.dry_run) if args.relay_enable else None

        # State
        self.latest = {}
        self.recent = deque(maxlen=25)
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()

        # MAF trim control
        self.trim_pct = 0.0
        self.target_volts = 0.0
        self.last_out_v = 0.0
        self.last_time = time.time()

    # ---- MAF trim strategy ----
    def compute_trim_pct(self, row):
        """
        Conservative strategy:
        - Only when closed loop AND commanded lambda ~ 1.00 (±0.01 default)
        - TPS within cruise band [--tps-min, --tps-max]
        - Not in ACIS switch region (rpm between --acis-rpm-avoid-1 and --acis-rpm-avoid-2)
        - AFR available and roughly stoich (optional)
        Returns a % trim bounded to ±maf_trim_max.
        """
        if not self.args.maf_write:  # nothing to do
            return 0.0, False

        fuel = row.get("fuel_status") or ""
        closed_loop = ("Closed" in fuel) or ("ClosedLoop" in fuel)

        lmbd_cmd = row.get("lmbd_cmd")
        if lmbd_cmd is None: return 0.0, False
        if abs(lmbd_cmd - 1.0) > self.args.lambda_tol:  # not targeting stoich
            return 0.0, False

        tps = (row.get("throttle_pos") or 0.0)
        if not (self.args.tps_min <= tps <= self.args.tps_max):
            return 0.0, False

        rpm = (row.get("rpm") or 0.0)
        if (self.args.acis_rpm_avoid_1 <= rpm <= self.args.acis_rpm_avoid_2):
            return 0.0, False

        # Optional AFR sanity: if available and far from stoich, don't engage
        afr = row.get("afr")
        if afr is not None and abs((afr / STOICH_AFR) - 1.0) > (self.args.lambda_tol * 2):
            return 0.0, False

        # Simple rule: nudge trims toward 0 based on LTFT average (banks 1/2)
        lt1, lt2 = row.get("ltft1"), row.get("ltft2")
        trims = [x for x in (lt1, lt2) if (x is not None)]
        if not trims:
            return 0.0, False
        avg_lt = sum(trims) / len(trims)  # % (positive = ECU adding fuel)
        # If ECU is adding fuel (+), increase MAF a bit (+) so ECU doesn't add as much.
        k = self.args.trim_gain  # small gain
        raw = k * avg_lt
        # Clamp
        trim = max(-self.args.maf_trim_max, min(self.args.maf_trim_max, raw))
        return trim, (closed_loop)

    def smooth_slew(self, desired_v):
        now = time.time()
        dt = max(1e-3, now - self.last_time)
        self.last_time = now
        max_step = self.args.slew_v_per_s * dt
        dv = desired_v - self.last_out_v
        if abs(dv) > max_step:
            desired_v = self.last_out_v + (max_step if dv > 0 else -max_step)
        self.last_out_v = desired_v
        return desired_v

    def connect(self):
        self.conn = obd.OBD(portstr=self.args.port, baudrate=self.args.baud, fast=False, timeout=2)
        self.protocol = str(self.conn.protocol_name()) if self.conn.is_connected() else None
        if self.wb_reader: self.wb_reader.start()
        if self.maf_reader: self.maf_reader.start()
        if self.dac: self.dac.start()
        # Ensure relay is bypassed at start
        if self.relay: self.relay.set(False)

    def query(self, cmd):
        if not cmd: return None
        try:
            r = self.conn.query(cmd)
            if r.is_null(): return None
            return r.value
        except Exception:
            return None

    def sample_once(self):
        t = round(time.time(), 2)
        row = {"t": t}

        # OBD
        if self.conn and self.conn.is_connected():
            for k, cmd in PIDS.items():
                if k in ("lmbd_cmd", "fuel_status"): continue
                row[k] = _to_float(self.query(cmd))
            row["lmbd_cmd"] = _to_float(self.query(PIDS["lmbd_cmd"]))
            row["fuel_status"] = _fuel_status_to_text(self.query(PIDS["fuel_status"]))
        else:
            for k in PIDS.keys(): row[k] = None

        # AFR volts -> AFR
        if self.wb_reader:
            wb_v = self.wb_reader.read()
            row["wb_volts"] = wb_v
            afr = None if wb_v is None else (self.args.afr_slope * wb_v + self.args.afr_intercept)
            row["afr"] = afr
            row["lmbd"] = (afr / STOICH_AFR) if (afr and afr > 0) else None
        else:
            row["wb_volts"] = row["afr"] = row["lmbd"] = None

        # MAF volts (OEM)
        maf_v = self.maf_reader.read() if self.maf_reader else None
        row["maf_volts"] = maf_v

        # ACIS estimate
        acis_on = False
        rpm = row.get("rpm") or 0.0
        tps = row.get("throttle_pos") or 0.0
        if (rpm >= self.args.acis_rpm) and (tps >= self.args.acis_tps):
            acis_on = True
        row["acis_est"] = acis_on

        # MAF trim calc
        trim_pct, safe_cond = self.compute_trim_pct(row)
        row["trim_pct"] = trim_pct

        # Decide bypass state (False=bypass ON (relay de-energized), True=inline)
        safe = (self.args.maf_write and self.args.relay_enable and safe_cond and (maf_v is not None))
        row["safe"] = safe
        bypass = (not safe)  # default to bypass whenever not safe
        row["bypass"] = bypass

        # Output voltage target
        dac_v = None
        if self.dac:
            if not bypass:
                # desired output = maf_v * (1 + trim_pct/100)
                desired = max(0.0, min(self.args.dac_vref, maf_v * (1.0 + trim_pct/100.0)))
                desired = self.smooth_slew(desired)
                dac_v = self.dac.write_volts(desired)
                # energize relay to go inline
                if self.relay: self.relay.set(True)
            else:
                # ensure bypass (relay off), but still park DAC at current maf_v to minimize step if engaged
                if self.relay: self.relay.set(False)
                park = 0.0 if maf_v is None else max(0.0, min(self.args.dac_vref, maf_v))
                dac_v = self.dac.write_volts(park)
        row["dac_volts"] = dac_v

        with self.lock:
            self.latest = row
            self.recent.append(row)

    def run(self):
        period = 1.0 / float(self.args.hz)
        while not self.stop_evt.is_set():
            self.sample_once()
            time.sleep(period)

    def stop(self):
        self.stop_evt.set()
        try:
            if self.relay: self.relay.set(False)  # fall back to bypass
        except Exception:
            pass
        if self.dac: pass
        if self.maf_reader: self.maf_reader.stop()
        if self.wb_reader: self.wb_reader.stop()
        try:
            if self.conn: self.conn.close()
        except Exception:
            pass
        if self.relay: self.relay.cleanup()


# ----------------- Flask -----------------
app = Flask(__name__)
ctl: Controller = None

@app.route("/")
def root():
    return render_template_string(HTML)

@app.route("/api/latest")
def api_latest():
    with ctl.lock:
        out = dict(ctl.latest)
        out["recent"] = list(ctl.recent)
        out["protocol"] = ctl.protocol
    return jsonify(out)


# ----------------- Main -----------------
def main():
    p = argparse.ArgumentParser(description="Tundra Pi Tuner — Phase 3 MAF Piggyback")
    # General
    p.add_argument('--port', help='ELM327 port, e.g., /dev/ttyUSB0')
    p.add_argument('--baud', type=int)
    p.add_argument('--hz', type=float, default=2.0)
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port-web', type=int, default=5000)
    p.add_argument('--dry-run', action='store_true', help='Do not drive DAC/relay (logic only)')

    # AFR reader (optional, but helpful)
    p.add_argument('--ads', action='store_true', help='Enable ADS1115 for wideband AFR')
    p.add_argument('--ads-addr', type=lambda x:int(x,0), default=0x48)
    p.add_argument('--ads-chan', default='A0', choices=['A0','A1','A2','A3'])
    p.add_argument('--afr-slope', type=float, default=2.0)
    p.add_argument('--afr-intercept', type=float, default=10.0)

    # OEM MAF volts reader
    p.add_argument('--maf-read', action='store_true', help='Read OEM MAF voltage via ADS1115')
    p.add_argument('--maf-ads-addr', type=lambda x:int(x,0), default=0x48)
    p.add_argument('--maf-ads-chan', default='A1', choices=['A0','A1','A2','A3'])

    # DAC output for ECU MAF
    p.add_argument('--maf-write', action='store_true', help='Enable MCP4725 DAC output for MAF')
    p.add_argument('--dac-addr', type=lambda x:int(x,0), default=0x60)
    p.add_argument('--dac-vref', type=float, default=5.00)

    # Relay control
    p.add_argument('--relay-enable', action='store_true', help='GPIO control of bypass relay')
    p.add_argument('--relay-pin', type=int, default=18, help='BCM pin number')
    p.add_argument('--relay-active-low', action='store_true', help='Relay energized when GPIO LOW (default is active-high)')

    # Safety / gating params
    p.add_argument('--lambda-tol', type=float, default=0.01, help='|λ_cmd-1.0| tolerance for enabling')
    p.add_argument('--tps-min', type=float, default=8.0)
    p.add_argument('--tps-max', type=float, default=35.0)
    p.add_argument('--acis-rpm', type=float, default=4300.0, help='ACIS est ON above this w/ TPS≥acis_tps')
    p.add_argument('--acis-tps', type=float, default=35.0)
    p.add_argument('--acis-rpm-avoid-1', type=float, default=4100.0, help='Avoid band start (don’t engage)')
    p.add_argument('--acis-rpm-avoid-2', type=float, default=4700.0, help='Avoid band end (don’t engage)')

    # Trim behavior
    p.add_argument('--maf-trim-max', type=float, default=3.0, help='Max ±% MAF trim')
    p.add_argument('--trim-gain', type=float, default=0.15, help='Scale factor from LTFT avg to % trim (small)')

    # Output dynamics
    p.add_argument('--slew-v-per-s', type=float, default=0.20, help='Max DAC volt change per second')

    args = p.parse_args()

    global ctl
    ctl = Controller(args)
    ctl.connect()

    # Web server (blocking)
    from waitress import serve
    serve(app, host=args.host, port=args.port_web, threads=4)

    # On exit
    ctl.stop()


if __name__ == "__main__":
    main()
