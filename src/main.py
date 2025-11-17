#!/usr/bin/env python3
import argparse, time, csv, threading, os
from collections import deque
import obd
from flask import Flask, jsonify, render_template_string

# Optional ADS1115 (for wideband AFR). Imported lazily
try:
    import board, busio
    from adafruit_ads1x15.ads1115 import ADS1115, Mode
    from adafruit_ads1x15.analog_in import AnalogIn
    ADS_OK = True
except Exception:
    ADS_OK = False

HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Tundra Pi Tuner — Live</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: system-ui, sans-serif; margin: 20px; }
    .grid { display: grid; grid-template-columns: repeat(4, minmax(180px, 1fr)); gap: 12px; }
    .card { padding: 12px; border: 1px solid #ddd; border-radius: 10px; }
    h1 { margin-top: 0; }
    small { color: #666; }
    table { width: 100%; border-collapse: collapse; font-size: 14px; }
    th, td { border-bottom: 1px solid #eee; padding: 6px; text-align: left; }
    .chip { display:inline-block; padding:2px 8px; border-radius: 999px; background:#eee; font-size:12px; }
  </style>
</head>
<body>
  <h1>Tundra Pi Tuner <small id="proto"></small></h1>
  <div class="grid" id="cards"></div>
  <h2>Recent Samples</h2>
  <table>
    <thead>
      <tr>
        <th>t</th><th>RPM</th><th>TPS %</th><th>Speed</th><th>Load %</th>
        <th>MAF g/s</th><th>Adv °</th><th>IAT °C</th>
        <th>STFT1 %</th><th>LTFT1 %</th><th>STFT2 %</th><th>LTFT2 %</th>
        <th>Cmd λ</th><th>Cmd AFR</th><th>AFR</th><th>λ</th><th>WB V</th>
        <th>MPG inst</th><th>MPG avg</th>
        <th>Fuel</th><th>ACIS</th>
      </tr>
    </thead>
    <tbody id="rows"></tbody>
  </table>
<script>
function fmt(x, digits=2) {
  return (x===null||x===undefined) ? '—' : (typeof x==='number'? x.toFixed(digits): x);
}
async function refresh() {
  const r = await fetch('/api/latest'); const j = await r.json();
  document.getElementById('proto').textContent = j.protocol ? "("+j.protocol+")" : "";
  const c = document.getElementById('cards'); c.innerHTML = "";
  const fields = [
    ['RPM', j.rpm],
    ['TPS %', j.throttle_pos],
    ['Speed mph', j.speed],
    ['Load %', j.load],
    ['MAF g/s', j.maf],
    ['Timing °', j.timing_advance],
    ['IAT °C', j.iat],
    ['Cmd λ', j.lmbd_cmd],
    ['Cmd AFR', j.afr_cmd],
    ['AFR', j.afr],
    ['Lambda', j.lmbd],
    ['WB Volts', j.wb_volts],
    ['MPG inst', j.mpg_inst],
    ['MPG avg', j.mpg_avg],
    ['Fuel', j.fuel_status],
    ['ACIS est', j.acis_est ? 'ON' : 'OFF']
  ];
  for (const [k,v] of fields) {
    const div = document.createElement('div'); div.className='card';
    const val = (typeof v === 'number') ? v.toFixed(2) : (v ?? '—');
    div.innerHTML = `<strong>${k}</strong><div>${val}</div>`; c.appendChild(div);
  }
  const rows = document.getElementById('rows'); rows.innerHTML = "";
  (j.recent || []).forEach(row => {
    const tr = document.createElement('tr');
    tr.innerHTML =
      `<td>${fmt(row.t)}</td><td>${fmt(row.rpm)}</td><td>${fmt(row.throttle_pos)}</td><td>${fmt(row.speed)}</td><td>${fmt(row.load)}</td>` +
      `<td>${fmt(row.maf)}</td><td>${fmt(row.timing_advance)}</td><td>${fmt(row.iat)}</td>` +
      `<td>${fmt(row.stft1)}</td><td>${fmt(row.ltft1)}</td><td>${fmt(row.stft2)}</td><td>${fmt(row.ltft2)}</td>` +
      `<td>${fmt(row.lmbd_cmd)}</td><td>${fmt(row.afr_cmd)}</td><td>${fmt(row.afr)}</td><td>${fmt(row.lmbd)}</td><td>${fmt(row.wb_volts)}</td>` +
      `<td>${fmt(row.mpg_inst)}</td><td>${fmt(row.mpg_avg)}</td>` +
      `<td>${row.fuel_status ?? '—'}</td><td>${row.acis_est ? 'ON' : 'OFF'}</td>`;
    rows.appendChild(tr);
  });
}
setInterval(refresh, 750); refresh();
</script>
</body>
</html>
"""

# Helper to access python-OBD commands safely
def _cmd(name):
    return getattr(obd.commands, name, None)

# Core + VVT-i related PIDs
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
    "lmbd_cmd": _cmd("EQUIV_RATIO"),   # Commanded equivalence ratio (λ)
    "fuel_status": _cmd("FUEL_STATUS"),
}

STOICH_AFR = 14.7
# Approximate gasoline mass per gallon [grams/gal]
GASOLINE_GPG = 2810.0

# --- Wideband AFR reader via ADS1115 ---
class AfrReader:
    def __init__(self, enabled=False, i2c_addr=0x48, channel='A0', slope=2.0, intercept=10.0, afr_min=None, afr_max=None):
        self.enabled = enabled and ADS_OK
        self.addr = i2c_addr
        self.channel = channel
        self.slope = slope
        self.intercept = intercept
        self.afr_min = afr_min
        self.afr_max = afr_max
        self._adc = None
        self._ain = None
        self._lock = threading.Lock()
        self._volts = None
        self._afr = None
        self._stop = threading.Event()

    def start(self):
        if not self.enabled:
            return
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self._adc = ADS1115(i2c, address=self.addr)
            self._adc.mode = Mode.CONTINUOUS
            self._adc.gain = 2/3  # ±6.144 V
            chmap = {'A0': 0, 'A1': 1, 'A2': 2, 'A3': 3}
            pin = chmap.get(self.channel.upper(), 0)
            self._ain = AnalogIn(self._adc, getattr(self._adc, f'P{pin}'))
            t = threading.Thread(target=self._loop, daemon=True)
            t.start()
        except Exception:
            self.enabled = False

    def _loop(self):
        samples, acc_v = 0, 0.0
        last_emit = time.time()
        while not self._stop.is_set():
            try:
                v = float(self._ain.voltage)
            except Exception:
                v = None
            if v is not None:
                samples += 1
                acc_v += v
            now = time.time()
            if now - last_emit >= 0.1:
                avg_v = (acc_v / samples) if samples else None
                afr = None
                if avg_v is not None:
                    afr = self.slope * avg_v + self.intercept
                    if self.afr_min is not None: afr = max(self.afr_min, afr)
                    if self.afr_max is not None: afr = min(self.afr_max, afr)
                with self._lock:
                    self._volts, self._afr = avg_v, afr
                samples, acc_v = 0, 0.0
                last_emit = now
            time.sleep(0.01)

    def stop(self):
        self._stop.set()

    def read(self):
        with self._lock:
            return self._volts, self._afr

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

# --- Logger ---
class Logger:
    def __init__(self, port=None, baud=None, csv_path=None, afr_reader: AfrReader=None, hz=2.0,
                 acis_enable=True, acis_rpm_thresh=4300.0, acis_tps_thresh=35.0):
        self.conn = None
        self.port = port
        self.baud = baud
        self.csv_path = csv_path
        self.protocol = None
        self.lock = threading.Lock()
        self.latest = {}
        self.recent = deque(maxlen=25)
        self._stop = threading.Event()
        self.hz = hz
        self.afr_reader = afr_reader
        self.acis_enable = acis_enable
        self.acis_rpm_thresh = acis_rpm_thresh
        self.acis_tps_thresh = acis_tps_thresh

        # MPG integration state
        self._last_ts = None
        self.total_dist_mi = 0.0
        self.total_fuel_gal = 0.0

        fields = ['t'] + list(PIDS.keys()) + ['afr_cmd','afr','lmbd','wb_volts','fuel_status','acis_est','mpg_inst','mpg_avg']
        if csv_path:
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
            self.csv_file = open(csv_path, 'w', newline='')
            self.writer = csv.DictWriter(self.csv_file, fieldnames=fields)
            self.writer.writeheader()
        else:
            self.csv_file = None
            self.writer = None

    def connect(self):
        self.conn = obd.OBD(portstr=self.port, baudrate=self.baud, fast=False, timeout=2)
        self.protocol = str(self.conn.protocol_name()) if self.conn.is_connected() else None
        if self.afr_reader: self.afr_reader.start()

    def _query(self, cmd):
        if not cmd: return None
        try:
            r = self.conn.query(cmd)
            if r.is_null(): return None
            return r.value
        except Exception:
            return None

    def sample_once(self):
        now = time.time()
        row = {"t": round(now,2)}

        if self.conn and self.conn.is_connected():
            for key, cmd in PIDS.items():
                if key in ("lmbd_cmd","fuel_status"): continue
                val = self._query(cmd)
                row[key] = _to_float(val)

            # Commanded λ + AFR
            lmbd_val = self._query(PIDS["lmbd_cmd"])
            lmbd_cmd = _to_float(lmbd_val)
            row["lmbd_cmd"] = lmbd_cmd
            row["afr_cmd"] = (STOICH_AFR*lmbd_cmd) if lmbd_cmd is not None else None

            # Fuel status
            fs_val = self._query(PIDS["fuel_status"])
            row["fuel_status"] = _fuel_status_to_text(fs_val)
        else:
            for k in PIDS.keys():
                row[k] = None
            row["afr_cmd"], row["fuel_status"], row["lmbd_cmd"] = None, None, None

        # Wideband AFR
        wb_v, afr = (None,None)
        if self.afr_reader:
            wb_v, afr = self.afr_reader.read()
        lmbd = (afr/STOICH_AFR) if (afr is not None and afr>0) else None
        row['afr'], row['lmbd'], row['wb_volts'] = afr, lmbd, wb_v

        # ACIS estimate
        acis_on = False
        if self.acis_enable:
            rpm = row.get("rpm") or 0.0
            tps = row.get("throttle_pos") or 0.0
            if (rpm >= self.acis_rpm_thresh) and (tps >= self.acis_tps_thresh):
                acis_on = True
        row["acis_est"] = acis_on

        # MPG calculations (instantaneous + running average)
        mpg_inst, mpg_avg = None, None
        speed_mph = row.get("speed")
        maf_gps = row.get("maf")  # grams per second
        # Effective AFR: prefer actual AFR, then commanded, then stoich
        afr_effective = None
        if row.get("afr") is not None and row["afr"] > 0:
            afr_effective = row["afr"]
        elif row.get("afr_cmd") is not None and row["afr_cmd"] > 0:
            afr_effective = row["afr_cmd"]
        else:
            afr_effective = STOICH_AFR

        if self._last_ts is not None and speed_mph is not None and maf_gps is not None and afr_effective and afr_effective > 0:
            dt = now - self._last_ts
            if dt > 0:
                # Fuel mass flow [g/s] -> [gal/s] -> [gal]
                fuel_gps = maf_gps / afr_effective    # g/s fuel
                fuel_gal = fuel_gps * dt / GASOLINE_GPG

                # Distance [mi] over dt
                dist_mi = speed_mph * dt / 3600.0

                # Instantaneous MPG (based on current flow + speed)
                fuel_gph = fuel_gps * 3600.0 / GASOLINE_GPG if fuel_gps > 0 else None
                if fuel_gph and fuel_gph > 0 and speed_mph > 1.0:
                    mpg_inst = speed_mph / fuel_gph

                # Accumulate for running average
                self.total_dist_mi += dist_mi
                self.total_fuel_gal += fuel_gal
                if self.total_fuel_gal > 0:
                    mpg_avg = self.total_dist_mi / self.total_fuel_gal

        self._last_ts = now
        row["mpg_inst"] = mpg_inst
        row["mpg_avg"] = mpg_avg

        with self.lock:
            self.latest = row
            self.recent.append(row)
        if self.writer:
            self.writer.writerow(row); self.csv_file.flush()

    def run(self):
        period = 1.0/float(self.hz)
        while not self._stop.is_set():
            self.sample_once()
            time.sleep(period)

    def stop(self):
        self._stop.set()
        try: 
            if self.conn: self.conn.close()
        except Exception: pass
        if self.afr_reader: self.afr_reader.stop()
        if self.csv_file: self.csv_file.close()

# --- Flask app ---
app = Flask(__name__)
logger = None

@app.route('/')
def root():
    return render_template_string(HTML)

@app.route('/api/latest')
def api_latest():
    with logger.lock:
        out = dict(logger.latest)
        out['recent'] = list(logger.recent)
        out['protocol'] = logger.protocol
    return jsonify(out)

def main():
    parser = argparse.ArgumentParser(description="Tundra Pi Tuner — OBD + AFR + VVT-i context")
    parser.add_argument('--port', help='ELM327 port, e.g. /dev/ttyUSB0 or COM3')
    parser.add_argument('--baud', type=int, help='ELM327 baudrate (usually auto)')
    parser.add_argument('--hz', type=float, default=2.0, help='Sample rate (Hz)')
    parser.add_argument('--csv', help='CSV log path (optional)')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port-web', type=int, default=5000)
    # AFR
    parser.add_argument('--ads', action='store_true', help='Enable ADS1115 wideband input')
    parser.add_argument('--ads-addr', type=lambda x: int(x,0), default=0x48, help='ADS1115 I2C address')
    parser.add_argument('--ads-chan', default='A0', choices=['A0','A1','A2','A3'], help='ADS1115 channel')
    parser.add_argument('--afr-slope', type=float, default=2.0, help='Volts→AFR slope')
    parser.add_argument('--afr-intercept', type=float, default=10.0, help='Volts→AFR intercept')
    parser.add_argument('--afr-min', type=float, default=None)
    parser.add_argument('--afr-max', type=float, default=None)
    # ACIS heuristic
    parser.add_argument('--acis-disable', action='store_true')
    parser.add_argument('--acis-rpm', type=float, default=4300.0)
    parser.add_argument('--acis-tps', type=float, default=35.0)
    args = parser.parse_args()

    afr_reader=None
    if args.ads:
        if not ADS_OK:
            print("ADS1115 not available (check I2C + adafruit-circuitpython-ads1x15)")
        afr_reader = AfrReader(enabled=True, i2c_addr=args.ads_addr, channel=args.ads_chan,
                               slope=args.afr_slope, intercept=args.afr_intercept,
                               afr_min=args.afr_min, afr_max=args.afr_max)

    global logger
    logger = Logger(port=args.port, baud=args.baud, csv_path=args.csv, afr_reader=afr_reader, hz=args.hz,
                    acis_enable=(not args.acis_disable), acis_rpm_thresh=args.acis_rpm, acis_tps_thresh=args.acis_tps)
    logger.connect()
    t = threading.Thread(target=logger.run, daemon=True); t.start()

    try:
        from waitress import serve
        serve(app, host=args.host, port=args.port_web, threads=4)
    finally:
        logger.stop()

if __name__ == '__main__':
    main()
