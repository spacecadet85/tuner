#!/usr/bin/env python3
"""
ELM327 Command-Line TUI

Features:
- Connects to ELM327
- Auto-detects OBD-II protocol
- Arrow-key menu
- Number-key menu shortcuts
- Displays live sensors
- Reads fault codes
- Clears fault codes
- Displays vehicle info
- Displays adapter info
- Allows raw ELM/OBD commands

Install:
    pip install -r requirements.txt

Run:
    python elm327_tui.py --port COM5
    python elm327_tui.py --port /dev/ttyUSB0

Controls:
    Arrow Up/Down  - move menu
    Enter          - select
    Number keys    - select menu item
    Q              - quit/back
    R              - refresh
"""

import argparse
import sys
import time
from datetime import datetime

import serial
from serial.tools import list_ports

try:
    import curses
except ImportError:
    print("This program requires curses.")
    print("On Windows install:")
    print("  pip install windows-curses")
    sys.exit(1)


PROTOCOL_NUMBERS = {
    "0": "Automatic",
    "1": "SAE J1850 PWM",
    "2": "SAE J1850 VPW",
    "3": "ISO 9141-2",
    "4": "ISO 14230-4 KWP 5 baud",
    "5": "ISO 14230-4 KWP fast",
    "6": "ISO 15765-4 CAN 11-bit 500k",
    "7": "ISO 15765-4 CAN 29-bit 500k",
    "8": "ISO 15765-4 CAN 11-bit 250k",
    "9": "ISO 15765-4 CAN 29-bit 250k",
    "A": "SAE J1939 CAN 29-bit 250k",
    "B": "User1 CAN",
    "C": "User2 CAN",
}


SENSOR_PIDS = {
    "Engine Load": {
        "cmd": "0104",
        "unit": "%",
        "decode": lambda b: round(b[0] * 100.0 / 255.0, 1),
    },
    "Coolant Temp": {
        "cmd": "0105",
        "unit": "C",
        "decode": lambda b: b[0] - 40,
    },
    "Short Fuel Trim B1": {
        "cmd": "0106",
        "unit": "%",
        "decode": lambda b: round((b[0] - 128) * 100.0 / 128.0, 1),
    },
    "Long Fuel Trim B1": {
        "cmd": "0107",
        "unit": "%",
        "decode": lambda b: round((b[0] - 128) * 100.0 / 128.0, 1),
    },
    "Short Fuel Trim B2": {
        "cmd": "0108",
        "unit": "%",
        "decode": lambda b: round((b[0] - 128) * 100.0 / 128.0, 1),
    },
    "Long Fuel Trim B2": {
        "cmd": "0109",
        "unit": "%",
        "decode": lambda b: round((b[0] - 128) * 100.0 / 128.0, 1),
    },
    "Fuel Pressure": {
        "cmd": "010A",
        "unit": "kPa",
        "decode": lambda b: b[0] * 3,
    },
    "Intake MAP": {
        "cmd": "010B",
        "unit": "kPa",
        "decode": lambda b: b[0],
    },
    "Engine RPM": {
        "cmd": "010C",
        "unit": "rpm",
        "decode": lambda b: int(((b[0] * 256) + b[1]) / 4),
    },
    "Vehicle Speed": {
        "cmd": "010D",
        "unit": "mph",
        "decode": lambda b: round(b[0] * 0.621371, 1),
    },
    "Timing Advance": {
        "cmd": "010E",
        "unit": "deg",
        "decode": lambda b: round((b[0] / 2.0) - 64.0, 1),
    },
    "Intake Air Temp": {
        "cmd": "010F",
        "unit": "C",
        "decode": lambda b: b[0] - 40,
    },
    "MAF Airflow": {
        "cmd": "0110",
        "unit": "g/s",
        "decode": lambda b: round(((b[0] * 256) + b[1]) / 100.0, 2),
    },
    "Throttle Position": {
        "cmd": "0111",
        "unit": "%",
        "decode": lambda b: round(b[0] * 100.0 / 255.0, 1),
    },
    "Runtime Since Start": {
        "cmd": "011F",
        "unit": "sec",
        "decode": lambda b: (b[0] * 256) + b[1],
    },
    "Fuel Level": {
        "cmd": "012F",
        "unit": "%",
        "decode": lambda b: round(b[0] * 100.0 / 255.0, 1),
    },
    "Barometric Pressure": {
        "cmd": "0133",
        "unit": "kPa",
        "decode": lambda b: b[0],
    },
    "Control Module Voltage": {
        "cmd": "0142",
        "unit": "V",
        "decode": lambda b: round(((b[0] * 256) + b[1]) / 1000.0, 3),
    },
    "Commanded Equivalence Ratio": {
        "cmd": "0144",
        "unit": "lambda",
        "decode": lambda b: round(((b[0] * 256) + b[1]) / 32768.0, 3),
    },
    "Relative Throttle Position": {
        "cmd": "0145",
        "unit": "%",
        "decode": lambda b: round(b[0] * 100.0 / 255.0, 1),
    },
    "Ambient Air Temp": {
        "cmd": "0146",
        "unit": "C",
        "decode": lambda b: b[0] - 40,
    },
    "Commanded Throttle Actuator": {
        "cmd": "014C",
        "unit": "%",
        "decode": lambda b: round(b[0] * 100.0 / 255.0, 1),
    },
}


class ELM327:
    def __init__(self, port, baudrate=38400, timeout=1.5, debug=False):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.debug = debug
        self.ser = None
        self.protocol_name = "Unknown"
        self.protocol_number = "Unknown"
        self.protocol_description = "Unknown"

    def connect(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, cmd, delay=0.2):
        cmd = cmd.strip()

        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port is not open")

        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\r").encode("ascii", errors="ignore"))
        self.ser.flush()
        time.sleep(delay)

        raw = ""
        start = time.time()

        while True:
            c = self.ser.read(1).decode(errors="ignore")

            if c:
                raw += c
                if ">" in raw:
                    break

            if time.time() - start > self.timeout:
                break

        return self.clean_response(raw, cmd)

    @staticmethod
    def clean_response(raw, sent_cmd=""):
        lines = []
        raw = raw.replace("\r", "\n")

        for line in raw.split("\n"):
            line = line.strip()

            if not line:
                continue

            if line == ">":
                continue

            if line.endswith(">"):
                line = line[:-1].strip()

            if not line:
                continue

            if sent_cmd and line.upper() == sent_cmd.upper():
                continue

            lines.append(line)

        return lines

    def initialize(self):
        commands = [
            ("ATZ", 1.0),
            ("ATE0", 0.2),
            ("ATL0", 0.2),
            ("ATS0", 0.2),
            ("ATH0", 0.2),
            ("ATAT2", 0.2),
            ("ATAL", 0.2),
            ("ATSP0", 0.5),
        ]

        for cmd, delay in commands:
            self.send(cmd, delay)

    def detect_protocol(self):
        self.send("ATSP0", 0.5)
        self.send("0100", 1.0)

        name = self.send("ATDP", 0.3)
        number = self.send("ATDPN", 0.3)

        self.protocol_name = " | ".join(name) if name else "Unknown"

        raw_number = number[0].strip().upper() if number else "Unknown"
        self.protocol_number = raw_number

        clean_number = raw_number
        if clean_number.startswith("A") and len(clean_number) > 1:
            clean_number = clean_number[1:]

        self.protocol_description = PROTOCOL_NUMBERS.get(clean_number, "Unknown")

        if clean_number in PROTOCOL_NUMBERS and clean_number != "0":
            self.send(f"ATSP{clean_number}", 0.5)

    def query_obd(self, cmd):
        response = self.send(cmd, 0.35)
        return response

    def query_pid_bytes(self, cmd):
        response = self.query_obd(cmd)
        expected_mode = int(cmd[0:2], 16) + 0x40
        expected_pid = int(cmd[2:4], 16)

        for line in response:
            cleaned = line.upper().replace(" ", "")

            if "NO" in cleaned or "DATA" in cleaned:
                continue

            if "SEARCHING" in cleaned:
                continue

            if "?" in cleaned:
                continue

            target = f"{expected_mode:02X}{expected_pid:02X}"
            idx = cleaned.find(target)

            if idx < 0:
                continue

            payload = cleaned[idx + 4:]

            try:
                return [
                    int(payload[i:i + 2], 16)
                    for i in range(0, len(payload), 2)
                    if len(payload[i:i + 2]) == 2
                ]
            except ValueError:
                continue

        return None


def list_ports_available():
    ports = list_ports.comports()

    if not ports:
        print("No serial ports found.")
        return

    print("Available serial ports:")
    for p in ports:
        print(f"  {p.device} - {p.description}")


def safe_addstr(stdscr, y, x, text, attr=0):
    max_y, max_x = stdscr.getmaxyx()

    if y < 0 or y >= max_y:
        return

    if x < 0 or x >= max_x:
        return

    available = max_x - x - 1
    if available <= 0:
        return

    stdscr.addstr(y, x, str(text)[:available], attr)


def draw_header(stdscr, title, elm):
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()

    safe_addstr(stdscr, 0, 0, "=" * (max_x - 1), curses.A_BOLD)
    safe_addstr(stdscr, 1, 2, title, curses.A_BOLD)
    safe_addstr(
        stdscr,
        2,
        2,
        f"Port: {elm.port} | Baud: {elm.baudrate} | Protocol: {elm.protocol_name}",
    )
    safe_addstr(stdscr, 3, 0, "=" * (max_x - 1), curses.A_BOLD)


def wait_for_key(stdscr):
    safe_addstr(stdscr, curses.LINES - 2, 2, "Press any key to continue...")
    stdscr.refresh()
    stdscr.getch()


def main_menu(stdscr, elm):
    menu_items = [
        "Live Sensors",
        "Fault Codes",
        "Clear Fault Codes",
        "Vehicle Info",
        "Adapter Info",
        "Raw Command",
        "Exit",
    ]

    selected = 0

    while True:
        draw_header(stdscr, "ELM327 Vehicle Interface", elm)

        safe_addstr(stdscr, 5, 2, "Use arrow keys, Enter, or number keys.", curses.A_DIM)
        safe_addstr(stdscr, 6, 2, "Q exits.", curses.A_DIM)

        for i, item in enumerate(menu_items):
            y = 8 + i
            number = i + 1
            prefix = f"{number}. "

            if i == selected:
                safe_addstr(stdscr, y, 4, prefix + item, curses.A_REVERSE)
            else:
                safe_addstr(stdscr, y, 4, prefix + item)

        stdscr.refresh()
        key = stdscr.getch()

        if key in (ord("q"), ord("Q")):
            break

        if key == curses.KEY_UP:
            selected = (selected - 1) % len(menu_items)

        elif key == curses.KEY_DOWN:
            selected = (selected + 1) % len(menu_items)

        elif key in (curses.KEY_ENTER, 10, 13):
            action = menu_items[selected]
            if not run_action(stdscr, elm, action):
                break

        elif ord("1") <= key <= ord(str(len(menu_items))):
            selected = key - ord("1")
            action = menu_items[selected]
            if not run_action(stdscr, elm, action):
                break


def run_action(stdscr, elm, action):
    if action == "Live Sensors":
        live_sensors_screen(stdscr, elm)
    elif action == "Fault Codes":
        fault_codes_screen(stdscr, elm)
    elif action == "Clear Fault Codes":
        clear_codes_screen(stdscr, elm)
    elif action == "Vehicle Info":
        vehicle_info_screen(stdscr, elm)
    elif action == "Adapter Info":
        adapter_info_screen(stdscr, elm)
    elif action == "Raw Command":
        raw_command_screen(stdscr, elm)
    elif action == "Exit":
        return False

    return True


def live_sensors_screen(stdscr, elm):
    stdscr.nodelay(True)

    try:
        while True:
            draw_header(stdscr, "Live Sensors", elm)

            safe_addstr(
                stdscr,
                5,
                2,
                "R = refresh | Q = back | Auto-refreshing once per loop",
                curses.A_DIM,
            )

            y = 7
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            safe_addstr(stdscr, y, 2, f"Updated: {timestamp}")
            y += 2

            for name, info in SENSOR_PIDS.items():
                cmd = info["cmd"]
                unit = info["unit"]

                try:
                    raw_bytes = elm.query_pid_bytes(cmd)

                    if raw_bytes is None:
                        value = "N/A"
                    else:
                        value = info["decode"](raw_bytes)
                        value = f"{value} {unit}"

                except Exception as e:
                    value = f"ERROR: {e}"

                safe_addstr(stdscr, y, 4, f"{name:<30} {value}")
                y += 1

                if y >= curses.LINES - 3:
                    break

            stdscr.refresh()

            for _ in range(10):
                key = stdscr.getch()

                if key in (ord("q"), ord("Q")):
                    return

                if key in (ord("r"), ord("R")):
                    break

                time.sleep(0.1)

    finally:
        stdscr.nodelay(False)


def fault_codes_screen(stdscr, elm):
    draw_header(stdscr, "Fault Codes", elm)

    safe_addstr(stdscr, 5, 2, "Reading stored diagnostic trouble codes...")
    stdscr.refresh()

    stored = elm.query_obd("03")
    pending = elm.query_obd("07")
    permanent = elm.query_obd("0A")

    stored_codes = decode_dtc_response(stored, "43")
    pending_codes = decode_dtc_response(pending, "47")
    permanent_codes = decode_dtc_response(permanent, "4A")

    y = 7

    y = draw_code_group(stdscr, y, "Stored Codes", stored_codes, stored)
    y += 1
    y = draw_code_group(stdscr, y, "Pending Codes", pending_codes, pending)
    y += 1
    y = draw_code_group(stdscr, y, "Permanent Codes", permanent_codes, permanent)

    wait_for_key(stdscr)


def draw_code_group(stdscr, y, title, codes, raw_response):
    safe_addstr(stdscr, y, 2, title, curses.A_BOLD)
    y += 1

    safe_addstr(stdscr, y, 4, f"Raw: {' | '.join(raw_response) if raw_response else 'N/A'}", curses.A_DIM)
    y += 1

    if not codes:
        safe_addstr(stdscr, y, 4, "No codes found.")
        y += 1
    else:
        for code in codes:
            safe_addstr(stdscr, y, 4, code)
            y += 1

    return y


def decode_dtc_response(lines, response_prefix):
    codes = []

    for line in lines:
        cleaned = line.upper().replace(" ", "")

        idx = cleaned.find(response_prefix)
        if idx < 0:
            continue

        data = cleaned[idx + 2:]

        try:
            bytes_list = [
                int(data[i:i + 2], 16)
                for i in range(0, len(data), 2)
                if len(data[i:i + 2]) == 2
            ]
        except ValueError:
            continue

        for i in range(0, len(bytes_list) - 1, 2):
            a = bytes_list[i]
            b = bytes_list[i + 1]

            if a == 0 and b == 0:
                continue

            codes.append(decode_single_dtc(a, b))

    return sorted(set(codes))


def decode_single_dtc(a, b):
    first_char_lookup = ["P", "C", "B", "U"]
    first_char = first_char_lookup[(a & 0xC0) >> 6]
    first_digit = (a & 0x30) >> 4
    second_digit = a & 0x0F
    third_digit = (b & 0xF0) >> 4
    fourth_digit = b & 0x0F

    return f"{first_char}{first_digit}{second_digit:X}{third_digit:X}{fourth_digit:X}"


def clear_codes_screen(stdscr, elm):
    selected = 1
    options = ["No", "Yes"]

    while True:
        draw_header(stdscr, "Clear Fault Codes", elm)

        safe_addstr(stdscr, 5, 2, "WARNING: This clears stored codes and may reset readiness monitors.")
        safe_addstr(stdscr, 6, 2, "Do you want to send Mode 04 clear command?")

        for i, option in enumerate(options):
            attr = curses.A_REVERSE if i == selected else 0
            safe_addstr(stdscr, 8 + i, 4, option, attr)

        stdscr.refresh()
        key = stdscr.getch()

        if key in (ord("q"), ord("Q")):
            return

        if key == curses.KEY_UP:
            selected = (selected - 1) % len(options)

        elif key == curses.KEY_DOWN:
            selected = (selected + 1) % len(options)

        elif key in (10, 13, curses.KEY_ENTER):
            if options[selected] == "No":
                return

            draw_header(stdscr, "Clear Fault Codes", elm)
            safe_addstr(stdscr, 5, 2, "Sending clear command...")
            stdscr.refresh()

            response = elm.query_obd("04")

            safe_addstr(stdscr, 7, 2, f"Response: {' | '.join(response) if response else 'N/A'}")
            wait_for_key(stdscr)
            return


def vehicle_info_screen(stdscr, elm):
    draw_header(stdscr, "Vehicle Info", elm)

    queries = [
        ("Supported Mode 09 PIDs", "0900"),
        ("VIN", "0902"),
        ("Calibration ID", "0904"),
        ("Calibration Verification Number", "0906"),
        ("ECU Name", "090A"),
    ]

    y = 5

    for label, cmd in queries:
        response = elm.query_obd(cmd)
        decoded = decode_mode_09(cmd, response)

        safe_addstr(stdscr, y, 2, label, curses.A_BOLD)
        y += 1

        safe_addstr(stdscr, y, 4, f"Command: {cmd}")
        y += 1

        safe_addstr(stdscr, y, 4, f"Raw: {' | '.join(response) if response else 'N/A'}", curses.A_DIM)
        y += 1

        safe_addstr(stdscr, y, 4, f"Decoded: {decoded}")
        y += 2

        if y >= curses.LINES - 4:
            break

    wait_for_key(stdscr)


def decode_mode_09(cmd, response):
    if not response:
        return "N/A"

    if cmd not in ("0902", "0904", "0906", "090A"):
        return "See raw response"

    chars = []

    for line in response:
        cleaned = line.upper().replace(" ", "")

        idx = cleaned.find("49")
        if idx < 0:
            continue

        payload = cleaned[idx:]

        try:
            byte_values = [
                int(payload[i:i + 2], 16)
                for i in range(0, len(payload), 2)
                if len(payload[i:i + 2]) == 2
            ]
        except ValueError:
            continue

        if len(byte_values) < 3:
            continue

        data_bytes = byte_values[3:]

        for value in data_bytes:
            if 32 <= value <= 126:
                chars.append(chr(value))

    text = "".join(chars).strip()

    if not text:
        return "N/A"

    return text


def adapter_info_screen(stdscr, elm):
    draw_header(stdscr, "Adapter Info", elm)

    queries = [
        ("Firmware Version", "ATI"),
        ("Device Description", "AT@1"),
        ("Device Identifier", "AT@2"),
        ("Input Voltage", "ATRV"),
        ("Current Protocol Name", "ATDP"),
        ("Current Protocol Number", "ATDPN"),
        ("Programmable Parameters", "ATPPS"),
    ]

    y = 5

    for label, cmd in queries:
        response = elm.send(cmd, 0.4)
        safe_addstr(stdscr, y, 2, f"{label:<30} {cmd:<8} {' | '.join(response) if response else 'N/A'}")
        y += 1

        if y >= curses.LINES - 4:
            break

    y += 1
    safe_addstr(stdscr, y, 2, f"Decoded Protocol: {elm.protocol_description}")

    wait_for_key(stdscr)


def raw_command_screen(stdscr, elm):
    curses.echo()
    stdscr.nodelay(False)

    try:
        while True:
            draw_header(stdscr, "Raw Command", elm)

            safe_addstr(stdscr, 5, 2, "Enter raw ELM327/OBD command.")
            safe_addstr(stdscr, 6, 2, "Examples: ATI, ATRV, ATDP, 010C, 03")
            safe_addstr(stdscr, 7, 2, "Type Q to go back.")

            safe_addstr(stdscr, 9, 2, "Command: ")
            stdscr.refresh()

            cmd = stdscr.getstr(9, 11, 32).decode(errors="ignore").strip()

            if not cmd:
                continue

            if cmd.upper() == "Q":
                return

            response = elm.send(cmd, 0.6)

            safe_addstr(stdscr, 11, 2, "Response:", curses.A_BOLD)

            y = 12
            if response:
                for line in response:
                    safe_addstr(stdscr, y, 4, line)
                    y += 1
            else:
                safe_addstr(stdscr, y, 4, "N/A")

            wait_for_key(stdscr)

    finally:
        curses.noecho()


def startup_screen(stdscr, elm):
    stdscr.erase()
    safe_addstr(stdscr, 2, 2, "ELM327 Vehicle Interface", curses.A_BOLD)
    safe_addstr(stdscr, 4, 2, f"Connecting to {elm.port} at {elm.baudrate} baud...")
    stdscr.refresh()

    elm.connect()

    safe_addstr(stdscr, 5, 2, "Initializing adapter...")
    stdscr.refresh()

    elm.initialize()

    safe_addstr(stdscr, 6, 2, "Detecting protocol...")
    stdscr.refresh()

    elm.detect_protocol()

    safe_addstr(stdscr, 8, 2, f"Protocol: {elm.protocol_name}")
    safe_addstr(stdscr, 9, 2, f"Protocol Number: {elm.protocol_number}")
    safe_addstr(stdscr, 10, 2, f"Decoded: {elm.protocol_description}")
    safe_addstr(stdscr, 12, 2, "Press any key to continue...")
    stdscr.refresh()
    stdscr.getch()


def curses_main(stdscr, args):
    curses.curs_set(0)
    stdscr.keypad(True)

    elm = ELM327(
        port=args.port,
        baudrate=args.baud,
        timeout=args.timeout,
        debug=args.debug,
    )

    try:
        startup_screen(stdscr, elm)
        main_menu(stdscr, elm)

    finally:
        elm.close()


def parse_args():
    parser = argparse.ArgumentParser(description="ELM327 command-line TUI")
    parser.add_argument("--port", help="Serial port, example: COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=38400)
    parser.add_argument("--timeout", type=float, default=1.5)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--list-ports", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.list_ports:
        list_ports_available()
        return

    if not args.port:
        print("Error: --port is required")
        print()
        list_ports_available()
        return

    curses.wrapper(curses_main, args)


if __name__ == "__main__":
    main()