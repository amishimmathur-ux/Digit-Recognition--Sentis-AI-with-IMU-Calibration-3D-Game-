"""
gesture_presenter.py
====================
Host app for the STM32 B-U585I-IOT02A Game Controller.
Reads UART over USB Serial (115200 baud) and drives keyboard/mouse.

Usage:
    python gesture_presenter.py
    python gesture_presenter.py --port COM3

Install deps first:
    pip install pyserial pynput
"""

import argparse
import time
import serial
import serial.tools.list_ports
from pynput.mouse import Button, Controller as Mouse
from pynput.keyboard import Key, KeyCode, Controller as Keyboard
import ctypes

# ── Raw SendInput-based relative mouse movement (Windows only) ──────────────
# pynput's mouse.move() uses SetCursorPos (absolute jump), which most game
# engines ignore for camera look (they read raw mouse deltas). SendInput with
# MOUSEEVENTF_MOVE sends relative deltas through the same input pipeline as a
# real mouse, which games pick up correctly.

PUL = ctypes.POINTER(ctypes.c_ulong)

class MouseInput(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_long),
                ("dy", ctypes.c_long),
                ("mouseData", ctypes.c_ulong),
                ("dwFlags", ctypes.c_ulong),
                ("time", ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class Input_I(ctypes.Union):
    _fields_ = [("mi", MouseInput)]

class Input(ctypes.Structure):
    _fields_ = [("type", ctypes.c_ulong), ("ii", Input_I)]

MOUSEEVENTF_MOVE = 0x0001

def send_relative_move(dx, dy):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.mi = MouseInput(dx, dy, 0, MOUSEEVENTF_MOVE, 0, ctypes.pointer(extra))
    cmd = Input(ctypes.c_ulong(0), ii_)
    ctypes.windll.user32.SendInput(1, ctypes.pointer(cmd), ctypes.sizeof(cmd))

# ── Controllers ──────────────────────────────────────────────────────────────
mouse    = Mouse()
keyboard = Keyboard()
W_KEY    = KeyCode.from_char('w')

# Track held state so we never double-press/release or leave something stuck
_w_held      = False
_lclick_held = False

def _release_all():
    """Safety net — called on disconnect/exit so nothing stays stuck down."""
    global _w_held, _lclick_held
    if _w_held:
        keyboard.release(W_KEY)
        _w_held = False
    if _lclick_held:
        mouse.release(Button.left)
        _lclick_held = False

def handle(line):
    """Map one UART line to a Windows action."""
    global _w_held, _lclick_held

    if line == "W_DOWN":
        if not _w_held:
            keyboard.press(W_KEY)
            _w_held = True
            print(f"[ACTION] W_DOWN → W held")

    elif line == "W_UP":
        if _w_held:
            keyboard.release(W_KEY)
            _w_held = False
            print(f"[ACTION] W_UP → W released")

    elif line == "LCLICK_DOWN":
        if not _lclick_held:
            mouse.press(Button.left)
            _lclick_held = True
            print(f"[ACTION] LCLICK_DOWN → left mouse held")

    elif line == "LCLICK_UP":
        if _lclick_held:
            mouse.release(Button.left)
            _lclick_held = False
            print(f"[ACTION] LCLICK_UP → left mouse released")

    elif line == "SPACE_TAP":
        keyboard.press(Key.space)
        keyboard.release(Key.space)
        print(f"[ACTION] SPACE_TAP → space bar tapped")

    elif line.startswith("MOUSE_MOVE:"):
        try:
            dx, dy = map(int, line[len("MOUSE_MOVE:"):].split(","))
            send_relative_move(dx, dy)
            if abs(dx) > 3 or abs(dy) > 3:
                print(f"[MOVE]   dx={dx:+d} dy={dy:+d}")
        except ValueError:
            print(f"[WARN]   Bad MOUSE_MOVE payload: {line}")

    # Boot/diagnostic messages — log and ignore
    elif any(line.startswith(p) for p in
             ("WHO_AM_I:", "BSP_I2C2_INIT:", "IMU_FAIL:", "FOUND:",
              "BOOT", "STARTING_IMU", "IMU_CHECKING", "IMU_OK",
              "IMU_BUS_FAIL", "IMU_INIT_FAIL", "READY")):
        print(f"[DIAG]   {line}")

    else:
        print(f"[SKIP]   Unknown: {line!r}")


# ── Serial reader with auto-reconnect ────────────────────────────────────────
def serial_loop(port):
    while True:
        try:
            print(f"\nOpening {port} at 115200 baud…")
            with serial.Serial(port, 115200, timeout=1) as ser:
                print(f"Connected. Waiting for STM32…\n")
                while True:
                    raw = ser.readline()
                    if raw:
                        line = raw.decode("ascii", errors="replace").strip()
                        if line:
                            handle(line)
        except serial.SerialException as e:
            _release_all()
            print(f"[WARN]   Disconnected: {e} — retrying in 2s…")
            time.sleep(2)


# ── Entry point ──────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="STM32 Game Controller host")
    parser.add_argument("--port", "-p", default=None, help="COM port, e.g. COM3")
    args = parser.parse_args()

    port = args.port
    if not port:
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("No serial ports found. Plug in the board and retry.")
            return
        print("Available ports:")
        for i, p in enumerate(ports, 1):
            print(f"  [{i}] {p.device}  {p.description}")
        choice = input("Select number: ").strip()
        port = ports[int(choice) - 1].device

    try:
        serial_loop(port)
    except KeyboardInterrupt:
        _release_all()
        print("\nExiting.")

if __name__ == "__main__":
    main()