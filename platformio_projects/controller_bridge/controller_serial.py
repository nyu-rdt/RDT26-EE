#!/usr/bin/env python3
"""
controller_serial.py  —  Xbox controller → i2c_parent Teensy serial bridge

Mirrors the GCS manual control button layout but sends over USB serial
instead of I2C. Requires the escape-byte patch in keyboard.cpp (0xFE prefix).

Install deps:
    pip install pygame pyserial

Usage:
    python controller_serial.py COM3          # Windows
    python controller_serial.py /dev/ttyACM0  # Linux

Run with --debug to print raw axis/button indices (useful for remapping).
"""

import sys
import time
import threading
import pygame
import serial

# ---------------------------------------------------------------------------
# Command bytes — matches GCS hex_map.py / i2c_parent config.h exactly
# ---------------------------------------------------------------------------
EMERGENCY_STOP    = 0x01

LOCO_STOP         = 0x10
LOCO_FORWARD_25   = 0x20
LOCO_FORWARD_50   = 0x21
LOCO_FORWARD_75   = 0x22
LOCO_FORWARD_100  = 0x23
LOCO_BACKWARD_25  = 0x30
LOCO_BACKWARD_50  = 0x31
LOCO_BACKWARD_75  = 0x32
LOCO_BACKWARD_100 = 0x33
LOCO_LEFT_25      = 0x40
LOCO_LEFT_50      = 0x41
LOCO_LEFT_75      = 0x42
LOCO_LEFT_100     = 0x43
LOCO_RIGHT_25     = 0x50
LOCO_RIGHT_50     = 0x51
LOCO_RIGHT_75     = 0x52
LOCO_RIGHT_100    = 0x53

EXCAV_ZERO        = 0x60  # A button
EXCAV_LOCO_POS    = 0x61  # B button
EXCAV_EXCAV_POS   = 0x62  # Y button
BELT_STOP         = 0x63  # Menu button
BELT_OUTWARD      = 0x64  # LB
BELT_INWARD       = 0x65  # RB

BIN_OPEN          = 0x70  # D-pad left
BIN_CLOSE         = 0x71  # D-pad right
VIBRATE_ON        = 0x72  # D-pad down
VIBRATE_STOP      = 0x73  # D-pad up

CMD_NAMES = {v: k for k, v in globals().items() if isinstance(v, int) and k[0].isupper()}

# ---------------------------------------------------------------------------
# Button / axis indices for Xbox One controller via pygame on Windows (XInput)
# If yours differ, run with --debug and adjust these.
# ---------------------------------------------------------------------------
BTN_A     = 0
BTN_B     = 1
BTN_X     = 2
BTN_Y     = 3
BTN_LB    = 4
BTN_RB    = 5
BTN_VIEW  = 6   # Back / View — request sensor data
BTN_MENU  = 7   # Start / Menu — belt stop

AXIS_LEFT_X  = 0   # left stick horizontal
AXIS_LEFT_Y  = 1   # left stick vertical  (pygame: up = -1)
AXIS_RIGHT_X = 2   # right stick horizontal
AXIS_LT      = 4   # left trigger  (-1 unpressed → +1 full)
AXIS_RT      = 5   # right trigger (-1 unpressed → +1 full)

TRIGGER_ESTOP_THRESHOLD = 0.5

DEADZONE     = 0.15
AXIS_POLL_HZ = 3        # matches GCS RATE_LIMIT = 1/3

LOCO_RANGES = [
    (0.75, 1.01, 3),
    (0.50, 0.75, 2),
    (0.25, 0.50, 1),
    (0.05, 0.25, 0),
]

BUTTON_MAP = {
    BTN_A:    EXCAV_ZERO,
    BTN_B:    EXCAV_LOCO_POS,
    BTN_Y:    EXCAV_EXCAV_POS,
    BTN_LB:   BELT_OUTWARD,
    BTN_RB:   BELT_INWARD,
    BTN_MENU: BELT_STOP,
}

ESCAPE = 0xFE   # must match keyboard.cpp patch


def send(ser, cmd, verbose=True):
    ser.write(bytes([ESCAPE, cmd]))
    if verbose:
        name = CMD_NAMES.get(cmd, f"0x{cmd:02X}")
        print(f"  → {name} (0x{cmd:02X})")


def request_data(ser):
    # Send plain 'I' so keyboard.cpp does the full I2C read + serial print.
    # Escape-byte path only sends the I2C byte, not the requestFrom/print.
    ser.write(b'I')
    print("  → REQUEST_DATA (waiting for Teensy response...)")


def stick_to_loco(ly, rx):
    y = -ly   # pygame Y is inverted: up = -1
    x = rx
    ay, ax = abs(y), abs(x)

    if ay > ax and ay > DEADZONE:
        base = LOCO_FORWARD_25 if y > 0 else LOCO_BACKWARD_25
        for lo, hi, spd in LOCO_RANGES:
            if lo <= ay < hi:
                return base + spd

    elif ax >= ay and ax > DEADZONE:
        base = LOCO_RIGHT_25 if x > 0 else LOCO_LEFT_25
        for lo, hi, spd in LOCO_RANGES:
            if lo <= ax < hi:
                return base + spd

    return LOCO_STOP


def serial_reader(ser, stop_event):
    """Background thread — prints anything the Teensy sends back."""
    while not stop_event.is_set():
        try:
            line = ser.readline()
            if line:
                print(f"[TEENSY] {line.decode('utf-8', errors='replace').strip()}")
        except Exception:
            break


def debug_loop(joy):
    print("DEBUG mode — press Ctrl+C to exit")
    print(f"Controller: {joy.get_name()}  "
          f"({joy.get_numaxes()} axes, {joy.get_numbuttons()} buttons, {joy.get_numhats()} hats)")
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                print(f"  BUTTON DOWN  index={event.button}")
            elif event.type == pygame.JOYBUTTONUP:
                print(f"  BUTTON UP    index={event.button}")
            elif event.type == pygame.JOYAXISMOTION and abs(event.value) > 0.1:
                print(f"  AXIS         index={event.axis}  value={event.value:+.3f}")
            elif event.type == pygame.JOYHATMOTION:
                print(f"  HAT          index={event.hat}  value={event.value}")


def main():
    debug = "--debug" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller detected. Plug in an Xbox controller and retry.")
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()

    if debug:
        debug_loop(joy)
        return

    if not args:
        print(__doc__)
        sys.exit(1)

    port = args[0]
    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(2)   # allow Teensy to finish USB-serial reset

    stop_event = threading.Event()
    reader = threading.Thread(target=serial_reader, args=(ser, stop_event), daemon=True)
    reader.start()

    print(f"Connected: {port}")
    print(f"Controller: {joy.get_name()}")
    print()
    print("  Left stick         → Drive (forward / backward)")
    print("  Right stick X      → Turn (left / right)")
    print("  A / B / Y          → Excavation positions")
    print("  LB / RB / Menu     → Belt outward / inward / stop")
    print("  D-pad ←/→          → Bin open / close")
    print("  D-pad ↓/↑          → Vibrate on / off")
    print("  Either trigger     → E-STOP")
    print("  View button        → Request sensor data")
    print()
    print("Ctrl+C to quit.")
    print()

    last_axis_poll = 0.0

    try:
        while True:
            now = time.time()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_VIEW:
                        request_data(ser)
                        continue
                    cmd = BUTTON_MAP.get(event.button)
                    if cmd is not None:
                        send(ser, cmd)

                if event.type == pygame.JOYAXISMOTION:
                    if event.axis in (AXIS_LT, AXIS_RT) and event.value >= TRIGGER_ESTOP_THRESHOLD:
                        print("  *** E-STOP TRIGGERED ***")
                        send(ser, EMERGENCY_STOP)

                if event.type == pygame.JOYHATMOTION:
                    hx, hy = event.value
                    if   hx == -1: send(ser, BIN_OPEN)
                    elif hx ==  1: send(ser, BIN_CLOSE)
                    if   hy == -1: send(ser, VIBRATE_ON)
                    elif hy ==  1: send(ser, VIBRATE_STOP)

            # Axis polling at 3 Hz — always send including LOCO_STOP when neutral.
            # Serves as keepalive; matches GCS rate limit.
            if now - last_axis_poll >= 1.0 / AXIS_POLL_HZ:
                last_axis_poll = now
                cmd = stick_to_loco(joy.get_axis(AXIS_LEFT_Y), joy.get_axis(AXIS_RIGHT_X))
                send(ser, cmd)

    except KeyboardInterrupt:
        print("\nExiting — sending stop.")
    finally:
        stop_event.set()
        send(ser, LOCO_STOP, verbose=False)
        ser.close()
        pygame.quit()


if __name__ == "__main__":
    main()
