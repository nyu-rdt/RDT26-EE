#!/usr/bin/env python3
"""
Jetson Nano I2C Controller for Teensy 4.1
Sends I2C commands matching the platformio RDT_2025_26_TEENSY4_1 protocol.

Command Structure (1 byte):
  - Upper 4 bits: Group (command type)
  - Lower 4 bits: Param (speed index 0-3)

Groups:
  0x0: Control (param 0x01 = E-stop)
  0x1: Locomotion Stop
  0x2: Forward
  0x3: Backward
  0x4: Turn Left
  0x5: Turn Right
  0x6: Excavation (not implemented)
  0x7: Deposition (not implemented)
  0x8: Data (not implemented)

Speed Levels (param 0-3):
  0: 25% duty
  1: 50% duty
  2: 75% duty
  3: 100% duty

Usage:
  python3 jetson_i2c_controller.py
  Then use WASD to control, Q/E to change speed, X to stop

Hardware Setup:
  - Jetson Nano I2C bus 1 (pins 3=SDA, 5=SCL)
  - Teensy 4.1 I2C (pins 18=SDA, 19=SCL)
  - Connect SDA-SDA, SCL-SCL, GND-GND
  - 4.7k pull-up resistors on SDA and SCL to 3.3V
"""

import smbus2
import sys
import time
import select
import termios
import tty

# I2C Configuration
I2C_BUS = 1  # Jetson Nano uses bus 1 for GPIO header I2C
I2C_CHILD_ADDRESS = 0x08

# Command Groups
GRP_CONTROL = 0x0
GRP_LOCO_STOP = 0x1
GRP_FORWARD = 0x2
GRP_BACKWARD = 0x3
GRP_LEFT = 0x4
GRP_RIGHT = 0x5
GRP_EXCAVATION = 0x6
GRP_DEPOSITION = 0x7
GRP_DATA = 0x8

# Speed levels (0-3 map to 25%, 50%, 75%, 100%)
SPEED_PERCENTAGES = [25, 50, 75, 100]


def build_command(group: int, param: int) -> int:
    """Build command byte from group and param."""
    return ((group & 0x0F) << 4) | (param & 0x0F)


class TeensyI2CController:
    def __init__(self, bus_num: int = I2C_BUS, address: int = I2C_CHILD_ADDRESS):
        self.bus_num = bus_num
        self.address = address
        self.bus = None
        self.speed_level = 1  # Default to 50%
        self.current_mode = "STOP"

    def connect(self) -> bool:
        """Initialize I2C bus connection."""
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            print(f"Connected to I2C bus {self.bus_num}")
            return True
        except Exception as e:
            print(f"Error connecting to I2C bus: {e}")
            return False

    def disconnect(self):
        """Close I2C bus connection."""
        if self.bus:
            self.bus.close()
            self.bus = None

    def send_command(self, group: int, param: int = 0) -> bool:
        """Send a command byte to the Teensy."""
        if not self.bus:
            print("Not connected to I2C bus")
            return False

        cmd = build_command(group, param)
        try:
            self.bus.write_byte(self.address, cmd)
            return True
        except Exception as e:
            print(f"I2C write error: {e}")
            return False

    def stop(self):
        """Send locomotion stop command."""
        self.current_mode = "STOP"
        self.send_command(GRP_LOCO_STOP, 0)
        self._print_status()

    def forward(self):
        """Move forward at current speed."""
        self.current_mode = "FORWARD"
        self.send_command(GRP_FORWARD, self.speed_level)
        self._print_status()

    def backward(self):
        """Move backward at current speed."""
        self.current_mode = "BACKWARD"
        self.send_command(GRP_BACKWARD, self.speed_level)
        self._print_status()

    def turn_left(self):
        """Turn left at current speed."""
        self.current_mode = "LEFT"
        self.send_command(GRP_LEFT, self.speed_level)
        self._print_status()

    def turn_right(self):
        """Turn right at current speed."""
        self.current_mode = "RIGHT"
        self.send_command(GRP_RIGHT, self.speed_level)
        self._print_status()

    def emergency_stop(self):
        """Send emergency stop (control group, param 1)."""
        self.current_mode = "E-STOP"
        self.send_command(GRP_CONTROL, 0x01)
        self._print_status()

    def speed_up(self):
        """Increase speed level."""
        if self.speed_level < 3:
            self.speed_level += 1
            self._resend_current_mode()
            self._print_status()

    def speed_down(self):
        """Decrease speed level."""
        if self.speed_level > 0:
            self.speed_level -= 1
            self._resend_current_mode()
            self._print_status()

    def _resend_current_mode(self):
        """Resend the current movement command with updated speed."""
        mode_map = {
            "FORWARD": GRP_FORWARD,
            "BACKWARD": GRP_BACKWARD,
            "LEFT": GRP_LEFT,
            "RIGHT": GRP_RIGHT,
        }
        if self.current_mode in mode_map:
            self.send_command(mode_map[self.current_mode], self.speed_level)

    def _print_status(self):
        """Print current status."""
        speed_pct = SPEED_PERCENTAGES[self.speed_level]
        print(f"Mode: {self.current_mode:8} | Speed: {self.speed_level}/3 ({speed_pct}%)")


def get_key_nonblocking():
    """Get a single keypress without blocking (Unix/Linux only)."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    print("=" * 50)
    print("Jetson Nano I2C Controller for Teensy 4.1")
    print("=" * 50)
    print()
    print("Controls:")
    print("  W/A/S/D - Forward/Left/Backward/Right")
    print("  E       - Speed Up")
    print("  Q       - Speed Down")
    print("  X/Space - Stop")
    print("  Ctrl+C  - Exit")
    print()

    controller = TeensyI2CController()

    if not controller.connect():
        print("Failed to connect. Check I2C wiring and permissions.")
        print("You may need to run: sudo chmod 666 /dev/i2c-1")
        sys.exit(1)

    # Send initial stop command
    controller.stop()

    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)

    try:
        # Set terminal to raw mode for single keypress reading
        tty.setcbreak(sys.stdin.fileno())

        print("\nReady. Waiting for input...")

        while True:
            key = get_key_nonblocking()

            if key:
                key = key.lower()

                if key == 'w':
                    controller.forward()
                elif key == 'a':
                    controller.turn_left()
                elif key == 's':
                    controller.backward()
                elif key == 'd':
                    controller.turn_right()
                elif key == 'e':
                    controller.speed_up()
                elif key == 'q':
                    controller.speed_down()
                elif key in ('x', ' '):
                    controller.stop()
                elif key == '\x03':  # Ctrl+C
                    break

            time.sleep(0.01)  # Small delay to prevent CPU spinning

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # Send stop before disconnecting
        controller.stop()
        controller.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
