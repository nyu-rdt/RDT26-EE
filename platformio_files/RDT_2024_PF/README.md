# RDT_2024_PF - Excavation Rover Control System

NYU Robotic Design Team | NASA Lunabotics
---

## Overview

This is the main firmware for the RDT26 excavation rover. It runs on a **Teensy 4.1** microcontroller and handles:
- 4-wheel drive locomotion (tank steering)
- Excavation arm positioning (up/down via servo)
- Conveyor belt control (material collection/ejection)
- Deposition bucket rotation
- Sensor monitoring (load cells, string potentiometer, rotary encoders)
- I2C communication with Raspberry Pi master
- Hardware E-Stop safety system

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI (Master)                        │
│                    High-level control / Vision                      │
└────────────────────────────┬────────────────────────────────────────┘
                             │ I2C (Address 0x24)
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      TEENSY 4.1 (This Code)                         │
│                                                                     │
│  ┌─────────┐ ┌───────────┐ ┌───────────┐ ┌───────────────────────┐ │
│  │ system  │ │   i2c_    │ │   can_    │ │       sensors         │ │
│  │         │ │ commands  │ │  driver   │ │                       │ │
│  │ E-Stop  │ │ Cmd Parse │ │ Motor Ctl │ │ HX711, String Pot     │ │
│  │ Safety  │ │ Data Xfer │ │ 500kbps   │ │ Rotary Encoders       │ │
│  └────┬────┘ └─────┬─────┘ └─────┬─────┘ └───────────┬───────────┘ │
│       │            │             │                   │             │
│  ┌────┴────────────┴─────────────┴───────────────────┴──────────┐  │
│  │                   locomotion / excavation                     │  │
│  │            4WD Drive | Arm Servo | Belt | Deposition          │  │
│  └───────────────────────────────────────────────────────────────┘  │
└───────────┬────────────────────────────┬────────────────────────────┘
            │ CAN Bus (500kbps)          │ PWM / GPIO
            ▼                            ▼
┌───────────────────────────┐    ┌──────────────────────────────────┐
│    CAN Motor Controllers  │    │       Physical Hardware          │
│  • 4× Drive Motors        │    │  • Servo (Excavation Arm)        │
│  • 1× Deposition Motor    │    │  • E-Stop Relay (Pin 2)          │
│  • 1× Excavation Belt     │    │  • 4× HX711 Load Cells           │
└───────────────────────────┘    │  • String Potentiometer          │
                                 │  • 4× Quadrature Encoders        │
                                 └──────────────────────────────────┘
```

---

## Code Structure

```
RDT_2024_PF/
├── include/                    # Header files (interfaces)
│   ├── config.h               # Pin definitions, CAN IDs, constants
│   ├── system.h               # E-stop and emergency stop
│   ├── can_driver.h           # CAN bus communication
│   ├── locomotion.h           # 4-wheel drive control
│   ├── excavation.h           # Arm, belt, deposition
│   ├── sensors.h              # Load cells, string pot, encoders
│   └── i2c_commands.h         # I2C slave interface
│
├── src/                        # Implementation files
│   ├── main.cpp               # Entry point, main loop
│   ├── system.cpp             # E-stop relay handling
│   ├── can_driver.cpp         # CAN message formatting/sending
│   ├── locomotion.cpp         # Motor direction mapping
│   ├── excavation.cpp         # Position control, belt/deposition
│   ├── sensors.cpp            # Sensor reading, encoder ISRs
│   └── i2c_commands.cpp       # Command processing
│
├── lib/HX711/                  # Load cell library
└── platformio.ini              # Build configuration
```

---

## Module Guide

### `main.cpp` - Entry Point
- Initializes all subsystems in order: System → CAN → Sensors → Excavation → I2C
- Main loop: Check E-Stop → Poll I2C → System Update
- Allows data requests even during E-Stop

### `system` - Safety & System Control
| Function | Purpose |
|----------|---------|
| `System_Init()` | Configure E-Stop relay pin |
| `System_CheckEStop()` | Check relay, trigger emergency stop if needed |
| `System_EmergencyStop()` | Stop ALL motors immediately |
| `System_Update()` | Periodic weight reading (500ms) |

### `can_driver` - CAN Bus Communication
- Uses FlexCAN_T4 library for Teensy 4.1
- 500kbps baud rate, extended frame format
- Speed values sent as fixed-point (×100,000)

### `locomotion` - 4-Wheel Drive
| Function | Motor Signs (FL, FR, RL, RR) |
|----------|------------------------------|
| `Forward()` | +, +, −, + |
| `Backward()` | −, −, +, − |
| `TurnLeft()` | +, −, −, − |
| `TurnRight()` | −, +, +, + |

Speed limited to 33% (`LOCOMOTION_DUTY_CYCLE`).

### `excavation` - Arm, Belt, Deposition
| Component | Control | Functions |
|-----------|---------|-----------|
| Arm | PWM Servo (pin 11) | `Up()`, `Down()`, `Stop()` |
| Belt | CAN (ID 0x68) | `BeltInward()`, `BeltOutward()`, `BeltStop()` |
| Deposition | CAN (ID 0x34) | `RotateCollection()`, `RotateDumping()`, `Stop()` |

Position functions (`MoveToLocomotionPosition`, `MoveToExcavationPosition`) use string potentiometer feedback.

### `sensors` - Input Devices
| Sensor | Purpose | Interface |
|--------|---------|-----------|
| HX711 ×4 | Weight measurement | GPIO (scales 3 & 4 active) |
| String Pot | Arm position (cm) | Analog A3 |
| Encoders ×4 | Wheel/joint angles | Interrupt-driven |

Encoders use quadrature decoding with 8192 counts/revolution.

### `i2c_commands` - Command Interface
- I2C slave at address 0x24
- Processes single-byte commands from Raspberry Pi
- Returns 3-byte data packet: [weight, length, encoder_angle]

---

## Hardware Pin Mapping

### Digital Pins
| Pin | Function |
|-----|----------|
| 2 | E-Stop Relay (INPUT_PULLUP, LOW = engaged) |
| 11 | Excavation Arm Servo PWM |
| 20, 21 | HX711 Scale 4 (DOUT, CLK) |
| 24, 25 | HX711 Scale 1 (DOUT, CLK) |
| 26, 27 | HX711 Scale 2 (DOUT, CLK) |
| 33, 34 | HX711 Scale 3 (CLK, DOUT) |
| 35, 36 | Encoder 1 (B, A) |
| 37, 38 | Encoder 2 (B, A) |
| 39, 40 | Encoder 3 (B, A) |
| 14, 15 | Encoder 4 (A, B) |

### Analog Pins
| Pin | Function |
|-----|----------|
| A3 | String Potentiometer |

### Communication
| Bus | Pins | Details |
|-----|------|---------|
| I2C | 18 (SDA), 19 (SCL) | Slave address 0x24 |
| CAN | 22 (TX), 23 (RX) | 500kbps, extended frames |
| USB Serial | USB | 115200 baud debug output |

---

## CAN Motor IDs

| Motor | CAN ID | Purpose |
|-------|--------|---------|
| Front Left | 0x78 | Drive |
| Front Right | 0x16 | Drive |
| Rear Left | 0x48 | Drive |
| Rear Right | 0x67 | Drive |
| Deposition | 0x34 | Bucket rotation |
| Excavation Belt | 0x68 | Conveyor belt |

---

## Command Reference

Commands are sent as single bytes over I2C from the Raspberry Pi.

### Movement Commands
| Command | Value | Description |
|---------|-------|-------------|
| `CMD_LOCOMOTION_STOP` | 16 | Stop all drive motors |
| `CMD_FORWARD_25/50/75/100` | 32-35 | Forward at 25/50/75/100% |
| `CMD_BACKWARD_25/50/75/100` | 48-51 | Backward at 25/50/75/100% |
| `CMD_LEFT_25/50/75/100` | 64-67 | Turn left at 25/50/75/100% |
| `CMD_RIGHT_25/50/75/100` | 80-83 | Turn right at 25/50/75/100% |

### Excavation Commands
| Command | Value | Description |
|---------|-------|-------------|
| `CMD_EXCAVATION_ZERO` | 96 | Stop arm movement |
| `CMD_EXCAVATION_LOCOMOTION_POS` | 97 | Move arm up for driving |
| `CMD_EXCAVATION_POSITION` | 98 | Move arm down for digging |
| `CMD_BELT_STOP` | 99 | Stop conveyor belt |
| `CMD_BELT_OUTWARD` | 100 | Eject material |
| `CMD_BELT_INWARD` | 101 | Collect material |
| `CMD_ACME_UP` | 102 | Manual arm up |
| `CMD_ACME_DOWN` | 103 | Manual arm down |

### Deposition Commands
| Command | Value | Description |
|---------|-------|-------------|
| `CMD_DEPOSITION_ROTATE_COLLECTION` | 112 | Rotate to collect position |
| `CMD_DEPOSITION_ROTATE_DUMPING` | 113 | Rotate to dump position |
| `CMD_DEPOSITION_ROTATE_STOP` | 114 | Stop rotation |

### System Commands
| Command | Value | Description |
|---------|-------|-------------|
| `CMD_EMERGENCY_STOP` | 1 | Stop ALL motors immediately |
| `CMD_REQUEST_DATA` | 128 | Request sensor data |
| `CMD_SWITCH_AUTONOMOUS` | 129 | Switch to autonomous mode |

---

## Data Packet Format

When the Pi requests data, Teensy responds with 3 bytes:

| Byte | Content | Range |
|------|---------|-------|
| 0 | Weight (÷20) | 0-255 |
| 1 | String length (inches) | 0-255 |
| 2 | Active encoder angle | 0-255 (mapped from 0-360°) |

---

## E-Stop Behavior

The E-Stop is a **hardware safety feature** using a relay connected to pin 2.

| Relay State | Pin Reading | System State |
|-------------|-------------|--------------|
| Energized (ON) | HIGH | Normal operation |
| De-energized (OFF) | LOW | **E-Stop engaged** |

When E-Stop is engaged:
1. All motors are immediately stopped
2. No new motor commands are processed
3. Data requests (CMD_REQUEST_DATA) still work
4. System remains in E-Stop until relay is re-energized

This is **fail-safe**: if power is lost to the relay, the rover stops.

---

## Building & Uploading

```bash
# Build only
pio run

# Build and upload
pio run -t upload

# Monitor serial output
pio device monitor -b 115200

# Clean build
pio run -t clean
```

---

## Debugging Tips

### Serial Debug Output
The code prints status messages to USB Serial at 115200 baud:
- "=== Rover Control System ===" on startup
- Motor commands when executed
- E-Stop state changes
- Position movement progress

### Common Issues

**Motors not responding:**
- Check CAN bus wiring and termination
- Verify motor controller CAN IDs match config.h
- Check E-Stop relay is energized (pin 2 = HIGH)

**Position movement times out:**
- Check string potentiometer wiring (A3)
- Verify threshold values in config.h

**I2C communication fails:**
- Verify Teensy I2C address (0x24)
- Check pull-up resistors on SDA/SCL
- Ensure Pi and Teensy share common ground

**Encoder readings wrong:**
- Check encoder direction (some are inverted in sensors.cpp)
- Verify interrupt pins are connected correctly

---

## Adding New Features

1. **Add constants** to `include/config.h`
2. **Create module** with `.h` in `include/` and `.cpp` in `src/`
3. **Add command** to `i2c_commands.cpp` switch statement
4. **Initialize** in `main.cpp` setup() function
5. **Document** changes in this README

---

## Team Contact

NYU Robotic Design Team  
[https://github.com/nyu-rdt](https://github.com/nyu-rdt)
