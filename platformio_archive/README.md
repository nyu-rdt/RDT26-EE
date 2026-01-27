# PlatformIO Projects

This folder contains all PlatformIO-based firmware projects for the RDT26-EE excavation rover.

---

## 🎯 Which Project Should I Use?

| Project | Status | Use Case |
|---------|--------|----------|
| **[RDT_2024_PF](./RDT_2024_PF/)** | ✅ **Active** | Modular architecture for Teensy 4.1 - **Start here!** |
| [2024_teensy_original](./2024_teensy_original/) | ✅ Reference | Single-file version for Teensy 4.1 (legacy reference) |
| [2024_STM32_migr](./2024_STM32_migr/) | ⚠️ Outdated | STM32 port - based on old code, not currently in use |
| [stm32_rtos_2024](./stm32_rtos_2024/) | ⚠️ Outdated | STM32 + FreeRTOS - based on old code, not currently in use |

**New team members should start with `RDT_2024_PF`** - it has clean modular code with proper documentation.

---

## Project Descriptions

### RDT_2024_PF (Recommended)
**Platform:** Teensy 4.1 | **Framework:** Arduino

The primary codebase for the excavation rover. Features a clean modular architecture:
- Separate modules for locomotion, excavation, sensors, CAN, I2C, and system control
- Full support for rotary encoders, HX711 load cells, E-stop relay
- CAN bus motor control at 500kbps
- I2C slave interface for Raspberry Pi communication

```
RDT_2024_PF/
├── include/          # Header files for each module
├── src/              # Implementation files
├── lib/HX711/        # Load cell library
└── platformio.ini    # Build configuration
```

**Build:** `cd RDT_2024_PF && pio run`

See [RDT_2024_PF/README.md](./RDT_2024_PF/README.md) for detailed system documentation.

---

### 2024_teensy_original
**Platform:** Teensy 4.1 | **Framework:** Arduino

The original single-file implementation (~1000 lines in `main.cpp`). This is the reference implementation that `RDT_2024_PF` was modularized from. Contains all features but is harder to navigate and maintain.

Use this if you need to:
- Compare behavior with the modular version
- Understand the original implementation intent
- Debug issues by testing against a known-working version

**Build:** `cd 2024_teensy_original && pio run`

---

### 2024_STM32_migr
**Platform:** STM32F446RE | **Framework:** STM32Cube HAL

⚠️ **Not currently in use** - This was a port to STM32, but it's based on an older version of the code that's missing:
- Rotary encoder support
- E-stop relay handling
- CAN-based belt motor control
- Updated pin mappings

If we decide to migrate to STM32 in the future, this project will need to be updated to match `RDT_2024_PF`.

---

### stm32_rtos_2024
**Platform:** STM32F446RE | **Framework:** STM32Cube HAL + FreeRTOS

⚠️ **Not currently in use** - Same as `2024_STM32_migr` but with FreeRTOS for real-time task scheduling. Also based on outdated code.

Contains a custom FreeRTOS configuration but would need significant updates before use.

---

## Getting Started

### Prerequisites
1. Install [PlatformIO](https://platformio.org/install) (VS Code extension recommended)
2. Install Teensy platform: `pio pkg install -g -p teensy`

### Building
```bash
# Navigate to desired project
cd RDT_2024_PF

# Build
pio run

# Build and upload to connected Teensy
pio run -t upload

# Monitor serial output
pio device monitor -b 115200
```