# Coding Style Guide

Guidelines for keeping the codebase readable. Not strict rules - use your judgment.

This doc is also useful context for AI assistants (Copilot, Claude, etc.) when working on the codebase.

---

## Naming

| Thing | Style | Example |
|-------|-------|---------|
| Files | `snake_case` | `can_driver.cpp` |
| Functions | Module prefix + descriptive | `CAN_SendSpeed()`, `Sensors_Init()` |
| Variables | `camelCase` | `motorSpeed`, `encoderCount` |
| Constants | `ALL_CAPS` | `CAN_BAUD_RATE`, `PIN_MOTOR_PWM` |

The module prefix (like `CAN_`, `Sensors_`) helps identify where functions come from since C++ doesn't have namespaces in the Arduino world.

---

## File Structure

**Header files (.h):**
```cpp
#ifndef MODULE_NAME_H
#define MODULE_NAME_H

#include <Arduino.h>

void Module_Init(void);
void Module_DoThing(int param);

#endif
```

**Source files (.cpp):**
```cpp
#include "module_name.h"
#include "config.h"

static int s_internalVariable = 0;  // static = file-private

void Module_Init(void) {
    // setup code
    Serial.println("Module ready");
}

void Module_DoThing(int param) {
    // implementation
}
```

---

## Comments

```cpp
// Use comments to explain WHY, not WHAT
// Bad: increment counter
// Good: increment to track missed CAN messages for timeout detection

// Mark things that need attention
// TODO: add timeout handling
// FIXME: this breaks if speed > 1.0
// VERIFIED: tested on hardware 2026-01-26

// For functions that others will call, brief description is helpful
/**
 * Send speed command to motor over CAN
 * @param canId Motor's CAN address
 * @param speed -1.0 to 1.0
 */
void CAN_SendSpeed(uint32_t canId, float speed);
```

---

## Hardware-Specific Stuff

**Put all pins in config.h:**
```cpp
// config.h
#define PIN_MOTOR_PWM       11
#define PIN_ESTOP_RELAY     2
#define PIN_STRING_POT      A3
```

**Volatile for interrupt variables:**
```cpp
volatile long encoderCount = 0;  // modified in ISR

void encoderISR(void) {
    encoderCount++;
}
```

**Protect shared data:**
```cpp
long getEncoderCount(void) {
    noInterrupts();
    long count = encoderCount;
    interrupts();
    return count;
}
```

---

## Avoid Magic Numbers

```cpp
// Bad
if (reading > 512) { ... }

// Good
#define SENSOR_THRESHOLD 512
if (reading > SENSOR_THRESHOLD) { ... }
```

---

## Error Handling

Check if things worked, especially hardware:
```cpp
if (!CAN_Init()) {
    Serial.println("ERROR: CAN failed to init");
    // handle it somehow
}
```

---

