# Testing with PlatformIO

This guide explains how to write and run tests for the rover firmware.

---

## What Are PlatformIO Tests?

Tests let you upload **alternative code** to the Teensy to verify that specific parts of the system work correctly. Instead of running the full rover firmware, you run focused test code that exercises one subsystem at a time.

This is useful for:
- Verifying hardware works before full integration
- Debugging issues in isolation
- Testing new components as they're assembled

---

## How Testing Works

### Normal Build (`pio run`)
Compiles and uploads the main firmware from `src/main.cpp`.

### Test Build (`pio test`)
Compiles and uploads code from the `test/` folder instead. Your test file provides its own `setup()` and `loop()` functions.

The main `src/main.cpp` is **not included** during test builds—your test takes its place.

---

## Running Tests

### Run a Specific Test
```bash
pio test -f test_blink
```
This compiles and uploads just the `test_blink` test.

### Run All Tests
```bash
pio test
```
Runs each test in the `test/` folder sequentially.

### View Serial Output
Open the Serial Monitor to see test output:
```bash
pio device monitor
```

---

## Test Folder Structure

Each test lives in its own subfolder under `test/`:

```
test/
├── test_blink/
│   └── test_blink.cpp
├── test_can/
│   └── test_can.cpp
├── test_motor/
│   └── test_motor.cpp
└── test_sensors/
    └── test_sensors.cpp
```

The folder name determines the test name used with `pio test -f`.

---

## Writing a Basic Test

A test file is a complete Arduino program with `setup()` and `loop()`:

```cpp
// test/test_blink/test_blink.cpp

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.println("=== Blink Test ===");
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("LED ON");
    delay(500);
    
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("LED OFF");
    delay(500);
}
```

Run with:
```bash
pio test -f test_blink
```

---

## Testing a Subsystem

You can import modules from `src/` and `include/` to test them:

```cpp
// test/test_can/test_can.cpp

#include <Arduino.h>
#include "can_driver.h"

void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for Serial Monitor to connect
    
    Serial.println("=== CAN Driver Test ===");
    
    // Test initialization
    if (CAN_Init()) {
        Serial.println("CAN init: PASS");
    } else {
        Serial.println("CAN init: FAIL");
        return;
    }
    
    // Test sending a message
    Serial.println("Sending test message to motor 0x78...");
    CAN_SendMotorSpeed(0x78, 0.0);
    Serial.println("Message sent - verify motor controller received it");
}

void loop() {
    // Test complete - nothing to do
}
```

---

## Interactive Testing

For hardware testing, it's often useful to control the test from the Serial Monitor:

```cpp
// test/test_motor_interactive/test_motor_interactive.cpp

#include <Arduino.h>
#include "locomotion.h"
#include "can_driver.h"

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    CAN_Init();
    
    Serial.println("=== Motor Interactive Test ===");
    Serial.println("Commands:");
    Serial.println("  f = Forward");
    Serial.println("  b = Backward");
    Serial.println("  s = Stop");
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'f':
                Serial.println("Moving forward...");
                Locomotion_Forward(0.25);
                break;
            case 'b':
                Serial.println("Moving backward...");
                Locomotion_Backward(0.25);
                break;
            case 's':
                Serial.println("Stopping...");
                Locomotion_Stop();
                break;
        }
    }
}
```

Open the Serial Monitor (`pio device monitor`) and type commands to control the robot.

---

## Testing Hardware Before Integration

When bringing up new hardware, write a minimal test that exercises just that component:

```cpp
// test/test_single_motor/test_single_motor.cpp
// Test one motor before wiring all four

#include <Arduino.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

#define MOTOR_CAN_ID  0x78  // Change to the motor you're testing

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    can1.begin();
    can1.setBaudRate(500000);
    
    Serial.println("=== Single Motor Test ===");
    Serial.print("Testing motor ID: 0x");
    Serial.println(MOTOR_CAN_ID, HEX);
}

void loop() {
    // Send speed command
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id = MOTOR_CAN_ID;
    msg.len = 4;
    
    int32_t speed = 25000;  // 25% speed (0.25 * 100000)
    msg.buf[0] = (speed >> 24) & 0xFF;
    msg.buf[1] = (speed >> 16) & 0xFF;
    msg.buf[2] = (speed >> 8) & 0xFF;
    msg.buf[3] = speed & 0xFF;
    
    can1.write(msg);
    Serial.println("Sent speed command");
    
    delay(100);
}
```

---

## Using the Unity Test Framework

For automated pass/fail testing, PlatformIO includes the Unity framework:

```cpp
// test/test_example/test_example.cpp

#include <Arduino.h>
#include <unity.h>
#include "can_driver.h"

void test_can_initialization(void) {
    bool result = CAN_Init();
    TEST_ASSERT_TRUE(result);
}

void test_speed_calculation(void) {
    // Test that speed conversion works correctly
    int expected = 25000;
    int actual = (int)(0.25 * 100000);
    TEST_ASSERT_EQUAL(expected, actual);
}

void setup() {
    delay(2000);
    
    UNITY_BEGIN();
    
    RUN_TEST(test_can_initialization);
    RUN_TEST(test_speed_calculation);
    
    UNITY_END();
}

void loop() {
    // Tests run in setup(), nothing needed here
}
```

Output:
```
test/test_example.cpp:7:test_can_initialization    [PASSED]
test/test_example.cpp:12:test_speed_calculation    [PASSED]
-----------------------
2 Tests 0 Failures 0 Ignored
```

### Common Unity Assertions

| Assertion | Purpose |
|-----------|---------|
| `TEST_ASSERT_TRUE(condition)` | Verify condition is true |
| `TEST_ASSERT_FALSE(condition)` | Verify condition is false |
| `TEST_ASSERT_EQUAL(expected, actual)` | Verify integers are equal |
| `TEST_ASSERT_EQUAL_FLOAT(exp, act, delta)` | Verify floats are close |
| `TEST_ASSERT_NULL(pointer)` | Verify pointer is null |
| `TEST_ASSERT_NOT_NULL(pointer)` | Verify pointer is not null |

---

## Important Notes

### Restore Main Firmware After Testing
After running tests, the Teensy contains test code, not the main firmware. Upload the real firmware:
```bash
pio run -t upload
```

### Test One Thing at a Time
Tests are most useful when they focus on a single subsystem. If a test fails, you know exactly where the problem is.

### Keep Tests in Version Control
Test files serve as documentation for how to use each subsystem. They help new team members understand the hardware.

### Safety
When testing motors:
- Always have E-Stop accessible
- Have another person present for first-time motor tests
- Start with low speeds (25%)

---

## Quick Reference

| Command | Description |
|---------|-------------|
| `pio test` | Run all tests |
| `pio test -f test_name` | Run specific test |
| `pio device monitor` | Open Serial Monitor |
| `pio run -t upload` | Upload main firmware |
