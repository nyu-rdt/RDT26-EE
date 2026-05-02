#include <Arduino.h>
#include "keyboard.h"
#include "config.h"
#include "parent.h"

#if USE_WASD
static uint8_t speedLevel = 1;  // 0-3
static uint8_t currentMode = GRP_LOCO_STOP;
static unsigned long lastSendTime = 0;
static const unsigned long KEEPALIVE_INTERVAL_MS = 300;

// Escape prefix used by controller_serial.py so raw hex commands
// can coexist with WASD ASCII keys without collision.
static const uint8_t SERIAL_ESCAPE = 0xFE;
static bool escapeArmed = false;

const char* message = "W/A/S/D=Move | E/Q=Speed | X/Space=Stop | U/J/H=Belt Fwd/Rev/Stop | O/L/K=Vert Fwd/Rev/Stop | R/F=Door Open/Close | T/G=Vib On/Off | I=Request Data";

static void printStatus(const char* mode) {
    Serial.print("Mode: ");
    Serial.print(mode);
    Serial.print(" | Speed: ");
    Serial.print(speedLevel);
    Serial.println("/3");
}

static void processKey(char key) {
    if (key >= 'a' && key <= 'z') key -= 32;  // Uppercase

    switch (key) {
        case 'W':
            currentMode = GRP_FORWARD;
            i2c_parent_sendCommand(currentMode, speedLevel);
            printStatus("FORWARD");
            break;
        case 'A':
            currentMode = GRP_LEFT;
            i2c_parent_sendCommand(currentMode, speedLevel);
            printStatus("LEFT");
            break;
        case 'S':
            currentMode = GRP_BACKWARD;
            i2c_parent_sendCommand(currentMode, speedLevel);
            printStatus("BACKWARD");
            break;
        case 'D':
            currentMode = GRP_RIGHT;
            i2c_parent_sendCommand(currentMode, speedLevel);
            printStatus("RIGHT");
            break;
        case 'E':
            if (speedLevel < 3) {
                speedLevel++;
                if (currentMode != GRP_LOCO_STOP) {
                    i2c_parent_sendCommand(currentMode, speedLevel);
                }
                printStatus("SPEED UP");
            }
            break;
        case 'Q':
            if (speedLevel > 0) {
                speedLevel--;
                if (currentMode != GRP_LOCO_STOP) {
                    i2c_parent_sendCommand(currentMode, speedLevel);
                }
                printStatus("SPEED DOWN");
            }
            break;
        case 'X':
            currentMode = GRP_CONTROL;
            i2c_parent_sendCommand(GRP_CONTROL, 1);
            printStatus("ESTOP");
            break;
        case ' ':
            currentMode = GRP_LOCO_STOP;
            i2c_parent_sendCommand(GRP_LOCO_STOP, STOP);
            printStatus("STOP");
            break;
        // belt excav control (old mapping: GRP_EXCAVATION params 3-5 = belt stop/fwd/rev)
        case 'U':
                currentMode = GRP_EXCAVATION;
                i2c_parent_sendCommand(currentMode, FORWARD + 3); // belt forward
                printStatus("FORWARD excav BELT");
            break;
        case 'J':
                currentMode = GRP_EXCAVATION;
                i2c_parent_sendCommand(currentMode, REVERSE + 3); // belt reverse
                printStatus("BACKWARD excav BELT");
            break;
        case 'H':
                currentMode = GRP_EXCAVATION;
                i2c_parent_sendCommand(currentMode, STOP + 3); // belt stop
                printStatus("STOP excav BELT");
            break;
        // vertical excav control (old mapping: GRP_EXCAVATION params 0-2 = vert stop/fwd/rev)
        case 'O':
                currentMode = GRP_EXCAVATION;
                i2c_parent_sendCommand(currentMode, FORWARD); // vert up
                printStatus("UP excav");
            break;
        case 'L':
                currentMode = GRP_EXCAVATION;
                i2c_parent_sendCommand(currentMode, REVERSE); // vert down
                printStatus("DOWN excav");
            break;
        case 'K':
                currentMode = GRP_EXCAVATION;
                i2c_parent_sendCommand(currentMode, STOP); // vert stop
                printStatus("STOP excav");
            break;
        // deposition door control: 0=open, 1=close (door self-stops, no stop command)
        case 'R':
                currentMode = GRP_DEPOSITION;
                i2c_parent_sendCommand(currentMode, DEPO_DOOR_OPEN);
                printStatus("OPEN depo DOOR");
            break;
        case 'F':
                currentMode = GRP_DEPOSITION;
                i2c_parent_sendCommand(currentMode, DEPO_DOOR_CLOSE);
                printStatus("CLOSE depo DOOR");
            break;
        // deposition vibration control: 2=vib off, 3=vib on
        case 'T':
                currentMode = GRP_DEPOSITION;
                i2c_parent_sendCommand(currentMode, DEPO_VIB_ON);
                printStatus("ON depo VIB");
            break;
        case 'G':
                currentMode = GRP_DEPOSITION;
                i2c_parent_sendCommand(currentMode, DEPO_VIB_OFF);
                printStatus("OFF depo VIB");
            break;
        case 'I': {
            uint8_t buf[RESPONSE_BYTES] = {};
            uint8_t count = i2c_parent_requestData(buf, RESPONSE_BYTES);
            if (count >= RESPONSE_BYTES) {
                // Packet layout (18 bytes) — matches task_telemetry.cpp:
                //   [0-7]   motor currents: byte * (20.0/255.0) = amps
                //   [8-9]   encoders: byte = degrees (clamped to 255)
                //   [10-13] load cells: byte * 0.1 = kg
                //   [14]    string pot: byte = cm
                //   [15]    depo door state enum
                //   [16]    flags: bit0=estop(relay off), bit1=overcurrent
                //   [17]    sentinel 0xFF
                Serial.print("[DATA]");
                Serial.print(" estop=");   Serial.print(buf[16] & 0x01 ? "YES" : "no");
                Serial.print(" overcurrent="); Serial.print(buf[16] & 0x02 ? "YES" : "no");
                Serial.print(" | strpot(cm)="); Serial.print(buf[14]);
                Serial.print(" door="); Serial.print(buf[15]);
#if CURRENT_SENSE_ENABLED
                Serial.print(" | currents(A):");
                for (uint8_t i = 0; i < NUM_CURRENT_SENSORS; i++) {
                    Serial.print(" CH");
                    Serial.print(i);
                    Serial.print("=");
                    Serial.print(buf[i] == 0xFF ? "N/A" : String(buf[i] * (20.0f / 255.0f), 1).c_str());
                }
#endif
#if ROTARY_ENCODERS_ENABLED
                Serial.print(" | enc(deg): L=");
                Serial.print(buf[8] == 0xFF ? "N/A" : String(buf[8] * (360.0f / 255.0f), 1).c_str());
                Serial.print(" R=");
                Serial.print(buf[9] == 0xFF ? "N/A" : String(buf[9] * (360.0f / 255.0f), 1).c_str());
#endif
                Serial.print(" | load(kg):");
                for (uint8_t i = 0; i < 4; i++) {
                    Serial.print(" LC");
                    Serial.print(i);
                    Serial.print("=");
                    Serial.print(buf[10 + i] == 0xFF ? "N/A" : String(buf[10 + i] * 0.1f, 2).c_str());
                }
                Serial.println();
            } else {
                Serial.print("[DATA] read failed, got ");
                Serial.print(count);
                Serial.print("/");
                Serial.print(RESPONSE_BYTES);
                Serial.println(" bytes");
            }
            break;
        }
        case 'P':{
            Serial.println(message);
            break;
        }
    }
}

void keyboard_init() {
    Serial.println("WASD Control Ready");
    Serial.println(message);
}

void keyboard_update() {
    while (Serial.available() > 0) {
        uint8_t b = Serial.read();

        if (escapeArmed) {
            escapeArmed = false;
            i2c_parent_sendByte(b);
            lastSendTime = millis();
            continue;
        }
        if (b == SERIAL_ESCAPE) {
            escapeArmed = true;
            continue;
        }

        processKey((char)b);
        lastSendTime = millis();
    }

    // Prevent child timeout from firing ESTOP across all subsystems.
    // GRP_CONTROL param 0 is a no-op on the child — it just resets lastCommandTime.
    if (millis() - lastSendTime >= KEEPALIVE_INTERVAL_MS) {
        i2c_parent_sendCommand(GRP_LOCO_STOP, 0);
        lastSendTime = millis();
    }
}
#endif
