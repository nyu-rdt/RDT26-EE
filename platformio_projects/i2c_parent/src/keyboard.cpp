#include <Arduino.h>
#include "keyboard.h"
#include "config.h"
#include "parent.h"

#if USE_WASD
static uint8_t speedLevel = 1;  // 0-3
static uint8_t currentMode = GRP_LOCO_STOP;

char* message = "W/A/S/D=Move | E/Q=Speed | X/Space=Stop | U/J/H=Belt Fwd/Rev/Stop | O/L/K=Vert Fwd/Rev/Stop | R/F/V=Door Open/Close/Stop | T/G=Vib On/Off | I=Request Data";

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
        // deposition door control (old mapping: GRP_DEPOSITION params 0-2 = door stop/open/close)
        case 'R':
                currentMode = GRP_DEPOSITION;
                i2c_parent_sendCommand(currentMode, FORWARD); // door open
                printStatus("OPEN depo DOOR");
            break;
        case 'F':
                currentMode = GRP_DEPOSITION;
                i2c_parent_sendCommand(currentMode, REVERSE); // door close
                printStatus("CLOSE depo DOOR");
            break;
        case 'V':
                currentMode = GRP_DEPOSITION;
                i2c_parent_sendCommand(currentMode, STOP); // door stop
                printStatus("STOP depo DOOR");
            break;
        // deposition vibration control (old mapping: GRP_DEPOSITION params 3-5 = vib stop/on/rev)
        case 'T':
                currentMode = GRP_DEPOSITION;
                i2c_parent_sendCommand(currentMode, FORWARD + 3); // vib on
                printStatus("ON depo VIB");
            break;
        case 'G':
                currentMode = GRP_DEPOSITION;
                i2c_parent_sendCommand(currentMode, STOP + 3); // vib off
                printStatus("OFF depo VIB");
            break;
        case 'I': {
            uint8_t buf[RESPONSE_BYTES] = {};
            uint8_t count = i2c_parent_requestData(buf, RESPONSE_BYTES);
            if (count >= RESPONSE_BYTES) {
                // Packet layout matches child firmware DATA_PACKET_SIZE (18 bytes):
                //   [0-7]  motor currents  (CURRENT_SENSE_ENABLED)
                //   [8-9]  encoders        (ROTARY_ENCODERS_ENABLED)
                //   [16]   flags: bit0=relay, bit1=3s_low, bit2=6s_low
                Serial.print("[DATA] relay=");
                Serial.print(buf[16] & 0x01 ? "ON" : "OFF");
                Serial.print(" 3s_low=");
                Serial.print(buf[16] & 0x02 ? "YES" : "no");
                Serial.print(" 6s_low=");
                Serial.print(buf[16] & 0x04 ? "YES" : "no");
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
                Serial.print(" | encoders(deg): L=");
                Serial.print(buf[8] == 0xFF ? "N/A" : String(buf[8] * (360.0f / 255.0f), 1).c_str());
                Serial.print(" R=");
                Serial.print(buf[9] == 0xFF ? "N/A" : String(buf[9] * (360.0f / 255.0f), 1).c_str());
#endif
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
        char key = Serial.read();
        processKey(key);
    }
}
#endif
