#include <Arduino.h>
#include "keyboard.h"
#include "config.h"
#include "parent.h"

#if USE_WASD
static uint8_t speedLevel = 1;  // 0-3
static uint8_t currentMode = GRP_LOCO_STOP;

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
        // belt excav control
        case 'U':
                currentMode = GRP_EXCAVATION_BELT;
                i2c_parent_sendCommand(currentMode, FORWARD); // forward
                printStatus("FORWARD excav BELT");
            break;
        case 'J':
                currentMode = GRP_EXCAVATION_BELT;
                i2c_parent_sendCommand(currentMode, REVERSE); // reverse
                printStatus("BACKWARD excav BELT");
            break; 
        case 'H':
                currentMode = GRP_EXCAVATION_BELT;
                i2c_parent_sendCommand(currentMode, STOP); // stop
                printStatus("STOP excav BELT");
            break; 
        // vertical excav control
        case 'O':
                currentMode =  GRP_EXCAVATION_VERT;
                i2c_parent_sendCommand(currentMode, FORWARD); // up
                printStatus("UP excav");
            break;
        case 'L':
                currentMode = GRP_EXCAVATION_VERT;
                i2c_parent_sendCommand(currentMode, REVERSE); // down
                printStatus("DOWN excav");
            break; 
        case 'K':
                currentMode = GRP_EXCAVATION_VERT;
                i2c_parent_sendCommand(currentMode, STOP); // stop
                printStatus("STOP excav");
            break; 
        // deposition door control
        case 'R':
                currentMode = GRP_DEPOSITION_DOOR;
                i2c_parent_sendCommand(currentMode, FORWARD); // open
                printStatus("OPEN depo DOOR");
            break;
        case 'F':
                currentMode = GRP_DEPOSITION_DOOR;
                i2c_parent_sendCommand(currentMode, REVERSE); // close
                printStatus("CLOSE depo DOOR");
            break;
        case 'V':
                currentMode = GRP_DEPOSITION_DOOR;
                i2c_parent_sendCommand(currentMode, STOP); // stop
                printStatus("STOP depo DOOR");
            break;
        // deposition vibration control
        case 'T':
                currentMode = GRP_DEPOSITION_VIB;
                i2c_parent_sendCommand(currentMode, FORWARD); // on
                printStatus("ON depo VIB");
            break;
        case 'G':
                currentMode = GRP_DEPOSITION_VIB;
                i2c_parent_sendCommand(currentMode, STOP); // off
                printStatus("OFF depo VIB");
            break;
    }
}

void keyboard_init() {
    Serial.println("WASD Control Ready");
    Serial.println("W/A/S/D=Move | E/Q=Speed | X/Space=Stop | U/J/H=Belt Fwd/Rev/Stop | O/L/K=Vert Fwd/Rev/Stop | R/F/V=Door Open/Close/Stop | T/G=Vib On/Off");
}

void keyboard_update() {
    while (Serial.available() > 0) {
        char key = Serial.read();
        processKey(key);
    }
}
#endif
