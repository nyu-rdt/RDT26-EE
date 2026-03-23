#include <Arduino.h>
#include "parent.h"
#include "config.h"

#if USE_WASD
#include "keyboard.h"
#endif

void sendCommand(uint8_t group, uint8_t param) {
    i2c_parent_sendCommand(group, param);
}

void setup() {
    Serial.begin(9600);
    i2c_parent_init();
    delay(1000);
    sendCommand(GRP_LOCO_STOP, 0);
    delay(1000);

#if USE_WASD
    keyboard_init();
#else
    Serial.println("Sequence mode");
#endif
}

void loop() {
#if USE_WASD
    keyboard_update();
#else
    Serial.println("Sending: FORWARD 50%");
    sendCommand(GRP_FORWARD, SPEED_PARAM_50);
    delay(3000);

    Serial.println("Sending: TURN LEFT 25%");
    sendCommand(GRP_LEFT, SPEED_PARAM_25);
    delay(3000);

    Serial.println("Sending: BACKWARD 75%");
    sendCommand(GRP_BACKWARD, SPEED_PARAM_75);
    delay(3000);

    Serial.println("Sending: STOP");
    sendCommand(GRP_LOCO_STOP, 0);
    delay(3000);
#endif
}