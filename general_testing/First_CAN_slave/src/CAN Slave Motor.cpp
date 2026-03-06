#include <Arduino.h>
#include <Wire.h>
#include "config.h"

volatile uint8_t command = CMD_STOP;

void receiveEvent(int count)
{
    if (Wire.available())
    {
        command = Wire.read();
    }
}

void setup()
{
    pinMode(PWM_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);

    Wire.begin(I2C_SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);

    Serial.begin(115200);
}

void loop()
{
    switch (command)
    {
        case CMD_FORWARD:
            digitalWrite(IN1_PIN, HIGH);
            digitalWrite(IN2_PIN, LOW);
            analogWrite(PWM_PIN, 150);
            break;

        case CMD_BACKWARD:
            digitalWrite(IN1_PIN, LOW);
            digitalWrite(IN2_PIN, HIGH);
            analogWrite(PWM_PIN, 150);
            break;

        default:
            analogWrite(PWM_PIN, 0);
            break;
    }
}