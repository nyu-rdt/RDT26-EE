#include <Arduino.h>
#include "config.h"
#include "deposition.h"
#include "depo_door_driver.h"
#include "vib_motor_driver.h"

void DEPO_Init() {
    DEPO_DOOR_Init();
    VIB_Init();
}

void DEPO_Update(float* currents, int numCurrents) {
#if DEPOSITION_DOOR_ENABLE_CURRENT_STOP && CURRENT_SENSE_ENABLED
    if (currents && DEPOSITION_DOOR_CURRENT_SENSOR_INDEX < numCurrents) {
        DEPO_DOOR_SetMeasuredCurrent(currents[DEPOSITION_DOOR_CURRENT_SENSOR_INDEX]);
    }
#endif
    DEPO_DOOR_Update();
}

void DEPO_OpenDoor() {
    DEPO_DOOR_Open();
}

void DEPO_CloseDoor() {
    DEPO_DOOR_Close();
}

void DEPO_SetVib(int direction) {
    VIB_drive(direction);
}

void DEPO_EmergencyStop() {
    DEPO_DOOR_EmergencyStop();
    VIB_drive(0);
}

DepoDoorState DEPO_GetDoorState() {
    return DEPO_DOOR_GetState();
}

