#include <Arduino.h>
#include "config.h"
#include "deposition.h"
#include "depo_door_driver.h"
#include "vib_motor_driver.h"
#include "load_cells.h"
#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif

void DEPO_Init() {
    DEPO_DOOR_Init();
    VIB_Init();
}

void DEPO_Update() {
#if DEPOSITION_DOOR_ENABLE_CURRENT_STOP && CURRENT_SENSE_ENABLED
    const float* currents = CURRENT_SENSORS_GetBuffer();
    DEPO_DOOR_SetMeasuredCurrent(currents[DEPOSITION_DOOR_CURRENT_SENSOR_INDEX]);
#endif
    DEPO_DOOR_Update();
    // Keep load-cell sampling running so deposition module can read weights
    lc_update();
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
    VIB_EmergencyStop();
}

DepoDoorState DEPO_GetDoorState() {
    return DEPO_DOOR_GetState();
}

