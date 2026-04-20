#include <Arduino.h>
#include "config.h"
#include "deposition.h"
#include "depo_door_driver.h"
#include "vib_motor_driver.h"
#if CURRENT_SENSE_ENABLED
#include "current_sensors.h"
#endif
#if LOAD_CELLS_ENABLED
#include "load_cell.h"
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

bool DEPO_IsBinEmpty() {
#if LOAD_CELLS_ENABLED
    return LOAD_CELL_GetTotalMass() < LOAD_CELL_EMPTY_THRESHOLD_KG;
#else
    return false;
#endif
}

