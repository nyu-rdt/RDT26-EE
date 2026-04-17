#pragma once

enum DepoDoorState {
    DEPO_DOOR_STATE_CLOSED = 0,
    DEPO_DOOR_STATE_OPENED,
    DEPO_DOOR_STATE_OPENING,
    DEPO_DOOR_STATE_CLOSING
};

void DEPO_DOOR_Init();
void DEPO_DOOR_SetDirection(int direction);
void DEPO_DOOR_Open();
void DEPO_DOOR_Close();
void DEPO_DOOR_EmergencyStop();
void DEPO_DOOR_Update();
void DEPO_DOOR_SetMeasuredCurrent(float currentAmps);
DepoDoorState DEPO_DOOR_GetState();
bool DEPO_DOOR_IsBusy();
