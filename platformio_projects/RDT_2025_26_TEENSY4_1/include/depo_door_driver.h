#pragma once

enum DepoDoorState {
    DEPO_DOOR_STATE_STOPPED = 0,
    DEPO_DOOR_STATE_OPENING,
    DEPO_DOOR_STATE_CLOSING,
    DEPO_DOOR_STATE_OPENED,
    DEPO_DOOR_STATE_CLOSED,
    DEPO_DOOR_STATE_TIMEOUT_OPEN,
    DEPO_DOOR_STATE_TIMEOUT_CLOSE
};

void DEPO_DOOR_Init();
void DEPO_DOOR_SetDirection(int direction);
void DEPO_DOOR_Open();
void DEPO_DOOR_Close();
void DEPO_DOOR_Stop();
void DEPO_DOOR_Update();
void DEPO_DOOR_SetMeasuredCurrent(float currentAmps);
DepoDoorState DEPO_DOOR_GetState();
bool DEPO_DOOR_IsBusy();
