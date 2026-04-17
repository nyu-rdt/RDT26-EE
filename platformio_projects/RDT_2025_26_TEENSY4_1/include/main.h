#pragma once
#include "config.h"

static void ROVER_init();
static void ROVER_update();

static void receiveEvent(int numBytes);
static void processCommand(uint8_t cmd);
static void registerHandlers();

static void grp_Control(uint8_t param);
static void grp_LocoStop(uint8_t param);
static void grp_Forward(uint8_t param);
static void grp_Backward(uint8_t param);
static void grp_TurnLeft(uint8_t param);
static void grp_TurnRight(uint8_t param);
#if USE_OLD_HEX_MAPPING
static void grp_Excavation(uint8_t param);
static void grp_Deposition(uint8_t param);
#endif
static void grp_ExcavationBelt(uint8_t param);
static void grp_ExcavationVert(uint8_t param);
static void grp_DepositionDoor(uint8_t param);
static void grp_DepositionVib(uint8_t param);
static void grp_Data(uint8_t param);
