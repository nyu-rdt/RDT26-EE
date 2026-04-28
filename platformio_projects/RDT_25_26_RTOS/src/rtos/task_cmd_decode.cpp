#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

static float speed(uint8_t param, float duty) {
    if (param > 3) param = 3;
    return kSpeedTable[param] * duty;
}

void TaskCmdDecode(void*) {
#if RTOS_SERIAL_DEBUG
    Serial.println("[cmd] task started");
#endif
    uint8_t raw;
    for (;;) {
        if (xQueueReceive(qI2cRxBytes, &raw, portMAX_DELAY) != pdTRUE) continue;

        uint8_t group = CMD_GROUP(raw);
        uint8_t param = CMD_PARAM(raw);

        EventBits_t safety = xEventGroupGetBits(egSafetyBits);
        bool hwEstop = (safety & SAFETY_HW_ESTOP) != 0;

        // Gate: block motion/mechanism groups while HW e-stop active
        bool isMotionGroup = (group >= GRP_LOCO_STOP && group <= GRP_DEPOSITION);
        if (hwEstop && isMotionGroup) {
            diag_cmd_rejected_killswitch++;
#if RTOS_SERIAL_DEBUG
            Serial.printf("[CMD] REJECTED (HW estop) raw=0x%02X grp=%u param=%u\n", raw, group, param);
#endif
            continue;
        }

#if RTOS_SERIAL_DEBUG
        Serial.printf("[CMD] raw=0x%02X grp=%u param=%u\n", raw, group, param);
#endif

        if (xSemaphoreTake(mDesiredState, pdMS_TO_TICKS(5)) != pdTRUE) continue;

        gDesiredState.last_cmd_ms = millis();

        switch (group) {
            case GRP_CONTROL:
                if (param == CTRL_SW_ESTOP) {
                    // Zero all motion targets and raise the flag for SafetyTask to
                    // latch into SAFETY_SW_ESTOP — keeps SafetyTask as sole bit writer.
                    gDesiredState.loco_left_target  = 0.0f;
                    gDesiredState.loco_right_target = 0.0f;
                    gDesiredState.excav_belt_target = 0.0f;
                    gDesiredState.excav_vert_dir    = 0;
                    gDesiredState.depo_door_cmd     = 0;
                    gDesiredState.depo_vib_cmd      = 0;
                    gDesiredState.sw_estop_requested = true;
                }
                break;

            case GRP_LOCO_STOP:
                gDesiredState.loco_left_target  = 0.0f;
                gDesiredState.loco_right_target = 0.0f;
                break;

            case GRP_FORWARD: {
                float s = speed(param, LOCO_DUTY);
                gDesiredState.loco_left_target  = -s;
                gDesiredState.loco_right_target =  s;
                break;
            }
            case GRP_BACKWARD: {
                float s = speed(param, LOCO_DUTY);
                gDesiredState.loco_left_target  =  s;
                gDesiredState.loco_right_target = -s;
                break;
            }
            case GRP_LEFT: {
                float s = speed(param, LOCO_DUTY);
                gDesiredState.loco_left_target  =  s;
                gDesiredState.loco_right_target =  s;
                break;
            }
            case GRP_RIGHT: {
                float s = speed(param, LOCO_DUTY);
                gDesiredState.loco_left_target  = -s;
                gDesiredState.loco_right_target = -s;
                break;
            }
            case GRP_EXCAVATION:
                if (param <= EXCAV_VERT_UP) {
                    gDesiredState.excav_vert_dir = (param == EXCAV_VERT_DOWN) ? -1
                                                 : (param == EXCAV_VERT_UP)   ?  1 : 0;
                } else if (param <= EXCAV_BELT_REV) {
                    uint8_t bp = param - 3;
                    gDesiredState.excav_belt_target = (bp == 1) ?  EXCAV_BELT_DUTY
                                                    : (bp == 2) ? -EXCAV_BELT_DUTY : 0.0f;
                } else {
                    diag_cmd_invalid++;
                }
                break;

            case GRP_DEPOSITION:
                if (param == DEPO_DOOR_OPEN)  gDesiredState.depo_door_cmd =  1;
                if (param == DEPO_DOOR_CLOSE) gDesiredState.depo_door_cmd = -1;
                if (param == DEPO_VIB_ON)     gDesiredState.depo_vib_cmd  =  1;
                if (param == DEPO_VIB_OFF)    gDesiredState.depo_vib_cmd  =  0;
                if (param > DEPO_VIB_OFF)     diag_cmd_invalid++;
                break;

            case GRP_DATA:
                // No state change - TelemetryTask handles the response
                break;

            default:
                diag_cmd_invalid++;
                break;
        }
        xSemaphoreGive(mDesiredState);
    }
}
