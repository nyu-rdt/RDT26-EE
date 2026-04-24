#include <Arduino.h>
#include <climits>
#include <FlexCAN_T4.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
#include "shared_state.h"
#include "safety_bits.h"
#include "rtos_config.h"

static void sendCAN(uint32_t id, float speed) {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = id;
    msg.len = 4;
    int32_t val = (int32_t)(speed * 100000.0f);
    msg.buf[0] = (val >> 24) & 0xFF;
    msg.buf[1] = (val >> 16) & 0xFF;
    msg.buf[2] = (val >>  8) & 0xFF;
    msg.buf[3] =  val        & 0xFF;
    gCan.write(msg);
}

static float slew(float cur, float tgt, float maxDelta) {
    float d = tgt - cur;
    if (d >  maxDelta) d =  maxDelta;
    if (d < -maxDelta) d = -maxDelta;
    return cur + d;
}

void TaskMotorControl(void*) {
    float curLeft  = 0.0f, curRight  = 0.0f, curExcav = 0.0f;
    float tgtLeft  = 0.0f, tgtRight  = 0.0f, tgtExcav = 0.0f;
    bool  stopped  = true;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        uint32_t notif = 0;
        if (xTaskNotifyWait(0, ULONG_MAX, &notif, 0) == pdTRUE) {
            if (notif == 1U) stopped = true;
            else             stopped = false;
        }

        if (xEventGroupGetBits(egSafetyBits) & SAFETY_ANY_STOP) stopped = true;

        if (!stopped) {
            if (xSemaphoreTake(mDesiredState, 0) == pdTRUE) {
                tgtLeft  = gDesiredState.loco_left_target;
                tgtRight = gDesiredState.loco_right_target;
                tgtExcav = gDesiredState.excav_belt_target;
                xSemaphoreGive(mDesiredState);
            }
        } else {
            tgtLeft = tgtRight = tgtExcav = 0.0f;
        }

        curLeft  = slew(curLeft,  tgtLeft,  0.01f);
        curRight = slew(curRight, tgtRight, 0.01f);
        curExcav = slew(curExcav, tgtExcav, 0.01f);

        if (xSemaphoreTake(mCanTx, pdMS_TO_TICKS(5)) == pdTRUE) {
            sendCAN(0x4C, curLeft);    // CAN_ID_LEFT_MOTOR
            sendCAN(0x78, curRight);   // CAN_ID_RIGHT_MOTOR
            sendCAN(0x48, curExcav);   // CAN_ID_EXCAVATION_MOTOR
            xSemaphoreGive(mCanTx);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PERIOD_MOTOR_CTRL_MS));
    }
}
