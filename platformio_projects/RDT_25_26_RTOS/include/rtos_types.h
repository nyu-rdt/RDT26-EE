#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float    loco_left_target;   // normalized -1..1 after duty scaling
    float    loco_right_target;
    float    excav_belt_target;  // normalized -1..1 after duty scaling
    int8_t   excav_vert_dir;     // -1=down, 0=stop, +1=up
    int8_t   depo_door_cmd;      // -1=close, 0=hold, +1=open
    int8_t   depo_vib_cmd;       // 0=off, 1=on
    uint32_t last_cmd_ms;        // millis() at last command receipt
    bool     sw_estop_requested; // set by CmdDecodeTask; cleared by SafetyTask after latching
} DesiredState;

typedef struct {
    float   motor_currents[8];   // amperes, from current mux
    float   string_pot_cm;       // excavation arm position in cm
    float   encoder_left_deg;    // left wheel encoder angle
    float   encoder_right_deg;   // right wheel encoder angle
    float   load_cells_kg[4];
    uint8_t depo_door_state;     // DepoDoorState enum value
    bool    relay_engaged;       // true = relay closed = system powered
} SensorSnapshot;

typedef struct {
    volatile uint8_t enabled;   // 0=disabled, 1=step pulses running
    volatile uint8_t dir;       // 0=down, 1=up
} StepperPulsePlan;
