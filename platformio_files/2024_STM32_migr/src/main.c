/**
 * Main Application - Rover Control System
 * STM32F446RE Bare-Metal Implementation
 * 
 * Migrated from Teensy 4.1 Arduino implementation
 * Time-triggered deterministic control without RTOS
 * 
 * Architecture:
 * - TIM6 provides 1kHz system tick
 * - Main loop checks scheduler flags for task execution
 * - I2C slave uses interrupt to buffer commands
 * - All processing is cooperative (non-preemptive)
 */

#include "stm32f4xx_hal.h"

#include "pins.h"
#include "commands.h"
#include "scheduler.h"
#include "can_driver.h"
#include "pwm_driver.h"
#include "i2c_driver.h"
#include "adc_driver.h"
#include "hx711_driver.h"
#include "uart_debug.h"

/* ============== Global State ============== */
static volatile ExcavationPosition_t currentPosition = POSITION_UNKNOWN;
static volatile bool isAutonomousMode = false;
static volatile float locomotionLengthThreshold = LOCOMOTION_LENGTH_DEFAULT;
static volatile float excavationLengthThreshold = EXCAVATION_LENGTH_DEFAULT;
static volatile float currentStringLength = 0.0f;
static volatile float currentWeight = 0.0f;
static volatile int lastCommand = 0;

/* Excavation state machine */
typedef enum {
    EXCAV_STATE_IDLE,
    EXCAV_STATE_MOVING_TO_LOCOMOTION,
    EXCAV_STATE_MOVING_TO_EXCAVATION,
    EXCAV_STATE_TIMEOUT
} ExcavationState_t;

static volatile ExcavationState_t excavationState = EXCAV_STATE_IDLE;
static volatile uint32_t excavationStartTick = 0;

/* Data packet for I2C response */
static uint8_t dataPacket[3] = {POSITION_UNKNOWN, 0, 0};

/* HX711 scale instances */
static HX711_t scale1, scale2, scale3, scale4;

/* ============== Function Prototypes ============== */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Task functions (called from main loop) */
static void Task_ProcessCommands(void);
static void Task_ReadSensors(void);
static void Task_ExcavationControl(void);
static void Task_DebugOutput(void);

/* Command processing */
static void ProcessCommand(uint8_t cmd);
static void I2C_DataRequestCallback(uint8_t *data, uint8_t *length);

/* Locomotion functions */
static void locomotion_stop(void);
static void go_forward(float speed_fac);
static void go_backward(float speed_fac);
static void turn_left(float speed_fac);
static void turn_right(float speed_fac);

/* Excavation functions */
static void excavation_stop(void);
static void excavation_up(void);
static void excavation_down(void);
static void belt_stop(void);
static void belt_outward(void);
static void belt_inward(void);
static void zero_excavation(void);
static void start_move_to_locomotion(void);
static void start_move_to_excavation(void);

/* Deposition functions */
static void rotate_collection(void);
static void rotate_dumping(void);
static void stop_rotating(void);

/* Emergency */
static void emergency_stop(void);

/* Sensor functions */
static float get_string_length(void);
static float get_weight(void);
static void update_data_packet(void);

/* ============== Main Entry Point ============== */
int main(void)
{
    /* HAL initialization */
    HAL_Init();
    
    /* Configure system clock to 180MHz */
    SystemClock_Config();
    
    /* Initialize scheduler first (provides HAL_GetTick) */
    Scheduler_Init();
    
    /* Initialize debug UART */
    UART_Debug_Init();
    UART_Println("Rover Control System - STM32F446RE");
    UART_Println("Bare-Metal Time-Triggered Architecture");
    UART_Println("Initializing peripherals...");
    
    /* Initialize CAN bus */
    CAN_Driver_Init();
    UART_Println("  CAN: 500kbps, extended IDs");
    
    /* Initialize PWM for servos */
    PWM_Driver_Init();
    UART_Println("  PWM: 50Hz servo control");
    
    /* Initialize ADC for string pot */
    ADC_Driver_Init();
    UART_Println("  ADC: String potentiometer");
    
    /* Initialize HX711 load cells */
    HX711_Init(&scale1, HX711_1_DOUT_PORT, HX711_1_DOUT_PIN, HX711_1_CLK_PORT, HX711_1_CLK_PIN);
    HX711_Init(&scale2, HX711_2_DOUT_PORT, HX711_2_DOUT_PIN, HX711_2_CLK_PORT, HX711_2_CLK_PIN);
    HX711_Init(&scale3, HX711_3_DOUT_PORT, HX711_3_DOUT_PIN, HX711_3_CLK_PORT, HX711_3_CLK_PIN);
    HX711_Init(&scale4, HX711_4_DOUT_PORT, HX711_4_DOUT_PIN, HX711_4_CLK_PORT, HX711_4_CLK_PIN);
    
    HX711_SetScale(&scale1, CALIBRATION_FACTOR_1);
    HX711_SetScale(&scale2, CALIBRATION_FACTOR_2);
    HX711_SetScale(&scale3, CALIBRATION_FACTOR_3);
    HX711_SetScale(&scale4, CALIBRATION_FACTOR_4);
    
    /* Tare scales with blocking delay (OK during init) */
    HX711_Tare(&scale1, 10);
    HX711_Tare(&scale2, 10);
    HX711_Tare(&scale3, 10);
    HX711_Tare(&scale4, 10);
    UART_Println("  HX711: 4 load cells calibrated");
    
    /* Initialize I2C slave (last, registers callbacks) */
    I2C_RegisterRequestCallback(I2C_DataRequestCallback);
    I2C_Driver_Init();
    UART_Println("  I2C: Slave 0x24");
    
    UART_Println("Initialization complete");
    UART_Println("Starting main loop...");
    UART_Println("CAN Control Initialized");
    
    /* ============== Main Super-Loop ============== */
    while (1)
    {
        /* Check scheduler flags and run tasks */
        
        /* Command processing - 100Hz (10ms) */
        if (Scheduler_ShouldRunCommand()) {
            Task_ProcessCommands();
        }
        
        /* Sensor reading - 20Hz (50ms) */
        if (Scheduler_ShouldRunSensor()) {
            Task_ReadSensors();
        }
        
        /* Excavation control - 50Hz (20ms) */
        if (Scheduler_ShouldRunControl()) {
            Task_ExcavationControl();
        }
        
        /* Debug output - 5Hz (200ms) */
        if (Scheduler_ShouldRunDebug()) {
            Task_DebugOutput();
        }
        
        /* Low power wait for interrupt (optional) */
        /* __WFI(); */
    }
}

/* ============== Task Implementations ============== */

/**
 * Process commands from I2C buffer
 * Runs at 100Hz
 */
static void Task_ProcessCommands(void)
{
    uint8_t cmd;
    
    /* Process all pending commands */
    while (I2C_GetCommand(&cmd)) {
        ProcessCommand(cmd);
        lastCommand = (cmd == CMD_EMERGENCY_STOP) ? 1000 : cmd;
    }
}

/**
 * Read sensors and update state
 * Runs at 20Hz
 */
static void Task_ReadSensors(void)
{
    /* Read string potentiometer */
    currentStringLength = get_string_length();
    
    /* Read load cells (this takes ~1ms per cell) */
    currentWeight = get_weight();
    
    /* Update I2C response packet */
    update_data_packet();
}

/**
 * Non-blocking excavation position control
 * Runs at 50Hz - uses state machine for async movement
 */
static void Task_ExcavationControl(void)
{
    uint32_t elapsed = Scheduler_GetTick() - excavationStartTick;
    
    switch (excavationState) {
        case EXCAV_STATE_IDLE:
            /* Nothing to do */
            break;
            
        case EXCAV_STATE_MOVING_TO_LOCOMOTION:
            if (currentStringLength <= locomotionLengthThreshold) {
                /* Reached target */
                excavation_stop();
                currentPosition = POSITION_LOCOMOTION;
                excavationState = EXCAV_STATE_IDLE;
                UART_Println("Reached Locomotion Position");
            } else if (elapsed > EXCAVATION_TIMEOUT_MS) {
                /* Timeout */
                excavation_stop();
                excavationState = EXCAV_STATE_TIMEOUT;
                UART_Println("Excavation Movement Timeout!");
            }
            break;
            
        case EXCAV_STATE_MOVING_TO_EXCAVATION:
            if (currentStringLength >= excavationLengthThreshold) {
                /* Reached target */
                excavation_stop();
                currentPosition = POSITION_EXCAVATION;
                excavationState = EXCAV_STATE_IDLE;
                UART_Println("Reached Excavation Position");
            } else if (elapsed > EXCAVATION_TIMEOUT_MS) {
                /* Timeout */
                excavation_stop();
                excavationState = EXCAV_STATE_TIMEOUT;
                UART_Println("Excavation Movement Timeout!");
            }
            break;
            
        case EXCAV_STATE_TIMEOUT:
            /* Stay in timeout until new command */
            break;
    }
}

/**
 * Debug output
 * Runs at 5Hz
 */
static void Task_DebugOutput(void)
{
    /* Periodic status (commented out to reduce noise) */
    /* 
    UART_Print("String: ");
    UART_PrintInt((int)currentStringLength);
    UART_Print(" Weight: ");
    UART_PrintInt((int)currentWeight);
    UART_Println("");
    */
}

/* ============== System Clock Configuration ============== */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    /* Configure HSE oscillator and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;  /* Nucleo uses 8MHz HSE from ST-Link */
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  /* 180MHz sysclk */
    RCC_OscInitStruct.PLL.PLLQ = 8;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    /* Enable overdrive mode for 180MHz */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }
    
    /* Configure clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   /* 180MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;    /* 45MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;    /* 90MHz */
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* ============== Command Processing ============== */
static void ProcessCommand(uint8_t cmd)
{
    switch (cmd) {
        case CMD_EMERGENCY_STOP:
            emergency_stop();
            break;
            
        case CMD_LOCOMOTION_STOP:
            locomotion_stop();
            break;
            
        case CMD_FORWARD_25:
            go_forward(0.25f);
            break;
        case CMD_FORWARD_50:
            go_forward(0.50f);
            break;
        case CMD_FORWARD_75:
            go_forward(0.75f);
            break;
        case CMD_FORWARD_100:
            go_forward(1.00f);
            break;
            
        case CMD_BACKWARD_25:
            go_backward(0.25f);
            break;
        case CMD_BACKWARD_50:
            go_backward(0.50f);
            break;
        case CMD_BACKWARD_75:
            go_backward(0.75f);
            break;
        case CMD_BACKWARD_100:
            go_backward(1.00f);
            break;
            
        case CMD_LEFT_25:
            turn_left(0.25f);
            break;
        case CMD_LEFT_50:
            turn_left(0.50f);
            break;
        case CMD_LEFT_75:
            turn_left(0.75f);
            break;
        case CMD_LEFT_100:
            turn_left(1.00f);
            break;
            
        case CMD_RIGHT_25:
            turn_right(0.25f);
            break;
        case CMD_RIGHT_50:
            turn_right(0.50f);
            break;
        case CMD_RIGHT_75:
            turn_right(0.75f);
            break;
        case CMD_RIGHT_100:
            turn_right(1.00f);
            break;
            
        case CMD_EXCAVATION_ZERO:
            zero_excavation();
            break;
        case CMD_EXCAVATION_LOCOMOTION_POS:
            start_move_to_locomotion();
            break;
        case CMD_EXCAVATION_POSITION:
            start_move_to_excavation();
            break;
            
        case CMD_BELT_STOP:
            belt_stop();
            break;
        case CMD_BELT_OUTWARD:
            belt_outward();
            break;
        case CMD_BELT_INWARD:
            belt_inward();
            break;
            
        case CMD_DEPOSITION_ROTATE_COLLECTION:
            rotate_collection();
            break;
        case CMD_DEPOSITION_ROTATE_DUMPING:
            rotate_dumping();
            break;
        case CMD_DEPOSITION_ROTATE_STOP:
            stop_rotating();
            break;
            
        case CMD_REQUEST_DATA:
            /* Data will be sent via I2C callback */
            break;
            
        case CMD_SWITCH_AUTONOMOUS:
            isAutonomousMode = true;
            UART_Println("Switched to Autonomous Mode");
            break;
            
        default:
            break;
    }
    
    /* Log command */
    UART_PrintInt(cmd);
    UART_Println("");
}

/* ============== I2C Data Request Callback ============== */
static void I2C_DataRequestCallback(uint8_t *data, uint8_t *length)
{
    /* Called from ISR context - keep it short */
    data[0] = dataPacket[0];
    data[1] = dataPacket[1];
    data[2] = dataPacket[2];
    *length = 3;
}

/* ============== Locomotion Functions ============== */
static void locomotion_stop(void)
{
    CAN_SendMotorCommand(FRONT_LEFT_MOTOR_CAN_ID, 0);
    CAN_SendMotorCommand(FRONT_RIGHT_MOTOR_CAN_ID, 0);
    CAN_SendMotorCommand(REAR_LEFT_MOTOR_CAN_ID, 0);
    CAN_SendMotorCommand(REAR_RIGHT_MOTOR_CAN_ID, 0);
    UART_Println("Locomotion Stopped");
}

static void go_forward(float speed_fac)
{
    float speed = speed_fac * DUTY_CYCLE_LIMIT;
    CAN_SendMotorCommand(REAR_RIGHT_MOTOR_CAN_ID, -speed);
    CAN_SendMotorCommand(FRONT_RIGHT_MOTOR_CAN_ID, speed);
    CAN_SendMotorCommand(FRONT_LEFT_MOTOR_CAN_ID, -speed);
    CAN_SendMotorCommand(REAR_LEFT_MOTOR_CAN_ID, speed);
    UART_Println("Moving Forward");
}

static void go_backward(float speed_fac)
{
    float speed = speed_fac * DUTY_CYCLE_LIMIT;
    CAN_SendMotorCommand(REAR_RIGHT_MOTOR_CAN_ID, speed);
    CAN_SendMotorCommand(FRONT_RIGHT_MOTOR_CAN_ID, -speed);
    CAN_SendMotorCommand(FRONT_LEFT_MOTOR_CAN_ID, speed);
    CAN_SendMotorCommand(REAR_LEFT_MOTOR_CAN_ID, -speed);
    UART_Println("Moving Backward");
}

static void turn_left(float speed_fac)
{
    float speed = speed_fac * DUTY_CYCLE_LIMIT;
    CAN_SendMotorCommand(FRONT_RIGHT_MOTOR_CAN_ID, -speed);
    CAN_SendMotorCommand(REAR_RIGHT_MOTOR_CAN_ID, speed);
    CAN_SendMotorCommand(REAR_LEFT_MOTOR_CAN_ID, speed);
    CAN_SendMotorCommand(FRONT_LEFT_MOTOR_CAN_ID, -speed);
    UART_Println("Turning Left");
}

static void turn_right(float speed_fac)
{
    float speed = speed_fac * DUTY_CYCLE_LIMIT;
    CAN_SendMotorCommand(FRONT_RIGHT_MOTOR_CAN_ID, speed);
    CAN_SendMotorCommand(REAR_RIGHT_MOTOR_CAN_ID, -speed);
    CAN_SendMotorCommand(REAR_LEFT_MOTOR_CAN_ID, -speed);
    CAN_SendMotorCommand(FRONT_LEFT_MOTOR_CAN_ID, speed);
    UART_Println("Turning Right");
}

/* ============== Excavation Functions ============== */
static void excavation_stop(void)
{
    PWM_ExcavationStop();
    UART_Println("Excavation Stopped");
}

static void excavation_up(void)
{
    PWM_ExcavationUp();
    UART_Println("Excavation Moving Up");
}

static void excavation_down(void)
{
    PWM_ExcavationDown();
    UART_Println("Excavation Moving Down");
}

static void belt_stop(void)
{
    PWM_BeltStop();
    UART_Println("Belt Stopped");
}

static void belt_outward(void)
{
    PWM_BeltOutward();
    UART_Println("Belt Moving Outward");
}

static void belt_inward(void)
{
    PWM_BeltInward();
    UART_Println("Belt Moving Inward");
}

static void zero_excavation(void)
{
    currentPosition = POSITION_LOCOMOTION;
    locomotionLengthThreshold = get_string_length();
    excavationState = EXCAV_STATE_IDLE;
    UART_Println("Excavation Zeroed");
}

/**
 * Start non-blocking move to locomotion position
 * Movement is monitored in Task_ExcavationControl
 */
static void start_move_to_locomotion(void)
{
    if (currentPosition == POSITION_LOCOMOTION) {
        return;
    }
    
    excavation_down();
    excavationStartTick = Scheduler_GetTick();
    excavationState = EXCAV_STATE_MOVING_TO_LOCOMOTION;
    UART_Println("Moving to Locomotion Position...");
}

/**
 * Start non-blocking move to excavation position
 * Movement is monitored in Task_ExcavationControl
 */
static void start_move_to_excavation(void)
{
    if (currentPosition == POSITION_EXCAVATION) {
        return;
    }
    
    excavation_up();
    excavationStartTick = Scheduler_GetTick();
    excavationState = EXCAV_STATE_MOVING_TO_EXCAVATION;
    UART_Println("Moving to Excavation Position...");
}

/* ============== Deposition Functions ============== */
static void rotate_collection(void)
{
    CAN_SendMotorCommand(DEPOSITION_MOTOR_CAN_ID, DEPOSITION_DUTY_CYCLE);
    UART_Println("Rotating to Collection Position");
}

static void rotate_dumping(void)
{
    CAN_SendMotorCommand(DEPOSITION_MOTOR_CAN_ID, -DEPOSITION_DUTY_CYCLE);
    UART_Println("Rotating to Dumping Position");
}

static void stop_rotating(void)
{
    CAN_SendMotorCommand(DEPOSITION_MOTOR_CAN_ID, 0);
    UART_Println("Rotation Stopped");
}

/* ============== Emergency Stop ============== */
static void emergency_stop(void)
{
    locomotion_stop();
    excavation_stop();
    belt_stop();
    stop_rotating();
    excavationState = EXCAV_STATE_IDLE;
    UART_Println("Emergency Stop Activated");
}

/* ============== Sensor Functions ============== */
static float get_string_length(void)
{
    return ADC_GetStringLength();
}

static float get_weight(void)
{
    float w1 = HX711_GetUnits(&scale1, 5);
    float w2 = HX711_GetUnits(&scale2, 5);
    float w3 = HX711_GetUnits(&scale3, 5);
    float w4 = HX711_GetUnits(&scale4, 5);
    return (w1 + w2 + w3 + w4) / 4.0f;
}

static void update_data_packet(void)
{
    dataPacket[0] = (uint8_t)currentPosition;
    dataPacket[1] = ((int)currentStringLength) >> 8;
    dataPacket[2] = ((int)currentWeight) & 0xFF;
}

/* ============== Error Handler ============== */
static void Error_Handler(void)
{
    UART_Println("Error Handler!");
    __disable_irq();
    while (1) {
        __NOP();
    }
}
