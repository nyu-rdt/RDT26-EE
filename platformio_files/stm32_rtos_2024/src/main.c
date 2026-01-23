/**
 * Main Application - Rover Control System
 * STM32F446RE + FreeRTOS
 * 
 * Using STM32Cube's bundled FreeRTOS middleware
 * Real-time deterministic control with preemptive scheduling
 */

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "pins.h"
#include "commands.h"
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

/* Data packet for I2C response */
static uint8_t dataPacket[3] = {POSITION_UNKNOWN, 0, 0};

/* HX711 scale instances */
static HX711_t scale1, scale2, scale3, scale4;

/* FreeRTOS objects */
static SemaphoreHandle_t stateMutex;
static QueueHandle_t commandQueue;

/* Task handles */
static TaskHandle_t commandTaskHandle;
static TaskHandle_t sensorTaskHandle;
static TaskHandle_t excavationTaskHandle;

/* ============== Function Prototypes ============== */
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CommandTask(void *pvParameters);
static void SensorTask(void *pvParameters);
static void ExcavationControlTask(void *pvParameters);
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
static bool move_to_locomotion_position(void);
static bool move_to_excavation_position(void);

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
    
    /* Initialize debug UART */
    UART_Debug_Init();
    UART_Println("Rover Control System - STM32F446RE + FreeRTOS");
    UART_Println("Using STM32Cube Middleware FreeRTOS");
    UART_Println("Initializing...");
    
    /* Initialize peripherals */
    CAN_Driver_Init();
    UART_Println("  CAN initialized");
    
    PWM_Driver_Init();
    UART_Println("  PWM initialized");
    
    ADC_Driver_Init();
    UART_Println("  ADC initialized");
    
    /* Initialize HX711 scales */
    HX711_Init(&scale1, HX711_1_DOUT_PORT, HX711_1_DOUT_PIN, HX711_1_CLK_PORT, HX711_1_CLK_PIN);
    HX711_Init(&scale2, HX711_2_DOUT_PORT, HX711_2_DOUT_PIN, HX711_2_CLK_PORT, HX711_2_CLK_PIN);
    HX711_Init(&scale3, HX711_3_DOUT_PORT, HX711_3_DOUT_PIN, HX711_3_CLK_PORT, HX711_3_CLK_PIN);
    HX711_Init(&scale4, HX711_4_DOUT_PORT, HX711_4_DOUT_PIN, HX711_4_CLK_PORT, HX711_4_CLK_PIN);
    
    HX711_SetScale(&scale1, CALIBRATION_FACTOR_1);
    HX711_SetScale(&scale2, CALIBRATION_FACTOR_2);
    HX711_SetScale(&scale3, CALIBRATION_FACTOR_3);
    HX711_SetScale(&scale4, CALIBRATION_FACTOR_4);
    
    HX711_Tare(&scale1, 10);
    HX711_Tare(&scale2, 10);
    HX711_Tare(&scale3, 10);
    HX711_Tare(&scale4, 10);
    UART_Println("  HX711 scales initialized");
    
    /* Initialize I2C (registers callbacks) */
    I2C_RegisterRequestCallback(I2C_DataRequestCallback);
    I2C_Driver_Init();
    UART_Println("  I2C slave initialized");
    
    /* Create FreeRTOS objects */
    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL) {
        UART_Println("ERROR: Failed to create mutex");
        Error_Handler();
    }
    
    commandQueue = xQueueCreate(16, sizeof(uint8_t));
    if (commandQueue == NULL) {
        UART_Println("ERROR: Failed to create queue");
        Error_Handler();
    }
    
    /* Create FreeRTOS tasks */
    BaseType_t ret;
    
    ret = xTaskCreate(CommandTask, "Command", 512, NULL, 3, &commandTaskHandle);
    if (ret != pdPASS) {
        UART_Println("ERROR: Failed to create Command task");
        Error_Handler();
    }
    
    ret = xTaskCreate(SensorTask, "Sensor", 256, NULL, 2, &sensorTaskHandle);
    if (ret != pdPASS) {
        UART_Println("ERROR: Failed to create Sensor task");
        Error_Handler();
    }
    
    ret = xTaskCreate(ExcavationControlTask, "Excavation", 256, NULL, 2, &excavationTaskHandle);
    if (ret != pdPASS) {
        UART_Println("ERROR: Failed to create Excavation task");
        Error_Handler();
    }
    
    UART_Println("Tasks created, starting scheduler...");
    UART_Println("CAN Control Initialized");
    
    /* Start FreeRTOS scheduler - never returns */
    vTaskStartScheduler();
    
    /* Should never reach here */
    UART_Println("ERROR: Scheduler returned!");
    while (1) {
        __NOP();
    }
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
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* ============== FreeRTOS Tasks ============== */

/**
 * Command Task - Processes I2C commands
 * Highest priority for responsive control
 */
static void CommandTask(void *pvParameters)
{
    (void)pvParameters;
    uint8_t cmd;
    
    for (;;) {
        /* Wait for command from queue (blocks until available) */
        if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            ProcessCommand(cmd);
            
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                lastCommand = (cmd == CMD_EMERGENCY_STOP) ? 1000 : cmd;
                xSemaphoreGive(stateMutex);
            }
            
            UART_PrintInt(cmd);
            UART_Println("");
        }
    }
}

/**
 * Sensor Task - Periodically reads sensors
 */
static void SensorTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        /* Read sensors */
        float length = get_string_length();
        float weight = get_weight();
        
        /* Update shared state */
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            currentStringLength = length;
            currentWeight = weight;
            update_data_packet();
            xSemaphoreGive(stateMutex);
        }
        
        /* Run at 20Hz (50ms period) */
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));
    }
}

/**
 * Excavation Control Task - Background position control
 */
static void ExcavationControlTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        /* This task can handle autonomous excavation movements */
        /* Currently monitors - actual movement triggered by commands */
        
        /* Run at 10Hz */
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
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
            move_to_locomotion_position();
            break;
        case CMD_EXCAVATION_POSITION:
            move_to_excavation_position();
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
}

/* ============== I2C Data Request Callback ============== */
static void I2C_DataRequestCallback(uint8_t *data, uint8_t *length)
{
    data[0] = dataPacket[0];
    data[1] = dataPacket[1];
    data[2] = dataPacket[2];
    *length = 3;
}

/* ============== I2C Command Callback (from ISR) ============== */
void I2C_CommandReceived(uint8_t cmd)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(commandQueue, &cmd, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentPosition = POSITION_LOCOMOTION;
        locomotionLengthThreshold = get_string_length();
        xSemaphoreGive(stateMutex);
    }
    UART_Println("Excavation Zeroed");
}

static bool move_to_locomotion_position(void)
{
    if (currentPosition == POSITION_LOCOMOTION) {
        return true;
    }
    
    excavation_down();
    TickType_t startTick = xTaskGetTickCount();
    
    while (currentStringLength > locomotionLengthThreshold) {
        if ((xTaskGetTickCount() - startTick) > pdMS_TO_TICKS(EXCAVATION_TIMEOUT_MS)) {
            excavation_stop();
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    excavation_stop();
    
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentPosition = POSITION_LOCOMOTION;
        xSemaphoreGive(stateMutex);
    }
    
    return true;
}

static bool move_to_excavation_position(void)
{
    if (currentPosition == POSITION_EXCAVATION) {
        return true;
    }
    
    excavation_up();
    TickType_t startTick = xTaskGetTickCount();
    
    while (currentStringLength < excavationLengthThreshold) {
        if ((xTaskGetTickCount() - startTick) > pdMS_TO_TICKS(EXCAVATION_TIMEOUT_MS)) {
            excavation_stop();
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    excavation_stop();
    
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentPosition = POSITION_EXCAVATION;
        xSemaphoreGive(stateMutex);
    }
    
    return true;
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

/* ============== FreeRTOS Hooks ============== */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    UART_Println("Stack Overflow!");
    taskDISABLE_INTERRUPTS();
    for (;;) {
        __NOP();
    }
}

void vApplicationMallocFailedHook(void)
{
    UART_Println("Malloc Failed!");
    taskDISABLE_INTERRUPTS();
    for (;;) {
        __NOP();
    }
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
