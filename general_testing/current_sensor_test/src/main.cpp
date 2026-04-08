#include <Arduino.h>
#include <FlexCAN_T4.h>

// ── Pin / hardware config ────────────────────────────────────────────────────
#define CURRENT_INPUT_PIN    26
#define CURRENT_SELECT_PIN_0 27
#define CURRENT_SELECT_PIN_1 28
#define CURRENT_SELECT_PIN_2 29
#define NUM_CURRENT_SENSORS  8
#define RELAY_DRIVER_PIN 2

// Sensor: 0–20 A maps to 0–2 V on a 3.3 V ADC (0–1023 raw)
#define CURRENT_SCALING      (33.0f / 1023.0f)
#define CURRENT_PRINT_MS     500   // auto-print interval

#define VIB_MOTOR_PIN 30

#define CAN_ID_LEFT_MOTOR  0x4C
#define CAN_ID_RIGHT_MOTOR 0x78
#define CAN_BAUD_RATE      500000

// ── State ────────────────────────────────────────────────────────────────────
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

static float motorSpeed    = 0.3f;  // current speed fraction (0.1–1.0)
static bool  vibOn         = false;
static bool  autoPrint     = true;
static unsigned long lastPrintMs = 0;
static float currents[NUM_CURRENT_SENSORS] = {0};

// ── Current sensor ───────────────────────────────────────────────────────────
static void sensors_init() {
    pinMode(CURRENT_INPUT_PIN,    INPUT);
    pinMode(CURRENT_SELECT_PIN_0, OUTPUT);
    pinMode(CURRENT_SELECT_PIN_1, OUTPUT);
    pinMode(CURRENT_SELECT_PIN_2, OUTPUT);
}

static void sensors_read_all() {
    for (int ch = 0; ch < NUM_CURRENT_SENSORS; ch++) {
        digitalWrite(CURRENT_SELECT_PIN_0, ch & 0x01);
        digitalWrite(CURRENT_SELECT_PIN_1, (ch >> 1) & 0x01);
        digitalWrite(CURRENT_SELECT_PIN_2, (ch >> 2) & 0x01);
        delay(10);  // mux settling time
        currents[ch] = analogRead(CURRENT_INPUT_PIN) * CURRENT_SCALING;
    }
}

static void print_currents() {
    Serial.print("Currents (A): ");
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) {
        Serial.print("CH");
        Serial.print(i);
        Serial.print("=");
        Serial.print(currents[i], 2);
        if (i < NUM_CURRENT_SENSORS - 1) Serial.print("  ");
    }
    Serial.println();
}

// ── CAN helpers ──────────────────────────────────────────────────────────────
static void can_send_speed(uint32_t id, float speed) {
    CAN_message_t msg;
    msg.flags.extended = 1;
    msg.id  = id;
    msg.len = 4;
    int32_t val = (int32_t)(speed * 100000.0f);
    msg.buf[0] = (val >> 24) & 0xFF;
    msg.buf[1] = (val >> 16) & 0xFF;
    msg.buf[2] = (val >>  8) & 0xFF;
    msg.buf[3] =  val        & 0xFF;
    can1.write(msg);
}

static void drive(float left, float right) {
    // sign convention matches main firmware: left negated for same-direction drive
    can_send_speed(CAN_ID_LEFT_MOTOR,  -left);
    can_send_speed(CAN_ID_RIGHT_MOTOR,  right);
}

static void stop_motors() {
    drive(0.0f, 0.0f);
}

// ── Vib motor ────────────────────────────────────────────────────────────────
static void vib_set(bool on) {
    digitalWrite(VIB_MOTOR_PIN, on ? HIGH : LOW);
    vibOn = on;
    Serial.print("Vib: ");
    Serial.println(on ? "ON" : "OFF");
}

// ── Menu ─────────────────────────────────────────────────────────────────────
static void print_help() {
    Serial.println();
    Serial.println("=== current_sensor_test ===");
    Serial.println("  w / s      forward / backward");
    Serial.println("  a / d      turn left / turn right");
    Serial.println("  [space]    stop motors");
    Serial.println("  v          toggle vib motor");
    Serial.println("  + / -      speed up / down  (10% steps, current shown)");
    Serial.println("  c          print currents now");
    Serial.println("  p          toggle auto-print currents every 500 ms");
    Serial.println("  ?          this help");
    Serial.println("===========================");
    Serial.print("Speed: ");
    Serial.print((int)(motorSpeed * 100));
    Serial.println("%");
}

// ── Arduino entry points ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    sensors_init();

    can1.begin();
    can1.setBaudRate(CAN_BAUD_RATE);
    stop_motors();

    pinMode(VIB_MOTOR_PIN, OUTPUT);
    vib_set(false);

    print_help();
    pinMode(RELAY_DRIVER_PIN, OUTPUT);
    digitalWrite(RELAY_DRIVER_PIN, HIGH);  // energise relay (active HIGH)
}

void loop() {
    // Auto-print currents
    if (autoPrint && millis() - lastPrintMs >= CURRENT_PRINT_MS) {
        lastPrintMs = millis();
        sensors_read_all();
        print_currents();
    }

    // Keyboard
    if (!Serial.available()) return;
    char key = Serial.read();

    switch (key) {
        case 'w':
            drive(motorSpeed, motorSpeed);
            break;
        case 's':
            drive(-motorSpeed, -motorSpeed);
            break;
        case 'a':
            drive(-motorSpeed, motorSpeed);
            break;
        case 'd':
            drive(motorSpeed, -motorSpeed);
            break;
        case ' ':
            stop_motors();
            break;
        case 'v':
            vib_set(!vibOn);
            break;
        case '+':
        case '=':
            motorSpeed = min(motorSpeed + 0.1f, 1.0f);
            Serial.print("Speed: ");
            Serial.print((int)(motorSpeed * 100));
            Serial.println("%");
            break;
        case '-':
            motorSpeed = max(motorSpeed - 0.1f, 0.1f);
            Serial.print("Speed: ");
            Serial.print((int)(motorSpeed * 100));
            Serial.println("%");
            break;
        case 'c':
            sensors_read_all();
            print_currents();
            break;
        case 'p':
            autoPrint = !autoPrint;
            Serial.print("Auto-print: ");
            Serial.println(autoPrint ? "ON" : "OFF");
            break;
        case '?':
            print_help();
            break;
        default:
            break;
    }
}
