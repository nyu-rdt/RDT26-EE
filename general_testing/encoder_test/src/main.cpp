// Rotary encoder standalone test — Teensy 4.1
//
// Wiring (matches main firmware config):
//   ENC1_A → pin 21,  ENC1_B → pin 20
//   ENC2_A → pin 23,  ENC2_B → pin 22
//   All encoder pins pulled up internally.
//
// Serial menu (115200 baud):
//   p  — toggle auto-print every 250 ms
//   r  — reset both counts to zero
//   1  — print encoder 1 now
//   2  — print encoder 2 now
//   ?  — help

#include <Arduino.h>

// ── Pin config (mirror of main firmware config.h) ────────────────────────────
#define ENC1_A 21
#define ENC1_B 20
#define ENC2_A 23
#define ENC2_B 22

// 8192 counts/rev (4x quadrature × 2048 PPR)
#define ENCODER_COUNTS_PER_REV 8192.0f
#define DEGREES_PER_COUNT      (360.0f / ENCODER_COUNTS_PER_REV)

#define PRINT_INTERVAL_MS 250

// ── State ────────────────────────────────────────────────────────────────────
volatile long count1 = 0;
volatile long count2 = 0;

static bool autoPrint = true;
static unsigned long lastPrintMs = 0;

// ── ISRs ─────────────────────────────────────────────────────────────────────
// Direction convention matches 2024 hardware (A==B on edge → negative)
static void isr1A() { count1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? -1 : +1; }
static void isr1B() { count1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? -1 : +1; }
static void isr2A() { count2 += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? -1 : +1; }
static void isr2B() { count2 += (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ? -1 : +1; }

// ── Helpers ──────────────────────────────────────────────────────────────────
static float countsToAngle(long counts) {
    float angle = fmod(counts * DEGREES_PER_COUNT, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle;
}

// Atomic snapshot to avoid reading a partially-updated count from the ISR
static void getCountsAtomic(long &c1, long &c2) {
    noInterrupts();
    c1 = count1;
    c2 = count2;
    interrupts();
}

static void printEncoder(uint8_t num) {
    long c1, c2;
    getCountsAtomic(c1, c2);
    long c = (num == 1) ? c1 : c2;

    Serial.print("ENC");
    Serial.print(num);
    Serial.print(": counts=");
    Serial.print(c);
    Serial.print("  angle=");
    Serial.print(countsToAngle(c), 2);
    Serial.println(" deg");
}

static void printBoth() {
    long c1, c2;
    getCountsAtomic(c1, c2);

    Serial.print("ENC1: ");
    Serial.print(c1);
    Serial.print(" counts / ");
    Serial.print(countsToAngle(c1), 2);
    Serial.print(" deg    ENC2: ");
    Serial.print(c2);
    Serial.print(" counts / ");
    Serial.print(countsToAngle(c2), 2);
    Serial.println(" deg");
}

static void printHelp() {
    Serial.println();
    Serial.println("=== encoder_test ===");
    Serial.println("  p    toggle auto-print (250 ms)");
    Serial.println("  r    reset both counts to 0");
    Serial.println("  1    print encoder 1");
    Serial.println("  2    print encoder 2");
    Serial.println("  ?    this help");
    Serial.println("====================");
}

// ── Arduino entry points ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), isr1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B), isr1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), isr2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B), isr2B, CHANGE);

    printHelp();
}

void loop() {
    if (autoPrint && millis() - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = millis();
        printBoth();
    }

    if (!Serial.available()) return;
    char key = Serial.read();

    switch (key) {
        case 'p':
            autoPrint = !autoPrint;
            Serial.print("Auto-print: ");
            Serial.println(autoPrint ? "ON" : "OFF");
            break;
        case 'r':
            noInterrupts();
            count1 = 0;
            count2 = 0;
            interrupts();
            Serial.println("Counts reset.");
            break;
        case '1': printEncoder(1); break;
        case '2': printEncoder(2); break;
        case '?': printHelp();     break;
        default:  break;
    }
}
