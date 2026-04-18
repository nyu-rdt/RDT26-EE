// test/test_sensor_cal/test_main.cpp
//
// Interactive sensor calibration tool — runs on the parent Teensy.
// Polls the child's 18-byte telemetry packet every 200 ms and displays all
// sensors live.  Motor control uses the same keys as keyboard.cpp.
// Press C to launch the calibration wizard for any sensor.
//
// Build + upload:  pio run -e teensy41_sensor_cal -t upload  (i2c_parent project)
// Monitor:         Serial Monitor @ 115200 baud  (line ending: Newline)
//
// ── Motor keys (same as keyboard.cpp) ────────────────────────────────────────
//   W/A/S/D   = fwd / left / back / right
//   E / Q     = speed up / speed down  (levels 0-3)
//   X / Space = stop
//   O / L / K = stepper up / down / stop
//   U / J / H = belt fwd / rev / stop
//   R / F     = door open / close
//   T / G     = vib on / off
//   C         = calibration wizard
//   P         = print this help
//
// ── Calibration wizard (C) ────────────────────────────────────────────────────
//   1 = String pot  — two-point (y = ax + b), suggests new STRING_POT_SCALE
//   2 = Load cell   — two-point per channel, suggests scale + offset
//   3 = Current     — one-point scale-only (no offset — baseline from converters
//                     is real current, not sensor error), suggests CURRENT_SCALING
//   4 = Encoder     — verify: rotate known degrees, reports measured vs expected
//
// During wizard positioning steps you can still use all motor keys.
// The wizard only blocks input while you are typing a number.
//
// ── Packet decode (child comms.cpp — keep in sync) ───────────────────────────
//   [0-7]  currents:  byte = current_A * 12.75   → A  = byte / 12.75
//   [8-9]  encoders:  byte = angle_deg*255/360   → deg = byte * 360/255
//   [10-13] load cells: uint8 each (0xFF = disabled)
//   [14]   string pot: byte = dist_cm directly   → cm = byte  (no scaling needed)
//   [15]   door state: 0=closed 1=opened 2=opening 3=closing
//   [16]   EE flags:   bit0=relay bit1=3s_low bit2=6s_low

#include <Arduino.h>
#include <Wire.h>
#include "parent.h"
#include "config.h"

// ── Decode constants ──────────────────────────────────────────────────────────
#warning "test_sensor_cal: kCurrentScaleChild, kSPScaleChild, kSPOffsetChild are mirrored from child config.h — update them here whenever the child firmware changes or calibration suggestions will be wrong."
static constexpr float kCurrentPackScale  = 12.75f;
static constexpr float kAngleScale        = 360.0f / 255.0f;
// Mirrored from child firmware config.h — keep in sync.
static constexpr float kCurrentScaleChild = 33.0f / 1023.0f;
static constexpr float kSPScaleChild  = 37.125f;
static constexpr float kSPOffsetChild = -3.511f;
static constexpr uint8_t kSentinel = 0xFF;

// ── Runtime state ─────────────────────────────────────────────────────────────
static uint8_t sPkt[RESPONSE_BYTES];
static bool    sPktValid   = false;
static uint8_t sSpeedLevel = 1;
static uint8_t sCurrentGrp = GRP_LOCO_STOP;

// Last command sent — used for keep-alive during wizard positioning steps
static uint8_t sLastGrp   = GRP_LOCO_STOP;
static uint8_t sLastParam  = 0;

static constexpr uint16_t kPollMs        = 200;
static constexpr uint16_t kKeepAliveMs   = 200;
static unsigned long sLastPollMs         = 0;
static bool sDashboardPaused             = false;

// ── Helpers ───────────────────────────────────────────────────────────────────
static float decodeCurrent(uint8_t b) { return b / kCurrentPackScale; }
static float decodeAngle(uint8_t b)   { return b * kAngleScale; }
// String pot pkt[14] is already dist_cm — no decode math needed
static float decodeStringPot(uint8_t b) { return (float)b; }

static const char* doorStr(uint8_t s) {
    switch (s) {
        case 0: return "CLOSED";   case 1: return "OPENED";
        case 2: return "OPENING";  case 3: return "CLOSING";
        default: return "?";
    }
}

// ── Command sending with keep-alive tracking ──────────────────────────────────
static void sendCmd(uint8_t grp, uint8_t param) {
    i2c_parent_sendCommand(grp, param);
    sLastGrp   = grp;
    sLastParam = param;
}

static void sendKeepAlive() {
    i2c_parent_sendCommand(sLastGrp, sLastParam);
}

// ── Dashboard ─────────────────────────────────────────────────────────────────
static void printDashboard() {
    Serial.print("\033[2J\033[H");
    Serial.print("=== SENSOR CALIBRATION  t=");
    Serial.print(millis() / 1000.0f, 1);
    Serial.print("s  | Speed: "); Serial.print(sSpeedLevel);
    Serial.println("/3  | [P]=help  [C]=calibrate ===\n");

    Serial.println("CURRENTS (ch0-7):");
    for (int i = 0; i < NUM_CURRENT_SENSORS; i++) {
        Serial.print("  ch"); Serial.print(i); Serial.print(": ");
        if (sPkt[i] == kSentinel) Serial.println("--- (disabled)");
        else {
            char buf[24];
            snprintf(buf, sizeof(buf), "%3u raw  %5.2f A", sPkt[i], decodeCurrent(sPkt[i]));
            Serial.println(buf);
        }
    }

    Serial.println("\nENCODERS:");
    if (sPkt[8] == kSentinel && sPkt[9] == kSentinel) {
        Serial.println("  disabled (0xFF)");
    } else {
        Serial.print("  Left : "); Serial.print(sPkt[8]);
        Serial.print(" raw  ->  "); Serial.print(decodeAngle(sPkt[8]), 1); Serial.println(" deg");
        Serial.print("  Right: "); Serial.print(sPkt[9]);
        Serial.print(" raw  ->  "); Serial.print(decodeAngle(sPkt[9]), 1); Serial.println(" deg");
    }

    Serial.println("\nSTRING POT:");
    if (sPkt[14] == kSentinel) Serial.println("  disabled (0xFF)");
    else {
        Serial.print("  "); Serial.print(sPkt[14]); Serial.print(" cm");
        if      (sPkt[14] < 11) Serial.print("  [!] near LOW limit");
        else if (sPkt[14] > 32) Serial.print("  [!] near HIGH limit");
        Serial.println();
    }

    Serial.println("\nLOAD CELLS:");
    bool allDis = (sPkt[10]==kSentinel && sPkt[11]==kSentinel &&
                   sPkt[12]==kSentinel && sPkt[13]==kSentinel);
    if (allDis) Serial.println("  disabled — set LOAD_CELLS_ENABLED=1 in child config.h");
    else {
        for (int i = 0; i < 4; i++) {
            Serial.print("  cell"); Serial.print(i); Serial.print(": ");
            if (sPkt[10+i] == kSentinel) Serial.println("disabled");
            else { Serial.print(sPkt[10+i]); Serial.println(" raw"); }
        }
    }

    Serial.println("\nDEPO DOOR:");
    if (sPkt[15] == kSentinel) Serial.println("  disabled (0xFF)");
    else { Serial.print("  "); Serial.println(doorStr(sPkt[15])); }

    Serial.println("\nEE FLAGS:");
    Serial.print("  relay=");  Serial.print( sPkt[16] & 0x01);
    Serial.print("  3s_low="); Serial.print((sPkt[16] >> 1) & 0x01);
    Serial.print("  6s_low="); Serial.println((sPkt[16] >> 2) & 0x01);
    Serial.println();
}

// ── Input helpers ─────────────────────────────────────────────────────────────

// Forward declaration so handleInput can be called from wizards
static void handleInput(char key);

// Waits for Enter. While waiting: processes motor keys and sends keep-alives
// so the child doesn't time out and motors stay running for calibration.
static void waitEnterWithMotor(const char* prompt) {
    Serial.println(prompt);
    Serial.println("  (Use motor keys to position. Stop with K/X before Enter.)");
    Serial.println("  (Keep-alive holds the last single command — stop loco before using stepper.)");
    unsigned long lastKa   = millis();
    unsigned long lastPoll = millis();
    while (true) {
        if (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                delay(5);
                while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r'))
                    Serial.read();
                break;
            }
            if (c != 'C' && c != 'c') handleInput(c);
        }
        unsigned long now = millis();
        if (now - lastKa >= kKeepAliveMs) {
            sendKeepAlive();
            lastKa = now;
        }
        // Keep sPkt fresh so wizard reads reflect current position.
        if (now - lastPoll >= kPollMs) {
            uint8_t n = i2c_parent_requestData(sPkt, RESPONSE_BYTES);
            sPktValid  = (n == RESPONSE_BYTES);
            lastPoll   = now;
        }
    }
    Serial.println();
}

// Blocks for a typed number. Sends keep-alives so an active command doesn't time
// out mid-entry and disturb the measurement.
static float readFloat(const char* prompt) {
    Serial.print(prompt);
    char buf[16]; uint8_t idx = 0;
    unsigned long lastKa = millis();
    while (true) {
        if (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                delay(5);
                while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r'))
                    Serial.read();
                break;
            }
            if (idx < sizeof(buf) - 1) buf[idx++] = c;
        }
        if (millis() - lastKa >= kKeepAliveMs) {
            sendKeepAlive();
            lastKa = millis();
        }
    }
    buf[idx] = '\0';
    Serial.println();
    return atof(buf);
}

static int readInt(const char* prompt) { return (int)readFloat(prompt); }

// ── Calibration wizards ───────────────────────────────────────────────────────

static void calStringPot() {
    Serial.println("\n=== STRING POT CALIBRATION ===");
    if (!sPktValid || sPkt[14] == kSentinel) {
        Serial.println("Disabled — set STRING_POT_ENABLED=1 in child config.h."); return;
    }

    // Point 1
    delay(2000);
    waitEnterWithMotor("Drive stepper to POSITION 1 (O=up L=down K=stop).");
    uint8_t raw1   = sPkt[14]; // already in cm
    float actual1  = readFloat("Measured actual distance at position 1 (cm): ");

    // Point 2
    delay(2000);
    waitEnterWithMotor("Drive stepper to POSITION 2 (O=up L=down K=stop).");
    uint8_t raw2   = sPkt[14];
    float actual2  = readFloat("Measured actual distance at position 2 (cm): ");

    int dr = (int)raw2 - (int)raw1;
    if (dr == 0) { Serial.println("Error: same reading at both positions."); return; }

    // pkt[14] is already dist_cm from child's linear function.
    // Two-point cal finds how child's dist maps to actual dist:
    //   actual_cm = a * child_cm + b
    // To fix the child: new_STRING_POT_SCALE = old * a, new_OFFSET = old_OFFSET * a - b
    float a = (actual2 - actual1) / (float)dr;
    float b = actual1 - a * raw1;
    float newScale  = kSPScaleChild * a;
    float newOffset = kSPOffsetChild * a - b;

    Serial.println("\n--- RESULT ---");
    Serial.print("  actual_cm = "); Serial.print(a, 4);
    Serial.print(" * child_cm + "); Serial.println(b, 3);
    Serial.print("  -> STRING_POT_SCALE  = "); Serial.println(newScale,  3);
    Serial.print("  -> STRING_POT_OFFSET = "); Serial.println(newOffset, 3);
    Serial.println("  Update: RDT_2025_26_TEENSY4_1/include/config.h");
    Serial.println("--------------\n");
}

static void calLoadCell() {
    Serial.println("\n=== LOAD CELL CALIBRATION ===");
    bool anyEnabled = (sPkt[10]!=kSentinel || sPkt[11]!=kSentinel ||
                       sPkt[12]!=kSentinel || sPkt[13]!=kSentinel);
    if (!sPktValid || !anyEnabled) {
        Serial.println("Disabled — set LOAD_CELLS_ENABLED=1 in child config.h."); return;
    }
    int ch = readInt("Channel (0-3): ");
    if (ch < 0 || ch > 3 || sPkt[10+ch] == kSentinel) {
        Serial.println("Channel unavailable."); return;
    }

    waitEnterWithMotor("Remove all weight (tare point).");
    uint8_t raw1  = sPkt[10 + ch];
    float actual1 = readFloat("Actual weight at tare (kg, usually 0.0): ");

    waitEnterWithMotor("Apply known reference weight.");
    uint8_t raw2  = sPkt[10 + ch];
    float actual2 = readFloat("Actual reference weight (kg): ");

    int dr = (int)raw2 - (int)raw1;
    if (dr == 0) { Serial.println("Error: same raw value at both points."); return; }

    float a = (actual2 - actual1) / (float)dr;
    float b = actual1 - a * raw1;

    Serial.println("\n--- RESULT ---");
    Serial.print("  ch"); Serial.print(ch);
    Serial.print(": weight_kg = "); Serial.print(a, 5);
    Serial.print(" * raw + "); Serial.println(b, 3);
    Serial.println("  Apply in load cell driver when implemented in child firmware.");
    Serial.println("--------------\n");
}

static void calCurrent() {
    Serial.println("\n=== CURRENT SENSOR CALIBRATION ===");
    Serial.println("No offset — baseline draw from converters is real current, not error.");
    Serial.println("Use a power meter in-line with the load you want to calibrate.");
    if (!sPktValid) { Serial.println("No packet data yet."); return; }

    int ch = readInt("Channel (0-7): ");
    if (ch < 0 || ch > 7 || sPkt[ch] == kSentinel) {
        Serial.println("Channel unavailable."); return;
    }

    waitEnterWithMotor("Drive a known load on this channel, read the power meter.");
    uint8_t raw     = sPkt[ch];
    float decoded   = decodeCurrent(raw);
    float actual    = readFloat("Actual current from power meter (A): ");

    if (raw == 0) { Serial.println("Error: raw byte is 0 — apply more load."); return; }

    // One-point, scale only.
    // decoded_A = raw / 12.75  →  actual_A = raw / new_pack_scale
    // correction = actual / decoded
    // new CURRENT_SCALING = old * correction
    float correction      = actual / decoded;
    float newCurrentScale = kCurrentScaleChild * correction;

    Serial.println("\n--- RESULT ---");
    Serial.print("  ch"); Serial.print(ch);
    Serial.print(":  raw="); Serial.print(raw);
    Serial.print("  decoded="); Serial.print(decoded, 2);
    Serial.print(" A  actual="); Serial.print(actual, 2); Serial.println(" A");
    Serial.print("  correction factor = "); Serial.println(correction, 4);
    Serial.print("  -> CURRENT_SCALING = "); Serial.println(newCurrentScale, 6);
    Serial.println("  Update: RDT_2025_26_TEENSY4_1/include/config.h");
    Serial.println("--------------\n");
}

static void verifyEncoder() {
    Serial.println("\n=== ENCODER VERIFICATION ===");
    if (!sPktValid || (sPkt[8]==kSentinel && sPkt[9]==kSentinel)) {
        Serial.println("Disabled — set ROTARY_ENCODERS_ENABLED=1 in child config.h."); return;
    }
    Serial.println("Note: for sub-degree accuracy use the child-side test_sensors env");
    Serial.println("(pkt encoding is 1.4 deg/step — coarse but enough to catch big errors).");

    int ch = readInt("Channel (1=Left, 2=Right): ");
    if (ch != 1 && ch != 2) { Serial.println("Invalid."); return; }
    uint8_t idx = (ch == 1) ? 8 : 9;

    waitEnterWithMotor("Mark a reference point on the wheel. Rotate by hand or with motor.");
    float startDeg = decodeAngle(sPkt[idx]);

    waitEnterWithMotor("Finish rotating the wheel to your target angle.");
    float endDeg = decodeAngle(sPkt[idx]);

    float measured = endDeg - startDeg;
    if (measured < -180.0f) measured += 360.0f;
    if (measured >  180.0f) measured -= 360.0f;

    float expected = readFloat("Angle you intended to rotate (deg, + = CW): ");
    float error    = measured - expected;

    Serial.println("\n--- RESULT ---");
    Serial.print("  Measured: "); Serial.print(measured, 1); Serial.println(" deg");
    Serial.print("  Expected: "); Serial.print(expected, 1); Serial.println(" deg");
    Serial.print("  Error:    "); Serial.print(error,    1); Serial.println(" deg");
    if (fabsf(error) < 3.0f) Serial.println("  -> OK");
    else Serial.println("  -> Check ENCODER_COUNTS_PER_REV in child config.h");
    Serial.println("--------------\n");
}

static void runCalWizard() {
    sDashboardPaused = true;
    Serial.println("\n=== CALIBRATION WIZARD ===");
    Serial.println("  1 = String pot");
    Serial.println("  2 = Load cell");
    Serial.println("  3 = Current sensor");
    Serial.println("  4 = Encoder verify");
    Serial.println("  0 = Cancel");
    int choice = readInt("Select: ");
    switch (choice) {
        case 1: calStringPot();  break;
        case 2: calLoadCell();   break;
        case 3: calCurrent();    break;
        case 4: verifyEncoder(); break;
        default: Serial.println("Cancelled."); break;
    }
    sDashboardPaused = false;
    sLastPollMs = 0;
}

// ── Help ──────────────────────────────────────────────────────────────────────
static void printHelp() {
    Serial.println("\n--- CONTROLS ---");
    Serial.println("  W/A/S/D   = fwd / left / back / right");
    Serial.println("  E / Q     = speed up / down (0-3)");
    Serial.println("  X / Space = stop");
    Serial.println("  O / L / K = stepper up / down / stop");
    Serial.println("  U / J / H = belt fwd / rev / stop");
    Serial.println("  R / F     = door open / close");
    Serial.println("  T / G     = vib on / off");
    Serial.println("  C         = calibration wizard");
    Serial.println("  P         = this help");
    Serial.println("----------------\n");
}

// ── Motor dispatch (mirrors keyboard.cpp) ─────────────────────────────────────
static void handleInput(char key) {
    if (key >= 'a' && key <= 'z') key -= 32;

    switch (key) {
        case 'W': sCurrentGrp = GRP_FORWARD;  sendCmd(GRP_FORWARD,  sSpeedLevel); break;
        case 'S': sCurrentGrp = GRP_BACKWARD; sendCmd(GRP_BACKWARD, sSpeedLevel); break;
        case 'A': sCurrentGrp = GRP_LEFT;     sendCmd(GRP_LEFT,     sSpeedLevel); break;
        case 'D': sCurrentGrp = GRP_RIGHT;    sendCmd(GRP_RIGHT,    sSpeedLevel); break;
        case 'E':
            if (sSpeedLevel < 3) {
                sSpeedLevel++;
                if (sCurrentGrp != GRP_LOCO_STOP) sendCmd(sCurrentGrp, sSpeedLevel);
            }
            break;
        case 'Q':
            if (sSpeedLevel > 0) {
                sSpeedLevel--;
                if (sCurrentGrp != GRP_LOCO_STOP) sendCmd(sCurrentGrp, sSpeedLevel);
            }
            break;
        case 'X': case ' ':
            sCurrentGrp = GRP_LOCO_STOP;
            sendCmd(GRP_LOCO_STOP, STOP);
            break;
        case 'O': sendCmd(GRP_EXCAVATION, FORWARD);     break;
        case 'L': sendCmd(GRP_EXCAVATION, REVERSE);     break;
        case 'K': sendCmd(GRP_EXCAVATION, STOP);        break;
        case 'U': sendCmd(GRP_EXCAVATION, FORWARD + 3); break;
        case 'J': sendCmd(GRP_EXCAVATION, REVERSE + 3); break;
        case 'H': sendCmd(GRP_EXCAVATION, STOP    + 3); break;
        case 'R': sendCmd(GRP_DEPOSITION, DEPO_DOOR_OPEN);  break;
        case 'F': sendCmd(GRP_DEPOSITION, DEPO_DOOR_CLOSE); break;
        case 'T': sendCmd(GRP_DEPOSITION, DEPO_VIB_ON);     break;
        case 'G': sendCmd(GRP_DEPOSITION, DEPO_VIB_OFF);    break;
        case 'C': runCalWizard(); break;
        case 'P': printHelp();    break;
        default: break;
    }
}

// ── Entry points ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    i2c_parent_init();
    delay(500);
    sendCmd(GRP_LOCO_STOP, 0);
    Serial.println("=== Sensor Calibration Tool ===");
    Serial.println("Child must run RDT_2025_26_TEENSY4_1 main firmware.");
    printHelp();
}

void loop() {
    if (!sDashboardPaused && millis() - sLastPollMs >= kPollMs) {
        sLastPollMs = millis();
        uint8_t n  = i2c_parent_requestData(sPkt, RESPONSE_BYTES);
        sPktValid   = (n == RESPONSE_BYTES);
        if (sPktValid) printDashboard();
        else Serial.println("No response from child — check wiring / firmware.");
    }

    if (Serial.available()) handleInput((char)Serial.read());
}
