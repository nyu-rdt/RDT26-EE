#include <Arduino.h>
#include <FlexCAN_T4.h>

// Teensy CAN bus instance (matches i2c_can_child_tony style).
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// Motor CAN IDs requested for this rig.
static const uint32_t CAN_ID_MOTOR_A = 0x48;
static const uint32_t CAN_ID_MOTOR_B = 0x78;
static const uint32_t CAN_BAUD_RATE = 500000;

// Match child locomotion scaling behavior: command speed fraction * 0.33.
static const float LOCOMOTION_DUTY_SCALE = 0.3f;
static const float SPEED_STEPS[5] = {0.0f, 0.25f, 0.50f, 0.75f, 1.0f};

enum DriveMode : uint8_t {
	MODE_STOP = 0,
	MODE_FORWARD,
	MODE_BACKWARD,
	MODE_LEFT,
	MODE_RIGHT
};

static DriveMode driveMode = MODE_STOP;
static uint8_t speedLevel = 0;  // User-selected target speed level: 0..4.
static float currentMotorA = 0.0f;
static float currentMotorB = 0.0f;
static unsigned long lastTxMs = 0;
static const unsigned long TX_PERIOD_MS = 20;
static const float MAX_SPEED_DELTA_PER_TICK = 0.01f;

static void sendMotorSpeed(uint32_t canId, float speed) {
	CAN_message_t msg;
	msg.flags.extended = 1;
	msg.id = canId;
	msg.len = 4;

	int32_t value = (int32_t)(speed * 100000.0f);
	msg.buf[0] = (value >> 24) & 0xFF;
	msg.buf[1] = (value >> 16) & 0xFF;
	msg.buf[2] = (value >> 8) & 0xFF;
	msg.buf[3] = value & 0xFF;

	can1.write(msg);
}

static void sendLocomotion(float motorA, float motorB) {
	sendMotorSpeed(CAN_ID_MOTOR_A, motorA);
	sendMotorSpeed(CAN_ID_MOTOR_B, motorB);
}

static float currentSpeed() {
	return SPEED_STEPS[speedLevel] * LOCOMOTION_DUTY_SCALE;
}

static void getTargetLocomotion(float &motorA, float &motorB) {
	const float spd = currentSpeed();
	motorA = 0.0f;
	motorB = 0.0f;

	switch (driveMode) {
		case MODE_FORWARD:
			motorA = -spd;
			motorB = spd;
			break;
		case MODE_BACKWARD:
			motorA = spd;
			motorB = -spd;
			break;
		case MODE_LEFT:
			motorA = spd;
			motorB = spd;
			break;
		case MODE_RIGHT:
			motorA = -spd;
			motorB = -spd;
			break;
		case MODE_STOP:
		default:
			motorA = 0.0f;
			motorB = 0.0f;
			break;
	}
}

static float slewToward(float current, float target, float maxDelta) {
	float delta = target - current;
	if (delta > maxDelta) {
		return current + maxDelta;
	}
	if (delta < -maxDelta) {
		return current - maxDelta;
	}
	return target;
}

static void updateRampedLocomotion() {
	float motorA = 0.0f;
	float motorB = 0.0f;
	getTargetLocomotion(motorA, motorB);

	currentMotorA = slewToward(currentMotorA, motorA, MAX_SPEED_DELTA_PER_TICK);
	currentMotorB = slewToward(currentMotorB, motorB, MAX_SPEED_DELTA_PER_TICK);

	sendLocomotion(currentMotorA, currentMotorB);
}

static void printStatus() {
	Serial.print("Mode: ");
	switch (driveMode) {
		case MODE_FORWARD: Serial.print("FORWARD"); break;
		case MODE_BACKWARD: Serial.print("BACKWARD"); break;
		case MODE_LEFT: Serial.print("LEFT"); break;
		case MODE_RIGHT: Serial.print("RIGHT"); break;
		case MODE_STOP:
		default: Serial.print("STOP"); break;
	}

	Serial.print(" | Speed Level: ");
	Serial.print(speedLevel);
	Serial.print("/4");
	Serial.print(" | Effective Duty: ");
	Serial.print(currentSpeed(), 4);
	Serial.print(" | Output A/B: ");
	Serial.print(currentMotorA, 4);
	Serial.print(" / ");
	Serial.println(currentMotorB, 4);
}

static void processKey(char key) {
	if (key >= 'a' && key <= 'z') {
		key = (char)(key - 'a' + 'A');
	}

	bool changed = false;
	switch (key) {
		case 'W':
			driveMode = MODE_FORWARD;
			changed = true;
			break;
		case 'A':
			driveMode = MODE_LEFT;
			changed = true;
			break;
		case 'S':
			driveMode = MODE_BACKWARD;
			changed = true;
			break;
		case 'D':
			driveMode = MODE_RIGHT;
			changed = true;
			break;
		case 'E':
			if (speedLevel < 4) {
				speedLevel++;
				changed = true;
			}
			break;
		case 'Q':
			if (speedLevel > 0) {
				speedLevel--;
				changed = true;
			}
			break;
		case 'X':
		case ' ':
			driveMode = MODE_STOP;
			speedLevel = 0;
			changed = true;
			break;
		default:
			break;
	}

	if (changed) {
		printStatus();
	}
}

void setup() {
	Serial.begin(115200);

	can1.begin();
	can1.setBaudRate(CAN_BAUD_RATE);

	delay(200);
	currentMotorA = 0.0f;
	currentMotorB = 0.0f;
	sendLocomotion(0.0f, 0.0f);

	Serial.println("michael_rig ready");
	Serial.println("Keys: W/A/S/D move, E speed up, Q slow down, X or SPACE stop");
	Serial.println("Speed starts at 0");
	printStatus();
}

void loop() {
	while (Serial.available() > 0) {
		char key = (char)Serial.read();
		processKey(key);
	}

	// Periodic resend keeps commanded state alive on motor controllers.
	if (millis() - lastTxMs >= TX_PERIOD_MS) {
		lastTxMs = millis();
		updateRampedLocomotion();
	}
}
