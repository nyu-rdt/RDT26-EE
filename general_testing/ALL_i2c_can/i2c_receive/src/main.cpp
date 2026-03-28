#include <Arduino.h>
#include <Wire.h>

// Minimal I2C bus activity logger for Teensy as slave.
// Goal: confirm whether the master is attempting communication at all.

static constexpr uint8_t I2C_ADDR = 0x08;
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr size_t RX_BUF_SIZE = 64;

volatile uint32_t g_rxEvents = 0;
volatile uint32_t g_txRequests = 0;
volatile uint32_t g_rxBytesTotal = 0;
volatile uint32_t g_zeroLenRx = 0;
volatile uint32_t g_readMismatch = 0;
volatile uint8_t g_lastLen = 0;
volatile uint8_t g_lastData[RX_BUF_SIZE];

void onReceiveHandler(int byteCount) {
	g_rxEvents++;
	if (byteCount <= 0) {
		g_zeroLenRx++;
		g_lastLen = 0;
		return;
	}

	uint8_t readCount = 0;
	while (Wire.available() && readCount < RX_BUF_SIZE) {
		g_lastData[readCount++] = static_cast<uint8_t>(Wire.read());
	}

	// Drain any extra bytes if master sent more than our debug buffer.
	while (Wire.available()) {
		(void)Wire.read();
		readCount++;
	}

	g_lastLen = readCount;
	g_rxBytesTotal += readCount;

	if (readCount != static_cast<uint8_t>(byteCount)) {
		g_readMismatch++;
	}
}

void onRequestHandler() {
	g_txRequests++;

	// Return a simple marker byte so the master can read something predictable.
	// 0xA5 is arbitrary and easy to recognize in logs.
	Wire.write(0xA5);
}

void printLastPayload() {
	if (g_lastLen == 0) {
		Serial.println("last_payload: <none>");
		return;
	}

	Serial.print("last_payload_hex:");
	for (uint8_t i = 0; i < g_lastLen; i++) {
		Serial.print(' ');
		if (g_lastData[i] < 0x10) {
			Serial.print('0');
		}
		Serial.print(g_lastData[i], HEX);
	}
	Serial.println();
}

void setup() {
	Serial.begin(SERIAL_BAUD);
	while (!Serial && millis() < 5000) {
		// Allow time for USB serial to open on boot.
	}

	Wire.begin(I2C_ADDR);
	Wire.onReceive(onReceiveHandler);
	Wire.onRequest(onRequestHandler);

	Serial.println("i2c_receive activity logger started");
	Serial.print("i2c_addr: 0x");
	if (I2C_ADDR < 0x10) {
		Serial.print('0');
	}
	Serial.println(I2C_ADDR, HEX);
	Serial.println("watch for rx_events / tx_requests changes");
}

void loop() {
	static uint32_t lastPrintMs = 0;
	const uint32_t now = millis();

	if (now - lastPrintMs >= 1000) {
		lastPrintMs = now;

		Serial.print("rx_events=");
		Serial.print(g_rxEvents);
		Serial.print(" rx_bytes_total=");
		Serial.print(g_rxBytesTotal);
		Serial.print(" tx_requests=");
		Serial.print(g_txRequests);
		Serial.print(" zero_len_rx=");
		Serial.print(g_zeroLenRx);
		Serial.print(" read_mismatch=");
		Serial.println(g_readMismatch);

		printLastPayload();
	}
}
