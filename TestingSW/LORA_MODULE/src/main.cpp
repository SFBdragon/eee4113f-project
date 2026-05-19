#include <Arduino.h>

#define ONBOARD_LED 8
#define BAUD_RATE 115200
#define SERIAL_TX_PIN 21
#define SERIAL_RX_PIN 20

#define OFFLOAD_INTERVAL_MS 1000  // 5s between packets

struct Packet {
  const char* name;
  uint8_t     data[64];
  uint8_t     len;
};

const Packet PACKETS[] = {

  { "MIN_6",
    { 0xAA, 6, 0x01, 0x00, 0x00, 0x55 },
    6 },

  { "COUNT_8",
    { 0xAA, 8, 0x02, 0x01, 0x02, 0x03, 0x04, 0x55 },
    8 },

  { "ALL_FF_16",
    { 0xAA, 16, 0x03,
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
      0xFF, 0xFF, 0xFF,
      0x55 },
    16 },

  { "ALT_AA55_16",
    { 0xAA, 16, 0x04,
      0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
      0xAA, 0x55, 0xAA,
      0x55 },
    16 },

  { "ASCII_32",
    { 0xAA, 32, 0x05,
      'H','E','L','L','O',' ','W','O','R','L','D','!',
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x55 },
    32 },

  { "RAMP_32",
    { 0xAA, 32, 0x06,
      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
      0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
      0x18, 0x19, 0x1A, 0x1B,
      0x55 },
    32 },

  { "CHECKER_48",
    { 0xAA, 48, 0x07,
      0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0,
      0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0,
      0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0,
      0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0,
      0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0,
      0x0F, 0xF0, 0x0F, 0xF0, 0x55 },
    48 },

  { "MAX_64",
    { 0xAA, 64, 0x08,
      'M','A','X','_','P','K','T','!',
      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
      0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
      0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
      0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20,
      0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
      0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30,
      0x31, 0x32, 0x33, 0x34, 0x55 },
    64 },
};

const uint8_t NUM_PACKETS = sizeof(PACKETS) / sizeof(PACKETS[0]);

// ─── RX BUFFER ────────────────────────────────────────────────────────────────

#define RX_BUF_SIZE 128
uint8_t  rxBuf[RX_BUF_SIZE];
uint8_t  rxLen = 0;
uint32_t lastByteTime = 0;
#define  PACKET_TIMEOUT_MS 20   // flush & print if no new byte for 20 ms

// ─── PRINT HELPERS ────────────────────────────────────────────────────────────

void printRxBuffer() {
  if (rxLen == 0) return;

  Serial.println("\n<<< RECEIVED:");
  Serial.print("    HEX: ");
  for (uint8_t i = 0; i < rxLen; i++) {
    if (rxBuf[i] < 0x10) Serial.print("0");
    Serial.print(rxBuf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("  ASCII: ");
  for (uint8_t i = 0; i < rxLen; i++) {
    Serial.print((rxBuf[i] >= 0x20 && rxBuf[i] < 0x7F) ? (char)rxBuf[i] : '.');
  }
  Serial.print("  (");
  Serial.print(rxLen);
  Serial.println(" bytes)");

  // Quick sanity check against known packet signatures
  if (rxLen >= 3 && rxBuf[0] == 0xAA && rxBuf[rxLen - 1] == 0x55) {
    Serial.print("  FRAME: START=0xAA  LEN_FIELD=");
    Serial.print(rxBuf[1]);
    Serial.print("  SEQ=0x0");
    Serial.print(rxBuf[2], HEX);
    Serial.println("  END=0x55  [OK]");
  } else {
    Serial.println("  FRAME: [no recognised framing — raw dump above]");
  }

  rxLen = 0;   // reset for next packet
}

void sendPacket(const Packet& p) {
  Serial1.write(p.data, p.len);
  Serial1.flush();

  Serial.print("\n>>> SENDING: ");
  Serial.print(p.name);
  Serial.print("  (");
  Serial.print(p.len);
  Serial.println(" bytes)");

  Serial.print("    HEX: ");
  for (uint8_t i = 0; i < p.len; i++) {
    if (p.data[i] < 0x10) Serial.print("0");
    Serial.print(p.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("  ASCII: ");
  for (uint8_t i = 0; i < p.len; i++) {
    Serial.print((p.data[i] >= 0x20 && p.data[i] < 0x7F) ? (char)p.data[i] : '.');
  }
  Serial.println();
}

// ─── SETUP ────────────────────────────────────────────────────────────────────

void setup() {
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW);

  Serial.begin(115200);
  Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

  Serial.println("=== Packet Tester Ready ===");
  Serial.println("TX and RX on Serial1 — both printed here.");
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────

uint8_t  packetIndex    = 0;
uint32_t lastSendTime   = 0;

void loop() {

  // ── 1. Drain Serial1 RX into buffer ────────────────────────────────────────
  while (Serial1.available()) {
    if (rxLen < RX_BUF_SIZE) {
      rxBuf[rxLen++] = Serial1.read();
    } else {
      Serial1.read();   // discard overflow
    }
    lastByteTime = millis();
  }

  // ── 2. If gap detected, flush & pretty-print what arrived ──────────────────
  if (rxLen > 0 && (millis() - lastByteTime) > PACKET_TIMEOUT_MS) {
    printRxBuffer();
  }

  // ── 3. Send next packet on interval ────────────────────────────────────────
  if (millis() - lastSendTime >= OFFLOAD_INTERVAL_MS) {
    digitalWrite(ONBOARD_LED, HIGH);
    sendPacket(PACKETS[packetIndex]);
    packetIndex = (packetIndex + 1) % NUM_PACKETS;
    lastSendTime = millis();
    digitalWrite(ONBOARD_LED, LOW);
  }
}