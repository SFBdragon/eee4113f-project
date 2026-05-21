#include <Arduino.h>

#define TRIGGER_PIN     22
#define ONBOARD_LED     2
#define BAUD_RATE       115200
#define SERIAL_TX_PIN   17
#define SERIAL_RX_PIN   16

#define PACKET_SIZE     (1024 * 6 / 5)

uint8_t packetData[PACKET_SIZE];

void build_packet(uint8_t* buf, uint8_t index) {
  memset(buf, index, PACKET_SIZE);
  buf[0] = 0xFF;
  buf[PACKET_SIZE - 1] = 0xFF;
}

void write_data(uint8_t* data, size_t len) {
  digitalWrite(TRIGGER_PIN, HIGH);
  digitalWrite(ONBOARD_LED, HIGH);
  delay(10);

  Serial1.write(data, len);
  Serial1.flush();

  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);
}

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);

  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

  Serial.println("ESP32 Ready");
}

void loop() {
  static uint8_t index = 0;

  build_packet(packetData, index);

  Serial.printf("Sending packet %02X  [FF %02X ... %02X FF]\n",
                index, index, index);

  write_data(packetData, PACKET_SIZE);

  index++;
  if (index >= 0xFF) index = 0;  // skip FF, reserved as marker

  delay(3000);
}