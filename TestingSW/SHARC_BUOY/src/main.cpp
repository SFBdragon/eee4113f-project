#include <Arduino.h>

#define TRIGGER_PIN 5
#define ONBOARD_LED 8
#define BAUD_RATE 115200
#define SERIAL_TX_PIN 21
#define SERIAL_RX_PIN 20

// 24KB test dataset
#define DATA_SIZE 24576
uint8_t testData[DATA_SIZE];

// Packet configuration
#define MIN_PACKET_SIZE 50
#define MAX_PACKET_SIZE 10240

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);

  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

  // Fill test dataset with recognizable pattern
  for (int i = 0; i < DATA_SIZE; i++) {
    testData[i] = (i % 26) + 65; // A-Z repeating
  }

  Serial.println("SHARC Buoy Emulator Ready");
}

void sendPacket(uint8_t* data, size_t len) {
  // Pull GPIO high to wake STM32L4
  digitalWrite(TRIGGER_PIN, HIGH);
  digitalWrite(ONBOARD_LED, HIGH);
  delay(15); // Allow STM32L4 to wake and setup DMA (~10us + margin)

  Serial1.write(data, len);
  Serial1.flush();

  delay(5);
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);
}

void loop() {
  uint32_t bytesSent = 0;

  Serial.println("Starting 24KB transmission...");

  while (bytesSent < DATA_SIZE) {
    // Vary packet size between MIN and MAX
    uint32_t packetSize = random(MIN_PACKET_SIZE, MAX_PACKET_SIZE);
    if (bytesSent + packetSize > DATA_SIZE) {
      packetSize = DATA_SIZE - bytesSent;
    }

    sendPacket(&testData[bytesSent], packetSize);
    bytesSent += packetSize;

    Serial.printf("Sent %lu / %d bytes\n", bytesSent, DATA_SIZE);

    // Vary interval between 10ms and 60s
    uint32_t interval = random(10, 60000);
    Serial.printf("Next packet in %lu ms\n", interval);
    delay(interval);
  }

  Serial.println("Transmission complete. Waiting 10s before repeat.");
  delay(10000);
}