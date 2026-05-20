#include <Arduino.h>

#define TRIGGER_PIN     22    // GPIO22 to toggle high/low
#define ONBOARD_LED     2     // ESP32 DevKit onboard LED
#define BAUD_RATE       115200
#define SERIAL_TX_PIN   17    // UART2 TX
#define SERIAL_RX_PIN   16    // UART2 RX

// 24KB test dataset
#define DATA_SIZE 24576
uint8_t testData[DATA_SIZE];

void write_data(uint8_t* data, size_t len) {

  digitalWrite(TRIGGER_PIN, HIGH);
  digitalWrite(ONBOARD_LED, HIGH);
  Serial.println("GPIO23 HIGH - Sending data...");
  delay(100);
  Serial1.write(data, len);
  Serial1.flush();
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);
  Serial.println("GPIO23 LOW");
}

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ONBOARD_LED, LOW);

  Serial.begin(BAUD_RATE);
  Serial1.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

  // Fill test dataset with recognizable pattern (A-Z repeating)
  for (int i = 0; i < DATA_SIZE; i++) {
    testData[i] = (i % 26) + 65;
  }

  Serial.println("ESP32 DevKit Ready");
}

void loop() {
  // Toggle GPIO23 high

  write_data(testData, DATA_SIZE);
  Serial.printf("Sent %d bytes over Serial1 TX (GPIO%d)\n", DATA_SIZE, SERIAL_TX_PIN);

  // Toggle GPIO23 low
  delay(1000);
}