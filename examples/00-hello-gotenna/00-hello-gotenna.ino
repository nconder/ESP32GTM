#include "ESP32GTM.h"

void setup() {
  // Enable serial port and introduce ourselves
  Serial.begin(115200);
  Serial.println("ESP32GTM EXAMPLE 00 - HELLO GOTENNA!");

  // Set the information level to "interesting"
  // (ESP_LOG_VERBOSE borders on "excessive")
  esp_log_level_set("*", ESP_LOG_DEBUG);

  // Run this once
  gtmInit();
}

void loop() {
  // Run this on every loop
  gtmLoop();

  // Delay a second between loops
  delay(1000);
}
