/*
 * ESP32GTM library example 01 - Hello goTenna!
 * 
 * Demonstrates the default operation of the ESP32GTM library
 * 
 * On startup:
 * - scan the BLE airspace for devices advertising goTenna services
 * - connect to first GTM device found
 * - blink lights on connected GTM
 * In the main loop:
 * - read and delete incoming messages
 * - query device info (battery, temperature etc) periodically
 * - if the GTM device disconnects, wait for it and reconnect
 * 
 * Informational messages are printed in the serial console.
 * 
 * https://gitlab.com/almurphy/ESP32GTM
 */

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
