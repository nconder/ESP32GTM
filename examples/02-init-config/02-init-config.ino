/*
 * ESP32GTM library example 02 - simple InitConfig event handler
 *
 * Hook the initConfig event to set a new GID on device init, in 
 *   addition to the existing GID(s) which will also be preserved
 * Use a boolean flag (firstTime) to ensure that our function is 
 *   only applied on the first encounter with the device, but not 
 *   on subsequent reconnect events.
 *
 * https://gitlab.com/almurphy/ESP32GTM
 */

#include "ESP32GTM.h"

bool firstTime = true;  // first time initializing the device


// our custom initialization function, will run as an event handler
bool mySettings()
{
  // data buffer for SET_GID command
  uint8_t tGID[10] = {
    0x00,                               // Addr type (0=private, 1=group)
    0x3f, 0xff,                         // App ID (0x3fff is goTenna App)
    0x55, 0x00, 0x00, 0x00, 0x00, 0x00, // 6-byte GID (dflt 0x550000000000)
    0x00};                              // GID suffix (group member ID etc)

  Serial.println("ENTERING initConfig function");

  // initialization may be destructive - causing an inbox wipe etc
  //  don't re-do if we're REconnecting to an already init'ed device

  if (not firstTime) {
    Serial.println("Welcome back, goTenna. No initialization necessary");
    return true;
  }

  // First time initialization
  Serial.println("Initializing goTenna");

  // use lowest bytes from ESP MAC to personalize the GID
  //  for demo purposes only, never do this in production
  uint64_t chipID = ESP.getEfuseMac();

  tGID[7] = (chipID >> 32) & 0xff;
  tGID[8] = (chipID >> 40) & 0xff;

  // set App ID to ATAK (0xd8ff)
  tGID[1] = 0xd8;
  tGID[2] = 0xff;

  // print out GID and announce what happens next
  Serial.printf("Setting GID: %02x %04x %04x%08x %02x\n",
                  tGID[0],
                  (tGID[1]<<8) + tGID[2],
                  (tGID[3]<<8) + tGID[4],
                  (tGID[5]<<24) + (tGID[6]<<16) + (tGID[7]<<8) + tGID[8],
                  tGID[9]);

  // finally execute SETGID request
  gtmExec(OP_SET_GID, tGID, 10);

  // blinken lights
  Serial.println("Blink blink blink");
  gtmExec(OP_BLINKEN, (uint8_t *) "x", 1);

  firstTime = false;  // next time won't be first time anymore
  return true;
}


void setup() {
  // Enable serial port and introduce ourselves
  Serial.begin(115200);
  Serial.println("ESP32GTM EXAMPLE 02 - simple InitConfig event handler");

  // avoid flooding the console
  esp_log_level_set("*", ESP_LOG_INFO);

  // set InitConfig handler to our function
  gtmSetInitConfigFunc(mySettings);

  // Run this once
  gtmInit();
}

void loop() {
  // Run this on every loop
  gtmLoop();

  // Delay a second between loops
  delay(1000);
}
