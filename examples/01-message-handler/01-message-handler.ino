/*
 * ESP32GTM library example 01 - simple Message event handler
 * 
 * Use a Message event handler to extract the timestamp from
 *  incoming goTenna messages; then use this timestamp as a 
 *  free time source to set the system clock.
 * (inaccurate by many minutes, but better than 01/01/1970)
 * 
 * The timezone for all displayed date/times is UTC
 * 
 * https://gitlab.com/almurphy/ESP32GTM
 */

#include "ESP32GTM.h"

// Time functions
#include <TimeLib.h>

// Timestamp extracted from message
time_t timeFromMsg = 0;


/* Our "time extractor" function that will run as a Message handler */
bool timeExtractor(uint8_t * buf, uint16_t len) {
  uint16_t pos = 2;  // skip command and sequence

  // traverse TLV structure
  while(pos < len) {
    // we only want the DATA element, ignore the rest
    if (buf[pos] == MESG_TLV_DATA) {

      // sanity check on length and header
      if ((buf[pos+1] < 16) || (buf[pos+2] != 0xfb) || (buf[pos+3] != 0x10)) {
        Serial.println("Unknown format for DATA element, skipping");
        break;
      }

      // read 32-bit timestamp from its known position within DATA element
      //   directly, without parsing the rest of the element
      // (https://github.com/sybip/pyGT/wiki/Messages#message-header-element)
      timeFromMsg = (buf[pos+13]<<24) + (buf[pos+14]<<16) + (buf[pos+15]<<8) + buf[pos+16];

      // display system clock and message time for comparison
      Serial.print("System clock: ");
      printDateTime(now());
      Serial.println();

      Serial.print("Message time: ");
      printDateTime(timeFromMsg);
      Serial.println();

      break;  // we got what we came for
    }

    // advance read position to next TLV element
    pos += buf[pos+1] + 2;
  }

  return true;
}


/* helper function to print datetime in human format */
void printDateTime(time_t t)
{
  Serial.printf("%04d-%02d-%02d %02d:%02d:%02d", 
          year(t), month(t), day(t), 
          hour(t), minute(t), second(t));
}


void setup() {
  // Enable serial port and introduce ourselves
  Serial.begin(115200);
  Serial.println("ESP32GTM EXAMPLE 01 - simple Message event handler");

  // avoid flooding the console
  esp_log_level_set("*", ESP_LOG_INFO);

  // set message handler to our function
  gtmSetMessageHandler(timeExtractor);

  // Run this once
  gtmInit();
}


void loop() {
  // Run this on every loop
  gtmLoop();

  // After each GTM loop, check time status, and if:
  //  - system clock is unset (year <2020) and 
  //  - we have captured a valid timestamp,
  //  then use the timestamp to set system clock
  if ((year() < 2020) && (year(timeFromMsg) >= 2020)) {
    Serial.printf("SETTING SYSTEM CLOCK (%d -> %d)\n", now(), timeFromMsg);
    setTime(timeFromMsg);

    // humanize it
    Serial.print("System clock: ");
    printDateTime(now());
    Serial.println();
  }

  // Delay a second between loops
  delay(1000);
}
