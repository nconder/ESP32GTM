/*
 * ESP32GTM library example 03 - ATAK CoT beacon
 *
 * Use the goTenna Mesh to periodically broadcast a compressed CoT object,
 *  compatible with the goTenna-Consumer plugin for ATAK
 *
 * For illustration purposes only. See README.txt for important info
 *
 * Part of ESP32GTM project, MIT licensed
 * https://gitlab.com/almurphy/ESP32GTM
 */

#include <TimeLib.h>
#include "ESP32GTM.h"

#define BCAST_INTERVAL 60  // seconds

// CoT identity variables
char selfUUID[40] = { 0 };      // will be set dynamically in setup()
char callSign[40] = { 0 };      // will be set dynamically in setup()
char unitType[] = "a-f-G-U-C";  // friendly-ground-unit-combat
char unitTeam[] = "Red";        // Oh YES

// CoT position variables, populated with some dummy data
// (in production, these should be updated from the GPS receiver!)
uint64_t gpsLAT = 51948900;    // microdeg
uint64_t gpsLON = 4053500;     // microdeg
uint32_t gpsHAE = 1000;        // millimeters
bool gpsFix = true;

// time (millis) of last broadcast
static uint64_t lastBcast = 0;


// Create a Shout class Message object, with the supplied buffer as payload
//   (TLV elements will be: DEST, DATA, TTL)
// Default AppID is 0xd8ff, for compatibility with ATAK goTenna plugin
//
// Formats and structures from: https://github.com/sybip/pyGT/wiki/Messages
//
bool gtmBroadcast(char * blobPtr, uint8_t blobLen, uint16_t appID=0xd8ff, uint8_t ttl=3)
{
  // temp staging buffer for assembling message object
  uint8_t outBuf[256];
  uint8_t outBufPos = 0;

  // CRC16X of payload blob
  uint16_t blobCRC16x = 0;

  // BAD news:  we need a timestamp for message envelope
  // GOOD news: ATAK plugin ignores the message envelope timestamp, so 
  //   we can use anything here and the conversion will still work
  // (in production, your clock should be GPS synced anyway!)
  uint32_t time32 = now();

  // DEST element - short version: 06 03 02 d8 ff
  outBuf[outBufPos++] = MESG_TLV_DEST;    // type = 6 (DEST)
  outBuf[outBufPos++] = 0x03;   // len = 3
  outBuf[outBufPos++] = MSG_CLASS_SHOUT;  // class = 2 (SHOUT)
  outBuf[outBufPos++] = (appID >> 8) & 0xff;  // App ID MSB
  outBuf[outBufPos++] = appID & 0xff;         // App ID LSB
  // DEST element complete

  // DATA element - starting with 16-byte head
  outBuf[outBufPos++] = MESG_TLV_DATA;   // type = 5 (DATA)
  // DATA element length (function of blob size and encryption)
  //   for cleartext: blobLen + head + CRC
  outBuf[outBufPos++] = 20 + blobLen;

  // 16-byte header starts here
  // https://github.com/sybip/pyGT/wiki/Messages#message-header-element
  outBuf[outBufPos++] = 0xfb;   // header
  outBuf[outBufPos++] = 0x10;   // len = 16 always
  // "BQLHB", cryptFlag, fromGID, tStamp, seqNo0, seqNo1
  outBuf[outBufPos++] = 0;      // not encrypted
  outBuf[outBufPos++] = 0;      // rsvd
  outBuf[outBufPos++] = 0;      // rsvd

  // Sender ID - unused in our case, fill with random data
  for (int ix = 0; ix<6; ix++) {
    outBuf[outBufPos++] = random(256);
  }

  // 32-bit timestamp
  outBuf[outBufPos++] = (time32 >> 24) & 0xff;
  outBuf[outBufPos++] = (time32 >> 16) & 0xff;
  outBuf[outBufPos++] = (time32 >> 8) & 0xff;
  outBuf[outBufPos++] = time32 & 0xff;

  // Sequence numbers - set to 1 if unused
  outBuf[outBufPos++] = 0;  // seq0
  outBuf[outBufPos++] = 1;  // seq0
  outBuf[outBufPos++] = 1;  // seq1

  // Content BLOB goes here
  // https://github.com/sybip/pyGT/wiki/Messages#message-content-element

  // Append buffer as cleartext
  for (int ix=0; ix<blobLen; ix++) {
    outBuf[outBufPos++] = blobPtr[ix];
    blobCRC16x = CRC16X_add(blobPtr[ix], blobCRC16x);
  }
  // Append CRC16X
  outBuf[outBufPos++] = (blobCRC16x >> 8) & 0xff;
  outBuf[outBufPos++] = blobCRC16x & 0xff;
  // DATA element complete

  // TTL element
  outBuf[outBufPos++] = MESG_TLV_TTL;   // type=22 (TTL)
  outBuf[outBufPos++] = 1;      // len=1
  outBuf[outBufPos++] = ttl;    // val
  // TTL element complete

  return (gtmExec(OP_SENDMSG, outBuf, outBufPos));
}


void setup() {
  // Enable serial port and introduce ourselves
  Serial.begin(115200);
  Serial.println("ESP32GTM EXAMPLE 03 - ATAK CoT beacon");

  // avoid flooding the console
  esp_log_level_set("*", ESP_LOG_INFO);

  // Set identity variables: UUID and callsign
  // we want these to be deterministic yet reasonably unique, 
  //   so we'll use the MAC LSBs again
  // for demo purposes only, never do this in production
  // (in production you should use non-volatile settings)
  uint64_t chipID = ESP.getEfuseMac();

  // Deterministic UUID: ESP32GTM-XXXXXXXX
  sprintf(selfUUID, "ESP32GTM-a1ec%04x", (chipID >> 32) & 0xffff);

  // Deterministic callsign: ESP32GTMXXXX
  sprintf(callSign, "ESP32GTM%04x", (chipID >> 32) & 0xffff);

  // Run this once
  gtmInit();
}


void loop() {
  // temporary message buffer
  char gtmCoT[200] = { 0 };

  // send on configured broadcast interval, and only if goTenna is connected
  if ((millis() - lastBcast) >= (1000 * BCAST_INTERVAL)) {

    if (gtmLive()) {
      // Build a compressed CoT message in ATAK-goTenna plugin format
      snprintf(gtmCoT, 200, "%s;%s;%s;%s;%.06f;%.06f;%.03f;%s;%d",
               selfUUID, unitType, callSign, 
               gpsFix ? "m-g" : ((gpsLON || gpsLAT) ? "h-e" : "h-g-i-g-o"),
               gpsLAT/1000000., gpsLON/1000000., gpsHAE/1000.,
               unitTeam, BCAST_INTERVAL);

      // Display, then broadcast the compressed CoT message
      Serial.printf("SENDING: %s\n", gtmCoT);

      if (! gtmBroadcast(gtmCoT, strlen(gtmCoT))) {
        Serial.println("SEND FAILED!");
      } else {
        Serial.println("SENT OK");
      }

      // Reset timer
      lastBcast = millis();
    }
  }

  // Run this on every loop
  gtmLoop();

  // Delay a second between loops
  delay(1000);
}
