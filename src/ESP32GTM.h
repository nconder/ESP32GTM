/*
 * goTenna Mesh Bluetooth-LE interface library for Arduino/ESP32
 * Copyright (c) 2020 by Alec Murphy. All rights reserved. MIT Licensed
 * 
 * https://gitlab.com/almurphy
 */
#ifndef ESP32GTM_H
#define ESP32GTM_H
/* 
 * GTM command opcodes
 * from https://github.com/sybip
 * renamed OP_FLASH to OP_BLINKEN because FLASH is an EPROM type :)
 */
#define OP_FLASH    0x00
#define OP_BLINKEN  0x00    // Blinken the light 3 times to help locate
#define OP_SET_GID  0x01    // Set GID on device
#define OP_SET_PUB  0x02    // Upload public key (don't know why)
#define OP_SENDMSG  0x03    // Send a message (shout, direct, key exch etc)
#define OP_SYSINFO  0x04    // Get system info (serial, version, battery etc)
#define OP_READMSG  0x06    // Read the first message in receive queue
#define OP_NEXTMSG  0x07    // Delete the first message from queue
#define OP_RST_GID  0x0b    // Reset GID
#define OP_DEL_GID  0x0d    // Delete GID?
#define OP_SET_APP  0x10    // Set App ID
#define OP_SET_GEO  0x21    // Set geopolitical region (1=US)
#define OP_GET_GEO  0x22    // Get geopolitical region

#define GT_OP_SUCCESS 0x40  // Command successful

/*
 * Message class IDs
 * from https://github.com/sybip
 */
#define MSG_CLASS_P2P    0     // Regular peer-to-peer message
#define MSG_CLASS_GROUP  1     // Group message
#define MSG_CLASS_SHOUT  2     // Shout message
#define MSG_CLASS_EMERG  3     // Emergency message

/*
 * Envelope-level TLV types in message objects
 *  (R) = present in rcvd msgs (air-to-app), (T) in sent msgs (app-to-air)
 * MANDATORY TLVs for app-generated msgs are DEST, DATA and TTL 
 * from https://github.com/sybip
 */
#define MESG_TLV_0x04  0x04    // (R,T) Undoc, required in some cases
#define MESG_TLV_DATA  0x05    // (R,T) Main section: sender, body, payloads
#define MESG_TLV_DEST  0x06    // (R,T) Message class and dest (GID or all)
#define MESG_TLV_HOPS  0x20    // (R,A) Rx'd msg hops count
#define MESG_TLV_DLR   0x21    // (R) Delivery report (created by device)
#define MESG_TLV_TTL   0x22    // (T) Message TTL (set by sender app)
#define MESG_TLV_MXRX  0x23    // (air only) Mesh metadata RX (stripped by device)
#define MESG_TLV_MXTX  0x24    // (air only) Mesh metadata TX (appended by device)

enum GTM_STATE {
  STATE_SCANNING,
  STATE_CONNECT,
  STATE_ONLINE
};

// One of the imported libraries overwrites the original ESP_LOGx macros
// For predictability, we define our own logging macros
#define GTM_LOGE( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   tag, format, ##__VA_ARGS__)
#define GTM_LOGW( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    tag, format, ##__VA_ARGS__)
#define GTM_LOGI( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    tag, format, ##__VA_ARGS__)
#define GTM_LOGD( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   tag, format, ##__VA_ARGS__)
#define GTM_LOGV( tag, format, ... ) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__)

uint16_t CRC16X_add(uint8_t b, uint16_t crc = 0);

// The main library functions are three: init, loop and exec
void gtmInit();
void gtmLoop();
bool gtmLive();
bool gtmExec(uint8_t cmd, uint8_t* data=nullptr, size_t len=0);

// Alternative loop functions - mutually exclusive with gtmLoop()
void gtmScanLoop();
void gtmLiveLoop();

// Functions to set event handlers
void gtmSetInitConfigFunc(bool (* funcPtr)());
void gtmSetMessageHandler(bool (* funcPtr)(uint8_t *, uint16_t));

// Built-in event handlers
bool gtmInitConfig();
bool gtmParseSysInfo(uint8_t * buf, uint16_t len);
bool gtmParseMessage(uint8_t * buf, uint16_t len);

#endif  // ESP32GTM_H
