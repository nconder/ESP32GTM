/*
 * goTenna Mesh Bluetooth-LE interface library for Arduino/ESP32
 * Copyright (c) 2020 by Alec Murphy. All rights reserved. MIT Licensed
 * 
 * https://gitlab.com/almurphy
 * 
 * Technical information and data from https://github.com/sybip/pygt
 */
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "Arduino.h"

#include "BLEDevice.h"
#include "esp_log.h"

#include "ESP32GTM.h"

// logging tag
static const char* TAG = "GTM";

// The GTM remote service we wish to connect to.
static BLEUUID serviceUUID("1276aaee-df5e-11e6-bf01-fe55135034f3");

// The characteristics of the GTM remote service:
//   Status (ST), Receive (RX) and Transmit (TX)
static BLEUUID charaSTUUID("12762b18-df5e-11e6-bf01-fe55135034f3");
static BLEUUID charaRXUUID("1276b20b-df5e-11e6-bf01-fe55135034f3");
static BLEUUID charaTXUUID("1276b20a-df5e-11e6-bf01-fe55135034f3");

static BLERemoteCharacteristic * pRemCharaST, * pRemCharaRX, * pRemCharaTX;

static BLEAdvertisedDevice * myDevice;

// MTU of the BLE connection - currently for informative purposes only
static uint16_t BLE_MTU = 23;

// GTM state variables
static uint8_t gtmSeqByte = 0;      // Sequence number for commands
static uint32_t gtmLastCmdAt = 0;   // time (millis) of last command
static uint8_t gtmReadErrors = 0;   // consecutive READMSG error counter, reset on success
static uint8_t gtmState = STATE_SCANNING; // State machine state

// leave these non-static for now
uint16_t gtmBattLevel = 0;    // battery level in 1/10000V (from SYSINFO)
uint16_t gtmUnreadMsg = 0;    // number of unread messages (from SYSINFO)
char gtmSerialNum[11] = { 0 };  // device serial number "MX99999999"
bool connGTM = false;       // true if GTM is connected

// probably public
bool gtmMWIFlag = false;    // message waiting indicator

// Maximum packet size for TX fragmentation
// conservatively based on the minimum MTU value of 23
// (should really be a function of the actual, negotiated MTU)
static uint8_t BLE_TXSIZE = 20;

// Maximum RX and TX buffer sizes - based on maximum TLV size of 256 + margin
#define BLE_BUFFER_SIZE 320
// Data buffers
// Because data is received asynchronously, we prefer separate buffers for RX and TX
static uint8_t txBuf[BLE_BUFFER_SIZE] = {0};
static uint16_t txBufPos = 0;
static uint16_t txBufCRC = 0;

static uint8_t rxBuf[BLE_BUFFER_SIZE] = {0};
static uint16_t rxBufPos = 0;
static uint16_t rxBufCRC = 0;
static bool rxBufRdy = false;  // indicate buffer complete and ready for processing


/*
 * USER FUNCTIONS AND CALLBACKS
 */
// Called from active loop whenever a message is received
bool (* messageHandler)(uint8_t *, uint16_t) = gtmParseMessage;

// Called from connection handler when a GTM device is connected
bool (* initConfigFunc)(void) = gtmInitConfig;


/*
 * BLE Library callbacks
 */
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    GTM_LOGD(TAG, "onConnect()");
  }

  void onDisconnect(BLEClient* pclient) {
    // https://github.com/nkolban/esp32-snippets/issues/498#issuecomment-384614251
    pclient->disconnect();

    // Update state machine
    if (gtmState == STATE_ONLINE) {
      // Return to state CONNECT and try to resume connection
      GTM_LOGD(TAG, "onDisconnect() state ONLINE -> CONNECT");
      gtmState = STATE_CONNECT;  // Try reconnecting
    } else {
      GTM_LOGD(TAG, "onDisconnect() no state change");
      // Don't change state
    }
    connGTM = false;  // Can't use GTM right now
  }
};


/* Scan for BLE servers and find the first one that advertises the service we are looking for.*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /* Called for each advertising BLE server.*/
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    GTM_LOGD(TAG, "BLE Advertised Device:\n  %s", advertisedDevice.toString().c_str());

    // We have found a device, see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      gtmState = STATE_CONNECT;
    }
  }
};


/* Handles the data received via the ST characteristic */
static void gtmNotifyCallbackST(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                           uint8_t* pData, size_t length, bool isNotify) 
{
    GTM_LOGD(TAG, "Notify-ST, len=%d, data[0]=0x%02x", length, pData[0]);
    if (pData[0] >0) {
      gtmMWIFlag = true;
    }
    return;
}


/* Handles the data received via the RX characteristic */
static void gtmNotifyCallbackRX(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                           uint8_t* pData, size_t length, bool isNotify) 
{
    GTM_LOGD(TAG, "Notify-RX, len=%d", length);
    GTM_LOGV(TAG, "BLE.RECV:");
    ESP_LOG_BUFFER_HEXDUMP(TAG, pData, length,  ESP_LOG_VERBOSE);

    uint8_t head = 0;
    uint8_t tail = 0;

    // 10 02 is the STX word; initialize our buffer
    if ((pData[0] == 0x10) &&
        (pData[1] == 0x02)) {
      rxBufPos = 0;
      rxBufCRC = 0;
      head = 2;
    }

    // 10 03 is the ETX word
    if ((pData[length-2] == 0x10) &&
        (pData[length-1] == 0x03)) {
      tail = 2;  // skip ETX
    }

    // skip STX (head) and ETX (tail) if present
    for (int ix=head; ix<length-tail; ix++) {
      rxBuf[rxBufPos++] = pData[ix];

      // 0x10 is a reserved character
      if ((pData[ix] == 0x10) && (pData[ix+1] == 0x10)) {
        // unescape by skipping the second 0x10
        ix++;
      }
    }

    // ETX present means packet complete and ready to process
    if (tail) {
      // rxBuf contains UNESCAPED data and 2 bytes CRC16X

      // Calculate CRC for data in rxBuf
      for (int ix=0; ix<rxBufPos-2; ix++) {
        rxBufCRC = CRC16X_add(rxBuf[ix], rxBufCRC);
      }
          
      // Check calculated CRC against expected value
      if (rxBufCRC != ((rxBuf[rxBufPos-2] << 8) + rxBuf[rxBufPos-1])) {
          GTM_LOGW(TAG, "CRC ERROR (CALC=%04x)", rxBufCRC);
          return;
      }

    } else {
      // NOT TAIL -> more data chunks to come
      return;
    }

    // Roll back 2 bytes to strip CRC
    rxBufPos -= 2;

    GTM_LOGV(TAG, "DATA RECV:");
    ESP_LOG_BUFFER_HEXDUMP(TAG, rxBuf, rxBufPos, ESP_LOG_VERBOSE);

    // indicate buffer is ready
    rxBufRdy = true;
}


/* CRC16-XMODEM - used in BLE PDUs and message objects */
uint16_t CRC16X_add(uint8_t b, uint16_t crc)
{
  crc ^= ((uint16_t)b << 8);

  for (int ix = 8; ix; ix--) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;  // (polynomial = 0x1021)
    else
      crc <<= 1;
  }
  return crc;
}


/* Proprietary goTenna 16-bit hash based on Park-Miller LCG */
uint16_t gtmAlgoH16(uint8_t * buf, size_t len)
{
  // Park-Miller LCG parameters
  #define LCGMULT 48271;
  #define LCGSEED 0xaa;
  #define LCGMOD ((1<<31)-1);

  uint32_t h = 0;
  uint64_t x = LCGSEED;
  for (int ix=0; ix<len; ix++) {
    x = (((x + buf[ix]) * 48271 + 1) & 0xffffffff);
    x %= LCGMOD;
    h ^= x;
  }
  return (((h & 0xffff0000) >> 16) ^ (h & 0xffff));
}


// Go low-level here, as the BLE lib doesn't support writing to non-UUID handles
// (we need to define our own function bleWriteToHandle() )
#include <esp_gattc_api.h>
#include <esp_err.h>

/* Write data to a handle
 *  (custom function, similar to writeValue, but using 
 *   handle number instead of UUID to designate target)
 */
bool bleWriteToHandle(BLEClient* pclient, uint16_t handle, uint8_t* data, size_t length, bool response)
{
  esp_err_t errRc = esp_ble_gattc_write_char(
                pclient->getGattcIf(),
                pclient->getConnId(),
                handle,
                length,
                data,
                response?ESP_GATT_WRITE_TYPE_RSP:ESP_GATT_WRITE_TYPE_NO_RSP,
                ESP_GATT_AUTH_REQ_NONE);

  if (errRc != ESP_OK) {
    GTM_LOGE(TAG, "esp_ble_gattc_write_char failed, rc=%d", errRc);
    return false;
  }
  return true;
}


/* Set user callback for incoming messages */
void gtmSetMessageHandler(bool (* funcPtr)(uint8_t *, uint16_t)) {
  messageHandler = funcPtr;
}

/* Set user callback for initializing device */
void gtmSetInitConfigFunc(bool (* funcPtr)()) {
  initConfigFunc = funcPtr;
}


/*  Append byte to BLE TX buffer, calculating CRC in parallel
 * escaping of special character 0x10 also performed here
 */
void gtmTxBufAppend(uint8_t oneByte)
{
    txBuf[txBufPos++] = oneByte;
    txBufCRC = CRC16X_add(oneByte, txBufCRC);

    // escape 0x10 (but do NOT update CRC)
    if (oneByte == 0x10)
      txBuf[txBufPos++] = oneByte;
}


/* Send a command to GTM, no wait for reply 
 *  cmd - command opcode
 *  data- (optional) pointer to command payload
 *  len - length of command payload
 * The payload is copied non-destructively into the TX buffer
 */
void gtmSendCMD(uint8_t cmd, uint8_t * data, size_t len)
{
  // Start with STX word
  txBuf[0] = 0x10;
  txBuf[1] = 0x02;
  txBufPos = 2;
  txBufCRC = 0;

  // Append command byte
  gtmTxBufAppend(cmd);

  // Append sequence byte
  gtmTxBufAppend(gtmSeqByte++);
  gtmSeqByte &= 0xff;

  // Append payload, byte by byte
  for (int ix = 0; ix < len; ix++) {
    gtmTxBufAppend(data[ix]);
  }

  // Append CRC
  txBuf[txBufPos++] = (0xff & (txBufCRC>>8));
  txBuf[txBufPos++] = (0xff & txBufCRC);

  // Append ETX word
  txBuf[txBufPos++] = 0x10;
  txBuf[txBufPos++] = 0x03;
  
  GTM_LOGV(TAG, "DATA.SEND:");
  ESP_LOG_BUFFER_HEXDUMP(TAG, txBuf, txBufPos, ESP_LOG_VERBOSE);

  // Send buffer to TX handle
  // PACKETIZE TX buffer to BLE_TXSIZE
  for (int ix=0; ix<txBufPos; ix+=BLE_TXSIZE) {
    GTM_LOGV(TAG, "BLE.SEND:");
    if (ix+BLE_TXSIZE<=txBufPos) {
      // full packet
      ESP_LOG_BUFFER_HEXDUMP(TAG, txBuf+ix, BLE_TXSIZE, ESP_LOG_VERBOSE);
      pRemCharaTX->writeValue(txBuf+ix, BLE_TXSIZE, false);
    } else {
      // last packet incomplete
      ESP_LOG_BUFFER_HEXDUMP(TAG, txBuf+ix, txBufPos % BLE_TXSIZE, ESP_LOG_VERBOSE);
      pRemCharaTX->writeValue(txBuf+ix, (txBufPos % BLE_TXSIZE), false);
    }
  }
}


/* Send a command to GTM, wait for reply and process error code */
bool gtmExec(uint8_t cmd, uint8_t * data, size_t len)
{
  rxBufPos = 0;
  rxBufCRC = 0;
  rxBufRdy = false;

  GTM_LOGD(TAG, "EXEC CMD 0x%02x with data len=%d", cmd, len);
  gtmSendCMD(cmd, data, len);

  // Reception happens in an interrupt, so from our perspective
  // we just sit and wait for the data to "show up" (rxBufRdy)
  for (int ix=0; ix<200; ix++) {
    if (rxBufRdy) {
      GTM_LOGV(TAG, "GOT DATA, len=%d", rxBufPos);
      break;
    }
    delay(10);
  }

  // rxBufPos is really the length of data in buffer
  if (rxBufPos == 0) {
    GTM_LOGW(TAG, "NO RESPONSE");
    return false;
  }

  // TODO: check sequence number!
  if ((rxBuf[0] ^ cmd) == GT_OP_SUCCESS) {
    GTM_LOGD(TAG, "EXEC SUCCESS");
    gtmLastCmdAt = millis();  // Record time of last successful cmd
    return true;
  } else {
    GTM_LOGW(TAG, "CMD 0x%02x EXEC ERROR: 0x%02x", cmd, (rxBuf[0] ^ cmd));
    return false;
  }
}


/* Get connected device info: battery level, unread msgs etc */
bool gtmGetSysInfo()
{
  if (gtmExec(OP_SYSINFO)) {
    gtmParseSysInfo(rxBuf, rxBufPos);
    return true;
  }
  GTM_LOGW(TAG, "GETINFO FAILED");
  return false;
}


/* Get GTM connection status */
bool gtmLive()
{
  return connGTM;
}


/* Simple placeholder for the user initialization function */
bool gtmInitConfig()
{
  GTM_LOGV(TAG, "Using built-in initializer");

  // Make it blinken
  gtmExec(OP_BLINKEN, (uint8_t *) "X", 1);
  delay(100);
}


/* Connect to the BLE server (GTM device) identified by var "myDevice"
 * Called from gtmLoop whenever state reaches "CONNECT"
 * - at startup, when the GTM device is first detected via scanning
 * - during runtime, following a disconnect event
 */
bool gtmConnect() {
    GTM_LOGI(TAG, "BLE connecting to: %s", myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    GTM_LOGD(TAG, " - Created client, connecting...");

    // Connect to the remote BLE Server
    // !!! FIXME !!! - sometimes connect() hangs forever requiring manual reset
    // - maybe related to https://github.com/nkolban/esp32-snippets/issues/880
    // - possibly fixed by updating BLEClient.cpp

    pClient->connect(myDevice);  
    GTM_LOGD(TAG, " - Connected to server");

    pClient->setClientCallbacks(new MyClientCallback());

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      GTM_LOGW(TAG, "Failed to find our service UUID: %s", serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    GTM_LOGD(TAG, " - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemCharaST = pRemoteService->getCharacteristic(charaSTUUID);
    if (! pRemCharaST) {
      GTM_LOGW(TAG, "Failed to find ST UUID: %s", charaSTUUID.toString().c_str());
      pClient->disconnect();
      return false;
    } else {
      GTM_LOGD(TAG, "%s", pRemCharaST->toString().c_str());
    }

    pRemCharaRX = pRemoteService->getCharacteristic(charaRXUUID);
    if (! pRemCharaRX) {
      GTM_LOGW(TAG, "Failed to find RX UUID: %s", charaRXUUID.toString().c_str());
      pClient->disconnect();
      return false;
    } else {
      GTM_LOGD(TAG, "%s", pRemCharaRX->toString().c_str());
    }

    pRemCharaTX = pRemoteService->getCharacteristic(charaTXUUID);
    if (! pRemCharaTX) {
      GTM_LOGW(TAG, "Failed to find TX UUID: %s", charaTXUUID.toString().c_str());
      pClient->disconnect();
      return false;
    } else {
      GTM_LOGD(TAG, "%s", pRemCharaTX->toString().c_str());
    }

    GTM_LOGD(TAG, "Detected ST_HND=%02x, RX_HND=%02x, TX HND=%02x", 
             pRemCharaST->getHandle(), 
             pRemCharaRX->getHandle(), 
             pRemCharaTX->getHandle());

    BLE_MTU = pClient->getMTU();
    GTM_LOGI(TAG, "BLE MTU: %d", BLE_MTU);

    // Delay codes are necessary during init to prevent crashing
    //  due to going too fast
    delay(200);

    // Register notification callbacks - do this before sending magic bytes
    GTM_LOGD(TAG, "Registering ST callback");
    pRemCharaST->registerForNotify(gtmNotifyCallbackST);
    delay(100);

    GTM_LOGD(TAG, "Registering RX callback");
    pRemCharaRX->registerForNotify(gtmNotifyCallbackRX);
    delay(100);

    // Send magic bytes - do this after registering callbacks
    uint16_t tmpHnd = pRemCharaRX->getHandle()+1;
    uint8_t tmpCmd[] = {0x02, 0x00};
    GTM_LOGD(TAG, "Activate RX: %02x", tmpHnd);
    bleWriteToHandle(pClient, tmpHnd, tmpCmd, 2, true);

    tmpHnd = pRemCharaST->getHandle()+1;
    tmpCmd[0] = 0x01;
    GTM_LOGD(TAG, "Activate ST: %02x", tmpHnd);
    bleWriteToHandle(pClient, tmpHnd, tmpCmd, 2, true);

    delay(100);

    // Get info: battery level, unread msgs etc
    gtmGetSysInfo();
    delay(200); 

    // Call user initialization routine (to set GIDs etc)
    if ((* initConfigFunc) != nullptr) {
      if (initConfigFunc()) {
        GTM_LOGD(TAG, "initConfigFunc() complete");
      } else {
        GTM_LOGW(TAG, "initConfigFunc() FAILED");
      }
    }

    connGTM = true;
    return true;
}


/* Called from main loop if GTM device is present */
void gtmLiveLoop()
{
    // Keep-alive - send SYSINFO to update state if idle for 30 sec
    if (millis() - gtmLastCmdAt > 30000) {
      gtmGetSysInfo();
    }

    while (gtmMWIFlag && connGTM) {  // Message waiting, try a read/next sequence
      GTM_LOGD(TAG, "MWI is SET");

      // Read one message from GTM inbox
      if (gtmExec(OP_READMSG)) {
        gtmReadErrors = 0;  // reset error counter
        GTM_LOGI(TAG, "INCOMING MESSAGE, LEN=%d", rxBufPos);
        // Call processing function if defined
        if ((* messageHandler) != nullptr) {
          if (messageHandler(rxBuf, rxBufPos)) {
            GTM_LOGD(TAG, "Message processed OK");
          } else {
            GTM_LOGD(TAG, "Message processing FAILED");
          }
        }

        // Delete and advance to next message in GTM inbox
        if (gtmExec(OP_NEXTMSG)) {
          // Returns a 16-bit value indicating "unread messages left"
          if ((rxBuf[2]<<8) + rxBuf[3]) {
            GTM_LOGD(TAG, "MORE messages (%d)", ((rxBuf[2]<<8) + rxBuf[3]));
          } else {
            GTM_LOGD(TAG, "NO MORE messages, resetting MWI");
            gtmMWIFlag = 0;  // No more messages
            // Refresh system info
            gtmGetSysInfo();
          }
        } else {
          GTM_LOGW(TAG, "NEXTMSG failed");
          delay(100);   // breathe
        }
      } else {
        GTM_LOGW(TAG, "READMSG failed");
        // Sometimes, on startup, my GTM turns MWI on but there are no unread messages
        // This may cause a loop where we try and fail to read a message that doesn't exist
        // To work around, check the "unread msgs" field from SYSINFO for a second
        //   opinion, and if zero, reset MWI indicator.
        gtmGetSysInfo();
        if (gtmUnreadMsg == 0) {
          GTM_LOGD(TAG, "Resetting FALSE MWI");
          gtmMWIFlag = 0;
        }
        gtmReadErrors ++;
        if (gtmReadErrors > 5) {
          GTM_LOGW(TAG, "Too many READMSG errors, probably FALSE MWI");          
          gtmMWIFlag = 0;
          gtmReadErrors = 0;  // reset error counter
        }
        delay(100);  // breathe
      }
      delay(10);
    } 
}


/* Called from main loop if GTM device is present */
void gtmScanLoop()
{
  // use a simple state machine to track BLE connection:
  //  0 - SCANNING (looking for a BLE device with GTM service UUID)
  //  1 - CONNECT (found a BLE device, now trying to connect to it)
  //  2 - ONLINE  (connected to a goTenna Mesh device)
  switch(gtmState) {
    case STATE_CONNECT:
      if (gtmConnect()) {
        GTM_LOGI(TAG, "GTM CONNECTED; state = ONLINE");
        gtmState = STATE_ONLINE;
      } else {
        GTM_LOGW(TAG, "FAILED TO CONNECT; state = SCANNING");
        gtmState = STATE_SCANNING;
      }
      break;

    case STATE_SCANNING:
      BLEDevice::getScan()->start(5, false);  // true = do NOT reset list of detected devices
      break;

    case STATE_ONLINE:
    default:
      break;
  }  
}


/* This is the library's initialization function
 * Called from Arduino setup() on startup
 */
void gtmInit()
{
  BLEDevice::init("");
  // BLEDevice::setMTU(23);

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 10 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false);
  // Set state
  gtmState = STATE_SCANNING;
}


/* This is the library's main recurrent activity
 * Called from Arduino loop()
 */
void gtmLoop()
{
  if (connGTM) {
      gtmLiveLoop();
  } else {
      gtmScanLoop();
  }
}


/* Parse info block from command 04 "get system info" */
bool gtmParseSysInfo(uint8_t * buf, uint16_t len)
{
  if (len != 30) {  // 28 bytes info block + T + L
    GTM_LOGW(TAG, "Unexpected infoblock length: %d", (len-2));
    return false;
  }
  gtmBattLevel = (buf[4]<<8) + buf[5];
  gtmUnreadMsg = (buf[2]<<8) + buf[3];
  memcpy(gtmSerialNum, buf+15, 10);

  GTM_LOGI(TAG, "DEVICE INFO: S/N=%s, FW=%d.%d.%d, Msgs=%d, Batt=%d, Temp=%d", 
           gtmSerialNum, buf[9], buf[10], buf[11], gtmUnreadMsg, gtmBattLevel, buf[13]);

  // Excessively verbose output
  GTM_LOGV(TAG, "GTM DEVICE INFO:");
  GTM_LOGV(TAG, "Unread:  %d", gtmUnreadMsg);
  GTM_LOGV(TAG, "Battery: %d", gtmBattLevel);
  GTM_LOGV(TAG, "UNK01:   %d", buf[6]);
  GTM_LOGV(TAG, "RadTemp: %d", buf[7]);
  GTM_LOGV(TAG, "UNK02:   %d", buf[8]);
  GTM_LOGV(TAG, "FW Ver:  %d.%d.%d", buf[9], buf[10], buf[11]);
  GTM_LOGV(TAG, "UNK03:   %d", buf[12]);
  GTM_LOGV(TAG, "SysTemp: %d", buf[13]);
  GTM_LOGV(TAG, "UNK04:   %d", buf[14]);
  GTM_LOGV(TAG, "Serial:  %s", gtmSerialNum);
  GTM_LOGV(TAG, "On batt: %d", buf[25]);
  GTM_LOGV(TAG, "HW Ver:  %d", buf[26]);
  GTM_LOGV(TAG, "LED ON:  %d", buf[27]);
  GTM_LOGV(TAG, "BootVer: %d", (buf[28]<<8) + buf[29]);
  return true;
}


/* Simple, application-neutral built-in message parser
 * Only outer layer fields are currently supported
 */
bool gtmParseMessage(uint8_t * buf, uint16_t len)
{
  uint16_t pos=2;   // skip command and sequence 
  uint8_t msgClass = 0;
  uint16_t msgAppID = 0;
  uint32_t msgTStamp = 0;
  // GID fields (From/Dest) are 6 bytes each
  uint32_t msgFromHi = 0, msgFromLo = 0;
  uint32_t msgDestHi = 0, msgDestLo = 0;
  uint8_t msgBlobPos = 0;
  uint8_t msgBlobLen = 0;

  GTM_LOGV(TAG, "Using built-in message parser");

  // Message objects have a TLV structure
  // Traverse this structure and parse the main elements
  while(pos < len) {
    GTM_LOGD(TAG, "TLV TYPE=0x%02x, LEN=%d", buf[pos], buf[pos+1]);
    switch(buf[pos]) {
      case MESG_TLV_DEST:
        msgClass = buf[pos+2];
        msgAppID = (buf[pos+3]<<8) + buf[pos+4];

        // Two types of TLV_DEST - short (3 bytes) and long (10 bytes)
        if (buf[pos+1] == 10) {
          // long TLV_DEST, Class-App-GID-Tag
          msgDestHi = (buf[pos+5]<<8) + buf[pos+6];
          msgDestLo = (buf[pos+7]<<24) + (buf[pos+8]<<16) + (buf[pos+9]<<8) + buf[pos+10];
          GTM_LOGD(TAG, " [DEST] Class=%d, AppID=0x%04x, Dest=0x%04x%08x, Suffix=%0x02x", 
                    msgClass, msgAppID, msgDestHi, msgDestLo, buf[pos+11]);
        } else {
          // short TLV_DEST, Class-App
          GTM_LOGD(TAG, " [DEST] Class=%d, AppID=0x%04x", msgClass, msgAppID);
        }
        break;

      case MESG_TLV_HOPS:
        GTM_LOGD(TAG, " [HOPS] Hops=%d, RSSI=%d", buf[pos+2], buf[pos+3]);
        break;

      case MESG_TLV_DATA:
        // Sanity check on length
        if (buf[pos+1] < 16) {
          GTM_LOGW(TAG, " [DATA] INVALID LENGTH");
          break;
        }

        // Expecting DATA value to begin with 0xFB 0x10; anything else confuses us
        if ((buf[pos+2] != 0xfb) || (buf[pos+3] != 0x10)) {
          GTM_LOGD(TAG, " [DATA] UNKNOWN FORMAT");
          break;  // bail out early
        }

        // from pos+4 start parsing FB section
        // "BQLHB", cryptFlag, fromGID, tStamp, seqNo0, seqNo1
        // process sender field in 32-bit chunks to avoid Arduino headache
        msgFromHi = (buf[pos+5]<<24) + (buf[pos+6]<<16) + (buf[pos+7]<<8) + buf[pos+8];
        msgFromLo = (buf[pos+9]<<24) + (buf[pos+10]<<16) + (buf[pos+11]<<8) + buf[pos+12];
        // msgFrom = (msgFromHi << 32) | msgFromLo;

        // 32-bit unix timestamp
        msgTStamp = (buf[pos+13]<<24) + (buf[pos+14]<<16) + (buf[pos+15]<<8) + buf[pos+16];

        // The FB section of the DATA element has a standard format
        //  however, the rest of the element is an application specific blob
        // Last 2 bytes of the blob may SOMETIMES contain a CRC16X
        msgBlobLen = buf[pos+1] - 18;  // Len from TLV minus sizeof FB header

        GTM_LOGD(TAG, " [DATA] Crypt=%d, From=0x%04x%08x, TStamp=%d, Len=%d", 
                  buf[pos+4], msgFromHi, msgFromLo, msgTStamp, msgBlobLen);

        // End processing, proprietary formats ahead
        break;

      default:
        // we don't need to support every single possible element type
        GTM_LOGD(TAG, " [OTHER]");
        break;
    }

    // Finished processing element, advance read position
    pos += buf[pos+1] + 2;  // Len + T/L bytes
  }
  return true;
}
