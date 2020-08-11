# ESP32GTM function reference

version 0.1

## Core functions

#### `void gtmInit()`

Setup function, called once from Arduino setup() to initialize BLE scanner and handlers

#### `void gtmLoop()`

Loop function, called from Arduino loop() to manage GTM connection and communication

#### `bool gtmLive()`

Query status of GTM connection

#### `bool gtmExec(uint8_t cmd, uint8_t* data=nullptr, size_t len=0)`

Execute a command on connected GTM and return success status
Parameters `data`, `len` point to the command payload where required

## Alternative loop functions - mutually exclusive with gtmLoop()

#### `void gtmScanLoop()`

Perform BLE scanning and connection attempts
(called from Arduino loop() ONLY when GTM not connected)

#### `void gtmLiveLoop()`
Handle inbound messages and update system information
(called from Arduino loop() ONLY when GTM is connected)

## Events and event handlers

### the InitConfig event

Raised by gtmLoop (gtmScanLoop) after successfully connecting to a GTM device, right before setting the connection "Live" flag to true.
This allows the application to perform any app-specific device initialization (clear memory, set GID etc) before the device going live.

#### `void gtmSetInitConfigFunc(bool (* funcPtr)())`

Set InitConfig event handler to a user-defined function

#### `bool gtmInitConfig()`

The built-in InitConfig event handler

### the Message event

Raised by gtmLoop (gtmLiveLoop) for each incoming message.
Parameters are message buffer pointer and length.

#### `void gtmSetMessageHandler(bool (* funcPtr)(uint8_t *, uint16_t))`

Set Message event handler to a user-defined function

#### `bool gtmParseMessage(uint8_t * buf, uint16_t len)`

The built-in Message event handler

---
Copyright 2020 by Alec Murphy, MIT licensed | https://gitlab.com/almurphy
