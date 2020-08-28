# ESP32GTM

goTenna Mesh Bluetooth-LE library for Arduino/ESP32

## project purpose

To open the goTenna Mesh (GTM) platform to a wider range of applications, which were not possible using the existing smartphone-oriented apps and SDKs.

## applications

- low-cost low-power beacons, telemetry and mesh-of-things
- industrial remote control and automation
- cross-protocol connectors (to LoRa, WiFi, third-party apps etc)
- other headless SWaP-sensitive applications
- secure and open source software development

## features and capabilities

### core functionality:
- detects, connects and initializes goTenna Mesh device
- BLE MTU packetization and reassembly
- sends commands (auto-add SEQ, CRC and STX/ETX) which the GTM accepts
- receives command responses via RX notify, with CRC verification
- receives the message waiting (MWI) flag via ST notify

### built-in extras
- SYSINFO command and parse sysinfo block
- READMSG/NEXTMSG command logic

### demo features also included
- simple TLV processor for message objects
- parse message elements: HEAD, HOPS
- parse 16-byte header of DATA element

## documentation
See REFERENCE-0.1.md and examples directory.

## requirements

My development environment is listed below. Please treat this list as a guideline and safety net only, rather than a hard requirement.

### software
- Arduino 1.8.10
- ESP32 Arduino 1.0.2

### hardware
- Espressif ESP32-WROOM-32
- goTenna Mesh v1.1.8

### extras
- Android phone v6.0 with working Bluetooth LE, no SIM or WiFi
- Android goTenna app v5.0.2
- Android FakeGPS app v2.0.8 (goTenna app requires a GPS fix)

### optional
- activated in-app goTenna Plus free trial (for location tethering)
- Android ATAK-CIV app 3.8.1.1 with GotennaConsumer-5-civ plugin

## status
*itWorksForMe*, however this is early stage code, only tested so far on my systems. There be bugs.

- some hanging / crash issues at startup
- generally stable in operation, with uptimes over 24 hours while continuously sending and receiving several messages per minute

## thanks and acknowledgements
- https://github.com/sybip/pyGT has been an important source of knowledge and information
- https://github.com/nkolban/ESP32_BLE_Arduino for taking the pain out of Bluetooth LE

---
Copyright 2020 by Alec Murphy, MIT licensed | https://gitlab.com/almurphy
