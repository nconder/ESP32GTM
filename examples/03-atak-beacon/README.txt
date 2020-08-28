ESP32GTM library example 03 - ATAK CoT beacon
=============================================

Use the goTenna Mesh to periodically broadcast a compressed CoT object,
compatible with the goTenna-Consumer plugin for ATAK as described here:
  https://old.reddit.com/r/ATAK/comments/i6y8n5/a/g17hfk7/

Notes
-----

This is a slightly more complex example, as it requires knowledge of
some goTenna API formats and structures, and a basic understanding
of the CoT (cursor-on-target) protocol concepts.

In order to focus on the message construction, envelope and broadcast, 
some code sections that would be required in a production application
have been intentionally omitted from this example. Specifically:

1) No GTM device initialization - this has been covered in another
example, and would have been redundant. To ensure that your GTM is
correctly initialized (for the purpose of this example), pair it with 
the goTenna app or the ATAK plugin, then turn off Bluetooth on the phone.
This will leave the GTM in an initialized state and ready to accept a new 
connection.

2) No GPS functionality - the GPS code is hardware-dependent and would
have added out-of-scope complexity. The GPS location variables (lat, lon, 
hae and fix) have been declared and populated with dummy values.

3) No clock setting. Opportunistic clock adjustment using incoming message
timestamps has been covered separately. In production, GPS should be used
as a time source for clock adjustment.

4) No encryption - while the goTenna plugin supports optional AES 
encryption with a pre-shared key, the implementation exceeds the scope of
this example. As a result, the broadcasts will be received and displayed
by all GTM-equipped ATAK devices in range.

5) No persistence - all identity related parameters are generated at
runtime using the lower bytes of the MAC address for uniqueness. This is 
poor security practice that should be avoided in production.

With these limitations understood and accepted, the code will compile and 
run out of the box. It will connect to the first available GTM device and 
will use it to broadcast a (stationary) CoT point at 60 second intervals.
The callsign will be automatically set to ESP32GTMXXXX, where XXXX is a 
4-digit HEX number.
Messages will be broadcast unencrypted, carried by the goTenna Mesh network
over a maximum of 3 hops and displayed by default on all receiving devices.

More information
----------------

- GTM message format: https://github.com/sybip/pyGT/wiki/Messages
- ATAK plugin format: https://old.reddit.com/r/ATAK/comments/i6y8n5/a/g17hfk7/
