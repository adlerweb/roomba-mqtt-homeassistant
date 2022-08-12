# roomba-mqtt-homeassistant

Smart Home integration some older pre-wifi Roomba robot vacuums (600,700,800 series). Tested and working on a Roomba 780.

## Features
 - HA/Lovelace control and status that actually works
 - MQTT support
 - Home Assistant Auto Discovery support
 - OTA 
 - NTP time sync
 - Power-savings for longer battery life
 - A basic software-based Battery Management System for the problems that some older roombas (eg. the 780), have when fitted with a Li-Ion battery.  This can help work around the "Err5 battery error wit 5 blinks".
 
## Hardware
 - Any ESP8266 board with sufficient exposed GPIO pins.  The WeMos D1 Mini clones are great because of the USB-serial converter and reset button.  A bigger board will be rather difficult to put inside the vacuum, and the older ESP-01 may not have the exposed pins needed for some of the debugging.
 - mini-DIN-7 or -8 pin connector to connect to SCI port, or a custom-made solution
 - A buck converter, logic level shifter or voltage divider; I used an MP1584EN 
 - 2N3906 TO-92 PNP (B331) Transistor to filter signals from Roomba, or equivalent
 - A micro-usb cable to connect to the USB-Serial port on the D1 Mini, or a serial cable solution for your use case.
 - Some wiring & connectors: an assortment of Dupont jumpers and headers work well
 - A soldering iron & solder

## Knowledge
 - Soldering small electronics
 

 
