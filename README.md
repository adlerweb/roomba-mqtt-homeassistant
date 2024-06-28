# roomba-mqtt-homeassistant

Smart Home integration some older pre-wifi Roomba robot vacuums (600,700,800 series). Uses an ESP8266 or ESP32 attached to the Roomba Service Port (SCI). Tested and working on a Roomba 605 and 780.

## Features
 - HA/Lovelace control and status that actually works
 - MQTT support
 - Home Assistant Auto Discovery support
 - OTA 
 - NTP time sync
 - Power-savings for longer battery life
 - A basic software-based Battery Management System for the problems that some older roombas (eg. the 780), have when fitted with a Li-Ion battery.  This can help work around the "Err5 battery error with 5 blinks".
 
## Hardware
 - Any ESP8266/ESP32 board with sufficient exposed GPIO pins. Bigger boards will be rather difficult to put inside the vacuum, older ones like ESP-01 may not have the exposed pins needed for some of the debugging.
 - mini-DIN-7 or -8 pin connector to connect to SCI port, or a custom-made solution
 - A buck converter, logic level shifter or voltage divider; Tested with MP1584EN and YF08E
 - PHP Transistoe (for example 2N3906) to filter signals from Roomba
 - A USB cable or programmer, depending on the board you are using
 - Some wiring & connectors: an assortment of Dupont jumpers and headers work well
 - A soldering iron & solder

## Knowledge
 - Soldering small electronics
