# roomba-mqtt-homeassistant

Smart Home integration some older pre-wifi Roomba robot vacuums (600,700,800 series).

## Hardware
 - Any ESP8266 board with sufficient exposed GPIO pins.  The WeMos D1 Mini clones are great because of the USB-serial converter and reset button.  A bigger board will be rather difficult to put inside the vacuum, and the older ESP-01 may not have the exposed pins needed for some of the debugging.
 - mini-DIN-7 or -8 pin connector to connect to SCI port, or a custom-made solution
 - A buck converter, logic level shifter or voltage divider
 - A micro-usb cable to connect to the USB-Serial port on the D1 Mini, or a serial cable solution for your use case.
 - Some wiring
 - A soldering iron & solder

## Knowledge
 - Soldering small electronics
 
## Features
 - MQTT support
 - Home Assistant Auto Discovery support
 - OTA 
 - NTP time sync
 - Power-saving to provide longer battery life
