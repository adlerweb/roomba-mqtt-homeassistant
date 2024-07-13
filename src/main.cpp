#include "Arduino.h"

#if defined ESP8266 || defined ESP32
  #if defined ESP8266
    #include <ESP8266WiFi.h>
    #include <ESP8266mDNS.h>
  #elif defined ESP32
    #include <WiFi.h>
    #include <ESPmDNS.h>
  #endif  // ESP32
#else
  #error "The board must be ESP8266 or ESP32"
#endif  // ESP

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "PubSubClient.h"
#include "SimpleTimer.h"
#include <ArduinoJson.h>
#include <NTPClient.h>
#include "secrets.h"

#if defined(CONFIG_IDF_TARGET_ESP32C3)
    HardwareSerial RoombaSerial(0);  // Use HardwareSerial on ESP32-C3 for device data
    #define DEBUG_SERIAL Serial      // Use Serial for debugging on ESP32-C3
#else
    #define RoombaSerial Serial      // Use Serial for device data on ESP32 and ESP8266
#endif

#define ROOMBA_WAKEUP D6

const int noSleepPin = 2;

//  Global variables through which to Pass various Sensor struct values needed for battery management & positioning
int chargingStateP;
int currentP;
int voltageP;
int temperatureP;
unsigned int batteryPercentP;
bool sensorsHasDataP;
bool isHomeP = true;

//  Variable for handling position relative to dock/charging station/home and special commands
bool isAtDock = false;
bool isReturning = false;
bool isSpotCleaning = false;

struct Sensors
{
  bool hasData;
  unsigned int bytesRead;
  bool bumpRight;
  bool bumpLeft;
  bool wheelDropRight;
  bool wheelDropLeft;
  unsigned int dirtLevel;
  int chargingState;
  int current;
  int voltage;
  int temperature;
  unsigned int batteryCharge;
  unsigned int batteryCapacity;
  unsigned int batteryPercent;
  bool isExternalCharger;
  bool isHome;
};


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
SimpleTimer timer;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

int getBitFromByte(int byte, int bit)
{
  return (byte >> bit) & 1;
}


// the output range should be -128 to 128
int signed1BytesInt(int byte)
{
  int topBit = getBitFromByte(byte, 7);
  int lowerBits = byte & 127;
  if (topBit == 1) {
  return lowerBits - (1 << 7);
  } else {
  return lowerBits;
}   
}

// the output range should be -32768 to 32767
int signed2BytesInt(int highByte, int lowByte)
 {
   // 1. Take everything except the top bit from the high byte
   int topBit = getBitFromByte(highByte, 7);
   int lowerBits = highByte & 127;
   int unsignedInt = lowerBits << 8 | (lowByte & 0xFF);

  if (topBit == 1) {
    return unsignedInt - (1 << 15);
  } else {
    return unsignedInt;
  }    
 }

 int unsigned2BytesInt(int highByte, int lowByte)
 {
   return highByte << 8 | lowByte;
 }

void wakeUp() 
{
  pinMode(noSleepPin, OUTPUT);
  delay(100);
  digitalWrite(noSleepPin, HIGH);
  delay(100);
  digitalWrite(noSleepPin, LOW);
  delay(100);
  digitalWrite(noSleepPin, HIGH);
  delay(100);
  digitalWrite(noSleepPin, LOW);
  pinMode(LED_BUILTIN, OUTPUT);             //  turn off the built-in LED
  digitalWrite(LED_BUILTIN, OUTPUT);        //  to save a bit of power
  delay(100);
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Wakeup complete");
  #endif
}


void resetRoomba()
{
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Reset");
  #endif
  wakeUp();
  RoombaSerial.write(128);                        //  start IO
  delay(50);
  RoombaSerial.write(131);                        //  safe mode
  delay(50);
  RoombaSerial.write(7);                          //  reset Roomba
  delay(50); 
  RoombaSerial.write(128);                        //  restart IO
}


void unDock() {                             //  pivot backwards then forwards [ un-Dock ]
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Undock");
  #endif
  RoombaSerial.write(131);                        //  Safe mode
  delay(50); 
  RoombaSerial.write(137);                        //  Signal opening of drive sequence (p13 of Roomba OI Spec)
  delay(50);
  RoombaSerial.write(255);                        //  drive backward 2 sec @ 100mm/sec w/ -30 radius (negative radius signifies right-side center)
  delay(50);   
  RoombaSerial.write(100);  
  delay(50); 
  RoombaSerial.write(255);  
  delay(50); 
  RoombaSerial.write(30);  
  delay(2000);
  RoombaSerial.write(137);                        //  Signal opening of drive sequence (p13 of Roomba OI Spec)
  delay(50);
  RoombaSerial.write(0);                          //  drive forward 2 sec @ 100mm/sec w/ 30 radius (positive radius signifies left-side center)
  delay(50);   
  RoombaSerial.write(100);  
  delay(50); 
  RoombaSerial.write(0);  
  delay(50); 
  RoombaSerial.write(30);  
  delay(2000);
}


void clean() {
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Sending 'clean' command");
  #endif
  wakeUp();
  if (isAtDock || isHomeP)                  // if at the Dock or on the Dock, unDock before starting cleaning run
  {
    unDock();
  }
  RoombaSerial.write(128);
  delay(50);
  RoombaSerial.write(135);                        // start cleaning
}



void cleanMax() {
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Sending 'cleanMax' command");
  #endif
  RoombaSerial.write(128);
  delay(50);
  RoombaSerial.write(136);
}



void cleanSpot() {
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Sending 'cleanSpot' command");
  #endif
  RoombaSerial.write(128);
  delay(50);
  RoombaSerial.write(134);
}



void seekDock() {
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Sending 'seekDock' command");
  #endif
  RoombaSerial.write(128);
  delay(50);
  RoombaSerial.write(143);
  isReturning = true;
}



void stop() {                                                         // clarification: opcode 173 has just a one function, to turn off the OI
  #ifdef DEBUG_SERIAL                                                 //                it is not for stopping a cleaning run
    DEBUG_SERIAL.println("Sending 'stop' command");
  #endif
  RoombaSerial.write(128);
  delay(50);
  RoombaSerial.write(173);
}



void powerOff() {
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Sending 'powerOff' command");
  #endif
  RoombaSerial.write(128);
  delay(50);
  RoombaSerial.write(133);
}



void flush() {
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Flushing serial buffer");
  #endif
  int bytes = RoombaSerial.available();
  char buf[bytes];
  if(bytes > 0) {
    RoombaSerial.readBytes(buf, bytes);
  }
}



bool updateTime() 
{
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Updating time with NTP");
  #endif
  timeClient.begin();
  timeClient.setTimeOffset(NTP_TIME_OFFSET);
  bool result = timeClient.forceUpdate();
  if(result) {
    int day = timeClient.getDay();
    int hour = timeClient.getHours();
    int minutes = timeClient.getMinutes();

    RoombaSerial.write(128);
    delay(50);

    RoombaSerial.write(168);
    delay(50);    
    RoombaSerial.write(day);
    delay(50);
    RoombaSerial.write(hour);
    delay(50);
    RoombaSerial.write(minutes);
    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.printf("Updating time success (%d:%d:%d)\n", day, hour, minutes);
    #endif
  }

  return result;
  
}



Sensors updateSensors() {
  Sensors sensors;

  // List of sensors to update
  // 7 - bumps & wheel drops, 1 byte | 0
  // 15 - dirt detect, 1 byte | 8
  // 21 - charging state, 1 byte | 16
  // 22 - Voltage, 2 bytes unsigned | 17-18
  // 23 - Current, 2 bytes unsigned | 19-20
  // 25 - Battery charge, 2 bytes | 22-23
  // 26 - Battery capacity, 2 bytes | 24-25
  
  flush();

  RoombaSerial.write(128);            // opcode start roomba OI
  delay(100);
  RoombaSerial.write(142);            // opcode "get sensor packet"
  delay(100);
  RoombaSerial.write(6);              // opcode "all sensor data"
  delay(100);
  
 
  int i = Serial.available();
  char sensorbytes[100];
  if(i > 0) {
    RoombaSerial.readBytes(sensorbytes, i);
    sensors.hasData = true;
    sensors.bytesRead = i;
    sensors.bumpRight = sensorbytes[0] & 1;
    sensors.bumpLeft = sensorbytes[0] & 2;
    sensors.wheelDropRight = sensorbytes[0] & 4;
    sensors.wheelDropLeft = sensorbytes[0] & 8;

    sensors.dirtLevel = sensorbytes[8];


    sensors.chargingState = sensorbytes[16];
    sensors.isHome = sensors.chargingState > 0 && sensors.chargingState < 4;

    sensors.voltage = unsigned2BytesInt(sensorbytes[17], sensorbytes[18]);
    sensors.current = signed2BytesInt(sensorbytes[19], sensorbytes[20]);
    sensors.temperature = signed1BytesInt(sensorbytes[21]);


    sensors.batteryCharge = unsigned2BytesInt(sensorbytes[22],sensorbytes[23]);
    sensors.batteryCapacity = unsigned2BytesInt(sensorbytes[24], sensorbytes[25]);
    sensors.batteryPercent = 100 * sensors.batteryCharge / sensors.batteryCapacity;
    
    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("sensors update SUCCESS");
    #endif
                                                          // some sensor Struct values that need to be passed for general use
    voltageP = sensors.voltage;
    currentP = sensors.current;
    chargingStateP = sensors.chargingState;
    batteryPercentP = sensors.batteryPercent;
    temperatureP = sensors.temperature;
    isHomeP = sensors.isHome;
    sensorsHasDataP = sensors.hasData;
  } else {
    sensors.hasData = false;
    sensorsHasDataP = sensors.hasData;
    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("sensors update FAILED");
    #endif                    // this is the normal/expected behavior/output when the roomba is not awake
  }

  return sensors;

}



String getState(Sensors sensors) {

  if(sensors.isHome) {
    return "Docked";                                        // Actually charging at the moment
  }

  else if(isAtDock) {
    return "Docked";                                        // Handle case where it was charging, but has been backed away for BMS reasons
  }

  else if(isReturning) {
    return "Returning to dock";                             // Roomba was sent "return_to_home" command
  }

  else if(sensors.current < MIN_CLEANING_CURRENT) {         // Determine "Cleaning" state solely by power draw, not command
    return "Cleaning";
  }

  else { 
    return "Idle";
  }
}



String getFanSpeed(String state) {

  return state == "cleaning" ? "max" : "off";
}



void publishSensorsInformation() {
  if(!mqttClient.connected() || WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  Sensors sensors = updateSensors();
  if(sensors.hasData) {

    JsonDocument doc;
    
    String state = getState(sensors);
    doc["state"] = state;
    doc["fan_speed"] = getFanSpeed(state);
    doc["battery_level"] = sensors.batteryPercent;

    doc["bump_left"] = sensors.bumpLeft;
    doc["bump_right"] = sensors.bumpRight;

    doc["wheel_drop_left"] = sensors.wheelDropLeft;
    doc["wheel_drop_right"] = sensors.wheelDropRight;

    doc["dirt_level"] = sensors.dirtLevel;

    doc["voltage"] = sensors.voltage;

    doc["battery_charge"] = sensors.batteryCharge;
    doc["battery_capacity"] = sensors.batteryCapacity;

    doc["charging_state"] = sensors.chargingState;
    doc["current"] = sensors.current;

    doc["temperature"] = sensors.temperature;

    doc["bytes"] = sensors.bytesRead;

    String str;
    serializeJson(doc, str);
    mqttClient.publish(MQTT_STATE_TOPIC, str.c_str());
    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Sensors info published to MQTT");
        DEBUG_SERIAL.println(str);
    #endif
  }
}



void publishHomeAssistantAutoDiscovery(String uniqueId, String name, String valueTemplate, String deviceClass, String unitOfMeasurement, String topicType, bool isVacuum)
{
  JsonDocument doc;
  String fullName;
  String fullId;
  if(name == "") {
    fullName =  String(MQTT_CLIENT_NAME);
  } else {
    fullName = String(MQTT_CLIENT_NAME) + " " + name;
  }

  if(uniqueId == "") {
    fullId =  String(HASS_UNIQUE_ID);
  } else {
    fullId = String(HASS_UNIQUE_ID) + "_" + uniqueId;
  }

  doc["name"] = fullName;
  doc["unique_id"] = fullId;
  doc["state_topic"] = MQTT_STATE_TOPIC;
  doc["availability_topic"] = MQTT_AVAILABILITY_TOPIC;

  if(valueTemplate != "") {
    doc["value_template"] = "{{ value_json."+ valueTemplate + " }}";
  }

  if(deviceClass != "") {
    doc["device_class"] = deviceClass;
  }

  if(unitOfMeasurement != "") {
    doc["unit_of_measurement"] = unitOfMeasurement;
  }

  JsonObject device = doc["device"].to<JsonObject>();

  if(isVacuum) {
    device["manufacturer"] = HASS_MANUFACTURER;
    device["model"] = HASS_MODEL;
    device["name"] = HASS_NAME;
    device["sw_version"] = HASS_VERSION;

    JsonArray connections = device["connections"].to<JsonArray>();
    JsonArray connection = connections.add<JsonArray>();
    connection.add("mac");
    connection.add(WiFi.macAddress());

  } else {
    device["via_device"] = HASS_UNIQUE_ID;
  }
  
  JsonArray identifiers = device["identifiers"].to<JsonArray>();
  identifiers.add(HASS_UNIQUE_ID);

  String str;
  serializeJson(doc, str);
  String topic = "homeassistant/" + topicType + "/" + fullId + "/config";
  mqttClient.publish(topic.c_str(), str.c_str(), true);
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Auto discovery info published to MQTT");
    DEBUG_SERIAL.println(str);
  #endif
}



void onMQTTMessage(char* topic, byte* payload, unsigned int length) 
{
  String newTopic = topic;
  payload[length] = '\0';
  String command = String((char *)payload);
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.printf("MQTT received command: %s\n", command.c_str());
  #endif
  //  There is an inconsistency in the naming and actions of the commands.  
  //  On the HA side, "stop" sent through the MQTT has an intended meaning... probably to stop a cleaning run
  //  On the roomba side of things, the "stop opcode" is a 173 serial command, which stops the OI
  //  Since these are HA Commands, it has to be the powerOff command, which actually doesn't power it off, it just stops a cleaning run
  //
  if (newTopic == MQTT_COMMAND_TOPIC) {
    wakeUp();
    isReturning = false;

    // Official Home Assistant commands
    if (command == "start" || command == "pause" || command == "start_pause" || command == "turn_on") {
      clean();
    }
    else if (command == "clean_spot") {
      cleanSpot();
    }
    else if (command == "return_to_base") {
      seekDock();
      isReturning = true;                                         //  set status to indicate returning to dock
    }
    else if (command == "stop" || command == "turn_off") {        //  this effectively stops any cleaning run; it doesn't power down or 
      powerOff();                                                 //  shutting off the OI
    }
  }
}



void connectMQTT() 
{
  while (!mqttClient.connected()) 
  {
     // Attempt to connect
      if (mqttClient.connect(MQTT_CLIENT_NAME, MQTT_USER, MQTT_PASSWORD, MQTT_AVAILABILITY_TOPIC, 0, true, "offline")) 
      {
        // Serial.printf("MQTT connection successful, subscribing to command topic: %s\n", MQTT_COMMAND_TOPIC);
        mqttClient.subscribe(MQTT_COMMAND_TOPIC);
        mqttClient.publish(MQTT_AVAILABILITY_TOPIC, "online", true);
        publishHomeAssistantAutoDiscovery("", "", "", "", "", "vacuum", true);
        publishHomeAssistantAutoDiscovery("state", "State", "state", "", "", "sensor", false);
        publishHomeAssistantAutoDiscovery("battery", "Battery", "battery_level", "battery", "%", "sensor", false);
        publishHomeAssistantAutoDiscovery("voltage", "Voltage", "voltage / 1000", "battery", "V", "sensor", false);
        publishHomeAssistantAutoDiscovery("temperature", "Temperature", "temperature", "temperature", "°C", "sensor", false);
        publishHomeAssistantAutoDiscovery("dirt_level", "Dirt Level", "dirt_level / 255 * 100 | round(0)", "temperature", "%", "sensor", false);
      } 
      else 
      {
        delay(MQTT_RECONNECT_DELAY);
      }
  }
}



void connectWifi() 
{
    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println();
        DEBUG_SERIAL.print("Connecting to ");
        DEBUG_SERIAL.print(WIFI_SSID);
        DEBUG_SERIAL.print(" as ");
        DEBUG_SERIAL.print(WIFI_CLIENT_NAME);
    #endif
  
  WiFi.hostname(WIFI_CLIENT_NAME);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) 
  {
    #ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print(".");
    #endif
    delay(WIFI_RECONNECT_DELAY);
  }
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println(" - OK!");
  #endif
}



void startOTA()
{
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Starting OTA");
  #endif
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(OTA_HOST_NAME);
  ArduinoOTA.begin();
}



void startMQTT()
{
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Starting MQTT");
  #endif
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(onMQTTMessage);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  connectMQTT();
}

void setup() {
  Serial.begin(115200);

  // Set WiFi to station mode and disconnect from an AP if it was previously connected.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Start scan");
  #endif

  // WiFi.scanNetworks will return the number of networks found.
  #ifdef DEBUG_SERIAL
    int n = WiFi.scanNetworks();
    if (n == 0) {
        DEBUG_SERIAL.println("no networks found");
    } else {
        DEBUG_SERIAL.print(n);
        DEBUG_SERIAL.println(" networks found");
        DEBUG_SERIAL.println("Nr | SSID                             | RSSI | CH | Encryption");
        for (int i = 0; i < n; ++i) {
        // Print SSID and RSSI for each network found
        DEBUG_SERIAL.printf("%2d", i + 1);
        DEBUG_SERIAL.print(" | ");
        DEBUG_SERIAL.printf("%-32.32s", WiFi.SSID(i).c_str());
        DEBUG_SERIAL.print(" | ");
        DEBUG_SERIAL.printf("%4ld", WiFi.RSSI(i));
        DEBUG_SERIAL.print(" | ");
        DEBUG_SERIAL.printf("%2ld", WiFi.channel(i));
        DEBUG_SERIAL.print(" | ");
        switch (WiFi.encryptionType(i)) {
          case WIFI_AUTH_OPEN:            DEBUG_SERIAL.print("open"); break;
          case WIFI_AUTH_WEP:             DEBUG_SERIAL.print("WEP"); break;
          case WIFI_AUTH_WPA_PSK:         DEBUG_SERIAL.print("WPA"); break;
          case WIFI_AUTH_WPA2_PSK:        DEBUG_SERIAL.print("WPA2"); break;
          case WIFI_AUTH_WPA_WPA2_PSK:    DEBUG_SERIAL.print("WPA+WPA2"); break;
          case WIFI_AUTH_WPA2_ENTERPRISE: DEBUG_SERIAL.print("WPA2-EAP"); break;
          case WIFI_AUTH_WPA3_PSK:        DEBUG_SERIAL.print("WPA3"); break;
          case WIFI_AUTH_WPA2_WPA3_PSK:   DEBUG_SERIAL.print("WPA2+WPA3"); break;
          case WIFI_AUTH_WAPI_PSK:        DEBUG_SERIAL.print("WAPI"); break;
          default:                        DEBUG_SERIAL.print("unknown");
        }
        DEBUG_SERIAL.println();
        delay(10);
        }
    }
    DEBUG_SERIAL.println("");
  #endif

  // Delete the scan result to free memory for code below.
  WiFi.scanDelete();

  // Wait a bit before scanning again.
  delay(1000);
  
  connectWifi();
  startOTA();
  startMQTT();
  updateTime();
  timer.setInterval(ROOMBA_WAKEUP_INTERVAL, wakeUp);
  timer.setInterval(MQTT_PUBLISH_INTERVAL, publishSensorsInformation);
}

void loop() {
  // Reconnect if connection has been lost
  if(!mqttClient.connected()) {
    connectMQTT();
  }

  ArduinoOTA.handle();
  timer.run();
  mqttClient.loop();
}