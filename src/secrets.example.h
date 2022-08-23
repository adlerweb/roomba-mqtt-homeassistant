// WiFi Settings
const char* WIFI_CLIENT_NAME = "Roomba-780";
const char* WIFI_SSID = "My-IoT-Wifi-24GHz";
const char* WIFI_PASSWORD = "MyPassword1234!@#$";
const int WIFI_RECONNECT_DELAY = 3000;

// MQTT Connection Settings
const char* MQTT_HOST = "192.168.100.123";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "Roomba-780";
const char *MQTT_PASSWORD = "Roomba-780";
const char *MQTT_CLIENT_NAME = "Roomba-780";

// MQTT Topic Settings
const char *MQTT_DISCOVERY_TOPIC = "homeassistant/vacuum/Roomba-780/config";
const char *MQTT_STATE_TOPIC = "vacuum/state";
const char *MQTT_COMMAND_TOPIC = "vacuum/command";
const char *MQTT_AVAILABILITY_TOPIC = "vacuum/status";
const int MQTT_PUBLISH_INTERVAL = 60000;              // (milliseconds): Publish sensors & status to MQTT broker this often 
const int MQTT_RECONNECT_DELAY = 3000;
const int MQTT_BUFFER_SIZE = 1024;

// Vacuum Settings
const int ROOMBA_WAKEUP_INTERVAL = 3600000;           // (milliseconds): 3600000 = 1 hour; Roomba will wake up this often
const int MIN_CLEANING_CURRENT = -300;                // (milliAmperes): Roomba 780 uses at least 350-400 mA when just moving, up to 1200 while brushes and fan are running

// Battery Management Settings for Li-Ion battery issues
const int VOLTAGE_HIGH_THRESHOLD = 16300;             // If voltage reads over this, then charging is probably not working (probably per battery - will vary)
const int MIN_IDLE_BATTERY_LEVEL = 95;                // (%) low battery % limit; If not charging && not running && lower than this, time to recharge
const int MIN_CEASE_CHARGING_BATTERY_LEVEL = 99;      // (%) high battery % limit; If charging && current = 0 && Voltage > VHT and battery is already here, then stop charging
const int BATTERY_CHECK_INTERVAL = 60000;             // (milliseconds) Check battery once a minute when not sleeping (most importantly, check while charging)

// OTA Settings
const int OTA_PORT = 8266;
const char* OTA_HOST_NAME = "roomba-780";         

// Home Assistant Settings
const char* HASS_UNIQUE_ID = "roomba-780";
const char* HASS_NAME = "Roomba-Upstairs";
const char* HASS_MANUFACTURER = "iRobot";
const char* HASS_MODEL = "Roomba 780";
const char* HASS_VERSION = "1.0.0-dev.0";

//NTP Settings
const char* NTP_SERVER = "pool.ntp.org";
const int NTP_UPDATE_INTERVAL = 24 * 3600 * 1000;
const int NTP_TIME_OFFSET = 3600 * 3;
