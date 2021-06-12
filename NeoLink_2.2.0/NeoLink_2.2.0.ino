// Compatiblee con Peripheral v1.0






//------------- OTA ------------------------------------------------
//#include <WebServer.h>
//#include <ESPmDNS.h>
//#include <Update.h>
//------------- OTA HTTPS ------------------------------------------------

//#include <HTTPClient.h>
#include "cJSON.h"
#include <WiFiClientSecure.h>
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include <HTTPUpdate.h>


//------------- I2C ------------------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// Serials----------------------------------------------------------
#include <SoftwareSerial.h>
//-----------------------------Firebase & Servers-------------------------------------
#include <WiFi.h>
#include "FirebaseESP32.h"
#include "FirebaseJson.h"
#include "neoFirebaseJson.h"
#include <Arduino_JSON.h>
#include <HTTPClient.h>
//------------------------ TIME NTP SERVER ----------------------------
#include "time.h"
//------------------------ Heltec ----------------------------
#include "heltec.h"
//-----------------------------------OWN---------------------------------
#include <beep.h>
#include <EEPROM.h>

//--------- INTERNAL TEMP ----------------
#include <OneWire.h>
#include <DallasTemperature.h>
//--------- Lora librarie-----------------




//_____________________________________________________________________//
//                                                                      //
//                  NEOLINK                                       //                                                                      //
//______________________________________________________________________//

const String DEVICE_TYPE = "NeoLink";
const String DEVICE_HEADER = "NL";
String SN = "XX0000-0000";
const String FIRMWARE_VER = "2.0.0";
//const double BUILT = 552;
const char* HARDWARE_VER = "2.0";

#define FIRMWARE_MODE 'DEV2.2'

#if FIRMWARE_MODE == 'PRO'
  #define FIREBASE_HOST "https://neolink-934b4.firebaseio.com"
  #define FIREBASE_AUTH "IroB3fdbcPb9vxPlJKDJcqmfJgs0KouJGe0sUBKN"
  #define UPDATE_JSON_URL  "https://firmware-neolink.s3-sa-east-1.amazonaws.com/firmware_pro.json"
  const String WIFI_SSID_DEFAULT = "NeoLink_wlan"; //modem default
  const String WIFI_PSSWD_DEFAULT = "123456789a";

 #elif FIRMWARE_MODE == 'DEV'
  #define FIREBASE_HOST "https://neolink-b2f81-default-rtdb.firebaseio.com"
  #define FIREBASE_AUTH "P2aDr6F6P1XZQ3zc7k4ABuPBT9o5szLwFHphsqZt"
  #define UPDATE_JSON_URL  "https://test-firmware-neolink.s3.us-east-2.amazonaws.com/firmware_dev.json"
  const String WIFI_SSID_DEFAULT = "LINUX1";
  const String WIFI_PSSWD_DEFAULT = "123456789abc";  

#elif FIRMWARE_MODE == 'DEV2.2'
  #define FIREBASE_HOST "https://neolink-934b4.firebaseio.com"
  #define FIREBASE_AUTH "IroB3fdbcPb9vxPlJKDJcqmfJgs0KouJGe0sUBKN"
  #define UPDATE_JSON_URL  "https://firmware-neolink.s3-sa-east-1.amazonaws.com/firmware_pro.json"
  const String WIFI_SSID_DEFAULT = "LINUX1";
  const String WIFI_PSSWD_DEFAULT = "123456789abc";
  byte localAddress = 0xBB; //address of this device.
  byte destination = 0x01; //destination to send to.


#elif FIRMWARE_MODE == 'DEV2.1'
  #define FIREBASE_HOST "https://neolink-934b4.firebaseio.com"
  #define FIREBASE_AUTH "IroB3fdbcPb9vxPlJKDJcqmfJgs0KouJGe0sUBKN"
  #define UPDATE_JSON_URL  "https://firmware-neolink.s3-sa-east-1.amazonaws.com/firmware_pro.json"
  const String WIFI_SSID_DEFAULT = "LINUX1";
  const String WIFI_PSSWD_DEFAULT = "123456789abc";
  byte localAddress = 0xBB; //address of this device.
  byte destination = 0x01; //destination to send to.

#endif



//------------- OTA HTTPS  ---------------------------------------------

const char* ca = \
                 "-----BEGIN CERTIFICATE-----\n" \
                 "MIIEizCCA3OgAwIBAgIQDI7gyQ1qiRWIBAYe4kH5rzANBgkqhkiG9w0BAQsFADBh\n" \
                 "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
                 "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
                 "MjAeFw0xMzA4MDExMjAwMDBaFw0yODA4MDExMjAwMDBaMEQxCzAJBgNVBAYTAlVT\n" \
                 "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxHjAcBgNVBAMTFURpZ2lDZXJ0IEdsb2Jh\n" \
                 "bCBDQSBHMjCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBANNIfL7zBYZd\n" \
                 "W9UvhU5L4IatFaxhz1uvPmoKR/uadpFgC4przc/cV35gmAvkVNlW7SHMArZagV+X\n" \
                 "au4CLyMnuG3UsOcGAngLH1ypmTb+u6wbBfpXzYEQQGfWMItYNdSWYb7QjHqXnxr5\n" \
                 "IuYUL6nG6AEfq/gmD6yOTSwyOR2Bm40cZbIc22GoiS9g5+vCShjEbyrpEJIJ7RfR\n" \
                 "ACvmfe8EiRROM6GyD5eHn7OgzS+8LOy4g2gxPR/VSpAQGQuBldYpdlH5NnbQtwl6\n" \
                 "OErXb4y/E3w57bqukPyV93t4CTZedJMeJfD/1K2uaGvG/w/VNfFVbkhJ+Pi474j4\n" \
                 "8V4Rd6rfArMCAwEAAaOCAVowggFWMBIGA1UdEwEB/wQIMAYBAf8CAQAwDgYDVR0P\n" \
                 "AQH/BAQDAgGGMDQGCCsGAQUFBwEBBCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29j\n" \
                 "c3AuZGlnaWNlcnQuY29tMHsGA1UdHwR0MHIwN6A1oDOGMWh0dHA6Ly9jcmw0LmRp\n" \
                 "Z2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RHMi5jcmwwN6A1oDOGMWh0dHA6\n" \
                 "Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RHMi5jcmwwPQYD\n" \
                 "VR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEWHGh0dHBzOi8vd3d3LmRpZ2lj\n" \
                 "ZXJ0LmNvbS9DUFMwHQYDVR0OBBYEFCRuKy3QapJRUSVpAaqaR6aJ50AgMB8GA1Ud\n" \
                 "IwQYMBaAFE4iVCAYlebjbuYP+vq5Eu0GF485MA0GCSqGSIb3DQEBCwUAA4IBAQAL\n" \
                 "OYSR+ZfrqoGvhOlaOJL84mxZvzbIRacxAxHhBsCsMsdaVSnaT0AC9aHesO3ewPj2\n" \
                 "dZ12uYf+QYB6z13jAMZbAuabeGLJ3LhimnftiQjXS8X9Q9ViIyfEBFltcT8jW+rZ\n" \
                 "8uckJ2/0lYDblizkVIvP6hnZf1WZUXoOLRg9eFhSvGNoVwvdRLNXSmDmyHBwW4co\n" \
                 "atc7TlJFGa8kBpJIERqLrqwYElesA8u49L3KJg6nwd3jM+/AVTANlVlOnAM2BvjA\n" \
                 "jxSZnE0qnsHhfTuvcqdFuhOWKU4Z0BqYBvQ3lBetoxi6PrABDJXWKTUgNX31EGDk\n" \
                 "92hiHuwZ4STyhxGs6QiA\n" \
                 "-----END CERTIFICATE-----\n" ;

// receive buffer
char rcv_buffer[200];

//------------- I2C ------------------------------------------------

#define SEALEVELPRESSURE_HPA 1015.85
Adafruit_BME280 bme_static;
Adafruit_BME280 bme_aux;



//--------------------------PINOUT-----------------------------
#define BAT_SOLAR_EN 23
#define BEEP 15
const byte esp32_RX___ard_TX = 16;
const byte esp32_TX___ard_RX = 17;
#define ARDUINO_RESTART 13
#define ATMOS_EN 12
#define TEMP_EN 25
#define RTC_IO 33
#define TOUCH_T9 32
#define AUX_IN 39
#define TEMP_VALUE 33
#define SOLAR_VALUE 37
#define BAT_VALUE 36

#define SIM_ON 4
#define FORCED_ATMEGA_RESET 2


// Serials----------------------------------------------------------

//to arduino
SoftwareSerial ArdSerial(esp32_RX___ard_TX, esp32_TX___ard_RX);

//-----------------------------Firebase & Servers-------------------------------------



const String openWeatherMapApiKey = "4720c89eaac409313d52d4434ed2844d";
String jsonWeatherBuffer;
const String WEATHER_SERVER_PATH_1 = "http://api.openweathermap.org/data/2.5/onecall?";
const String WEATHER_SERVER_PATH_2 = "&exclude=hourly,daily&appid=4720c89eaac409313d52d4434ed2844d";




//------------------------------------ PATHS -------------------------
String DEVICE;
const String PATH_CONFIGURATION_VALUES =  "/Conf_values/";
const String PATH_CONFIGURATION_STATUS = "/State/";
const String PATH_DATA = "/DataSet/";



//------------------------ TIME NTP SERVER ----------------------------

//const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec =  -18000;
const int   daylightOffset_sec = 0;


//-----------------------------------OWN---------------------------------

Beep beep(BEEP);

//------------------------------- INTERNAL ----------------------------------------

OneWire oneWire(TEMP_VALUE);

DallasTemperature sensors(&oneWire);


//------------------------------- Heltec ----------------------------------------




#define BAND    433E6


//Deep sleep ---------------------------------------

#define uS_TO_S_FACTOR 1000000



//TIME VARIABLES -------------------------------------
#define POWERLESS_TIME 600
#define WIFI_TIME_LIMIT 6000
#define UNREGISTERED_TIME 120

//Firebase  ---------------------------------------
FirebaseData firebasedata;


//------------------  volatile variables -------------------------------

uint64_t chipid;
String chipid_str;
//time
int8_t actual_hour;
int8_t actual_min;
int8_t actual_secs;

//enviroment
//raw
float dry_bulb_temp = 0;
float dry_bulb_temp_aux = 0;

float barometric_pressure = 0;
float relative_humidity = 0;
float relative_humidity_aux = 0;
float pressure_altitude = 0;
RTC_DATA_ATTR double relative_humidity_past = 0;

int wind_deg = 0;
float wind_speed = 0;


//constitutive
float solar_radiation = 0;
float vapor_pressure = 0;
float wet_bulb_temp = 0;

//Status
float neonode_temp;

int16_t LoRa_dB;
String LoRa_rssi;
String LoRa_snr;

RTC_DATA_ATTR double LATITUDE = -8.079025;
RTC_DATA_ATTR double LONGITUDE = -79.122865;

String port1_type;
float port1_variables[40];
int8_t port1_nvar = 0;
int16_t port1_msg_len;

String port2_type;
float port2_variables[40];
int8_t port2_nvar = 0;
int16_t port2_msg_len;

String port3_type;
float port3_variables[40];
int8_t port3_nvar = 0;
int16_t port3_msg_len;

String port4_type;
float port4_variables[40];
int8_t port4_nvar = 0;
int16_t port4_msg_len;

float P1_humidity_m3m3;
float P1_aparent_perm;
float P1_pore_perm;
float P1_water_pore_cond;

float P2_humidity_m3m3;
float P2_aparent_perm;
float P2_pore_perm;
float P2_water_pore_cond;

float P3_humidity_m3m3;
float P3_aparent_perm;
float P3_pore_perm;
float P3_water_pore_cond;

float P4_humidity_m3m3;
float P4_aparent_perm;
float P4_pore_perm;
float P4_water_pore_cond;
//-----Neonode variables----------------------------
float P1_humidity_m3m3_nn;
float P1_aparent_perm_nn;
float P1_pore_perm_nn;
float P1_water_pore_cond_nn;

float P2_humidity_m3m3_nn;
float P2_aparent_perm_nn;
float P2_pore_perm_nn;
float P2_water_pore_cond_nn;

float P3_humidity_m3m3_nn;
float P3_aparent_perm_nn;
float P3_pore_perm_nn;
float P3_water_pore_cond_nn;

float P4_humidity_m3m3_nn;
float P4_aparent_perm_nn;
float P4_pore_perm_nn;
float P4_water_pore_cond_nn;

float dry_bulb_temp_mov_nn;
float dry_bulb_temp_nn;
float barometric_pressure_nn;
float relative_humidity_nn;
float relative_humidity_aux_nn;
float pressure_altitude_nn;
float battery_voltage_nn;
float solar_voltage_nn;
float internal_temperature_nn;
float internal_temperature_raw_nn;
float dry_bulb_temp_aux_nn;
float nn_variables[27];




//------------------  other volatile meh variables ---------------------

unsigned long init_timestamp;
int8_t running_status;
int8_t no_atmos = 1;
int8_t counter_aux;
unsigned long start_MODE_PRG = 0;
int8_t half_conf_flag = 0;



//------------------  non-volatile configuration states ----------------
RTC_DATA_ATTR int8_t INCOMPATIBILIDAD_FIRMWARE_ERROR = 0;
RTC_DATA_ATTR int8_t Start_or_Restart = 1;
RTC_DATA_ATTR int8_t Update_LocalSN_Flag = 0;
RTC_DATA_ATTR int8_t NewConf_flag = 1;
RTC_DATA_ATTR int8_t battery_hysteresis = 0;
RTC_DATA_ATTR int8_t is_registered = 0;
RTC_DATA_ATTR int8_t WIFI_EN = 1;
RTC_DATA_ATTR int8_t wifi_default = 1;
RTC_DATA_ATTR int8_t wifi_conf_isSet = 0;  //1 if parameters are defined
RTC_DATA_ATTR int8_t sim_enable = 0;
RTC_DATA_ATTR int8_t GPS_RQ = 0;
RTC_DATA_ATTR int8_t PORT_RQ = 1;
RTC_DATA_ATTR int8_t BEEP_EN = 0;
RTC_DATA_ATTR int8_t GPS_CLOUD = 0;
RTC_DATA_ATTR int8_t OBSERVER = 0;
RTC_DATA_ATTR int8_t SENSOR_P1 = 0;
RTC_DATA_ATTR int8_t SENSOR_P2 = 0;
RTC_DATA_ATTR int8_t SENSOR_P3 = 0;
RTC_DATA_ATTR int8_t SENSOR_P4 = 0;

RTC_DATA_ATTR int DEPTH1;
RTC_DATA_ATTR int DEPTH2;
RTC_DATA_ATTR int DEPTH3;
RTC_DATA_ATTR int DEPTH4;

RTC_DATA_ATTR int UNNECESARY_MEASURE = 0;

bool   MODE_PRG = false;
int LOCAL_UPDATE = 0;

int8_t Port1_Active;
int8_t Port2_Active;
int8_t Port3_Active;
int8_t Port4_Active;

String port1_msg;
String port2_msg;
String port3_msg;
String port4_msg;


/*
  RTC_DATA_ATTR int8_t day_in_mem=1;
  RTC_DATA_ATTR int8_t month_in_mem=1;
  RTC_DATA_ATTR int8_t year_in_mem=1;
  RTC_DATA_ATTR double min_dayly_temperature;
  RTC_DATA_ATTR double max_dayly_temperature; */

RTC_DATA_ATTR float dry_bulb_temp_mov = 200;
RTC_DATA_ATTR float battery_voltage = -10;
RTC_DATA_ATTR float solar_voltage = -10;
RTC_DATA_ATTR float internal_temperature= 0;
float internal_temperature_raw =0 ;

RTC_DATA_ATTR float SLEEP_TIME = 900;
RTC_DATA_ATTR float N_SLEEP_TIME = 1200;
RTC_DATA_ATTR float NO_WIFI = 20;
RTC_DATA_ATTR float N_START_HOUR = 18;  //default N_START_HOUR
RTC_DATA_ATTR float N_END_HOUR = 4;    //same here

//------------------  non-volatile configuration values -----------------

RTC_DATA_ATTR char WIFI_SSID[30];
RTC_DATA_ATTR char WIFI_PSSWD[30];
RTC_DATA_ATTR float BAT_L = 3.25;
RTC_DATA_ATTR float BAT_H = 3.60;
RTC_DATA_ATTR int8_t prg_iteration = 0;
RTC_DATA_ATTR float SLEEP_TIME_modem;
int SLEEP_TIME_PRE = 35;

// to make temporary storage for the average

RTC_DATA_ATTR float MA_port1_G_1[3] = {0,0,0};
RTC_DATA_ATTR float MA_port2_G_1[3] = {0,0,0};
RTC_DATA_ATTR float MA_port3_G_1[3] = {0,0,0};
RTC_DATA_ATTR float MA_port4_G_1[3] = {0,0,0};

RTC_DATA_ATTR float MA_port1_G_3[3] = {0,0,0};
RTC_DATA_ATTR float MA_port2_G_3[3] = {0,0,0};
RTC_DATA_ATTR float MA_port3_G_3[3] = {0,0,0};
RTC_DATA_ATTR float MA_port4_G_3[3] = {0,0,0};
//Lora communication variables 
String incoming;               // incoming message
String outgoing; //outgoing message
byte msgCount=0;        // count of outgoing messages
long lastSendTime=0;    // last send time
int interval = 2000;    // interval between sends

//------------------  -----------------------------------  -----------------------------------  -----------------------------------  -----------------
//------------------  -----------------------------------  -----------------------------------  -----------------------------------  -----------------
//------------------  -----------------------------------  -----------------------------------  -----------------------------------  -----------------
//------------------  -----------------------------------  -----------------------------------  -----------------------------------  -----------------
//--------------------------------Funciones del sistema------------------------------------

double ReadVoltage(byte pin);
void deepsleep(int time2sleep);
int checking_battery();

void check_first_WiFi();

void turn_modem_on();



int16_t setting_wifi();
void starting_wifi();
void deepsleep_beep(int deepsleep_time, unsigned long beep_time);
void get_LOCAL_SN();

void NodeExistance();
void turn_modem_off();
int16_t real_sleep_time();
void check_registered();

void depth_request();
void check_configuration();

void moving_average_sensor();
void get_ports_sensor();

void update_firmware( String file);
bool verify_json ( String new_firmware_ver, String current_firmware_ver, int built, int chk);
int FirmwareCheck();
void read_OpenWeather();
void get_environment_sensor();

void get_neolink_status();

void transform_variables();

void get_neonodes_signals();

void compatibility_error();
void FORCED_RESET_TASK();
void send_cloud();
void config_LoRa();
void sendMessage(String outgoing);
String onReceive(int packetSize);
String reciveBig();
bool Comparator_msg(String msg1,String msg2);
bool Comparator(String msg);
void sendDeepSleep();
void LoRa_Communication();


String httpGETRequest(const char* serverName);
void setClock();
String getValue(String data, char separator, int index);
//--------------------------------------------------------
//--------------------------------------------------------
//--------------------------------------------------------
void setup() {

  pinMode(ARDUINO_RESTART, OUTPUT);
  digitalWrite(ARDUINO_RESTART, LOW);

  pinMode(BAT_SOLAR_EN, OUTPUT);
  digitalWrite(BAT_SOLAR_EN, LOW);

  pinMode(TEMP_EN, OUTPUT);
  digitalWrite(TEMP_EN, LOW);

  pinMode(ATMOS_EN, OUTPUT);
  digitalWrite(ATMOS_EN, LOW);

  pinMode(SIM_ON, OUTPUT);
  digitalWrite(SIM_ON, HIGH);

  pinMode(FORCED_ATMEGA_RESET, OUTPUT);

 

  //Heltec.begin(false /*DisplayEnable Enable*/, false /*Heltec.LoRa Disable*/, false /*Serial Enable*/, false /*PABOOST Enable*/, BAND /*long BAND*/);
  //delay(1000);
  ArdSerial.begin(57600);
  Serial.begin(115200);

  EEPROM.begin(11);

  String sn_temporal="XX0000-0000";
  for(int i=0; i<11;i++) {
    //Serial.println((char)EEPROM.read(i)); 
    sn_temporal[i]=(char)EEPROM.read(i);
  }

  //SN_WE= sn_temporal + '\0';
  SN = sn_temporal;
  chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  chipid_str = String((uint16_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);
  

  
  
  

  if (prg_iteration == 0) {
    Serial.println("\n----------------------------------");
    Serial.println("[STEP 1]: ");
    if (!checking_battery()) 
     {
      Serial.println(" Not enought energy. Shutting down until battery_hysteresis ");
      deepsleep(POWERLESS_TIME);

    }
    
    check_first_WiFi();


    //physical forced reset Atmega328pu
    Serial.println("[Prevention] Physical Atmega328PU forced resetting.");
    
    digitalWrite(FORCED_ATMEGA_RESET, HIGH);
    delay(1000);
    digitalWrite(FORCED_ATMEGA_RESET, LOW);

    prg_iteration = 9;
    turn_modem_on();
    
    if (Start_or_Restart) beep.vbeep(250);
    if(FIRMWARE_MODE == 'DEV'||FIRMWARE_MODE == 'DEV2.2'||FIRMWARE_MODE == 'DEV2.1') deepsleep(1);
    deepsleep(SLEEP_TIME_PRE);


  }

  else if (prg_iteration == 9) {

   
    

    Serial.println("[STEP 2]:" );//principal code
    if(SN[0]=='X' || Update_LocalSN_Flag) get_LOCAL_SN();
    else{
      Serial.println("CHIP ID: " + chipid_str);
      Serial.println("SN FOUND: " + SN+"");
    }

    

    if (SN[0] == DEVICE_HEADER[0] && SN[1] == DEVICE_HEADER[1] ) INCOMPATIBILIDAD_FIRMWARE_ERROR =0;
    else INCOMPATIBILIDAD_FIRMWARE_ERROR = 1;

    DEVICE = "/" + DEVICE_TYPE + "/" + SN ;
    

    Serial.println("NeoLink version=" + FIRMWARE_VER);

    //---------------------------    Checking battery to start---------------

    if (checking_battery()) check_registered(); //checking if neolink is registered;
    //if (1) check_registered();
    else {
      Serial.println(" Not enought energy. Shutting down until battery_hysteresis ");
      deepsleep(POWERLESS_TIME);
    }

    
    if (INCOMPATIBILIDAD_FIRMWARE_ERROR) compatibility_error();

    if (is_registered) {
      if(!FIRMWARE_MODE == 'DEV2.2'){
        check_configuration();
        get_ports_sensor();
        get_environment_sensor();//BME
      } 
   
      get_neolink_status(); //solar, internal temp
      transform_variables();
      send_cloud();  
        //if (MODE_PRG)  start_MODE_PRG = millis();
    


    }

  }
}

void loop() {
  if ( MODE_PRG) {
    //server.handleClient();
    //delay(1);
    Serial.println("No MODE_PRG defined in this version");
    deepsleep(1);
  }
  //if (start_MODE_PRG - millis() >=300000) deepsleep(1);
  deepsleep(1);
}

//---------------------------------------------------------

void check_first_WiFi(){
  Serial.println("Checking WiFi Default as First Step... NOT IMPLEMENTET JET");
};


int checking_battery() {
if(FIRMWARE_MODE == 'DEV'||FIRMWARE_MODE == 'DEV2.2'||FIRMWARE_MODE == 'DEV2.1') return 1;
  Serial.println("Checking Battery.. ");
  pinMode(BAT_VALUE, INPUT_PULLDOWN);
  pinMode(SOLAR_VALUE, INPUT);

  digitalWrite(BAT_SOLAR_EN, HIGH);
  delay(20);
  float solar_voltage_temp;
  float battery_voltage_temp;
  solar_voltage_temp  = ReadVoltage(SOLAR_VALUE) ;
  
  battery_voltage_temp = ReadVoltage(BAT_VALUE) ;
  //battery_voltage_temp = 2000;

  digitalWrite(BAT_SOLAR_EN, LOW);
  if (battery_voltage <= 0 ) {
    battery_voltage = battery_voltage_temp * 0.00182189 ;
  }
  if ( solar_voltage <= 0) {
    solar_voltage = solar_voltage_temp * 0.003692945;
  }
  //Serial.print ("past solar_voltage: ");
  //Serial.println (solar_voltage);
  //Serial.print ("past battery_voltage: ");
  //Serial.println (battery_voltage);

  solar_voltage = solar_voltage_temp*0.003692946  ;
  //solar_voltage = (solar_voltage_temp*0.002371486  + solar_voltage) / 2 ;
  battery_voltage = (battery_voltage_temp * 0.00182189 + battery_voltage ) / 2 ;

  Serial.print (" solar_voltage: ");
  Serial.println (solar_voltage);
  Serial.print (" battery_voltage: ");
  Serial.println (battery_voltage);


  if (battery_voltage < BAT_L && !battery_hysteresis ) {
    battery_hysteresis = 1;

  }

  if (battery_voltage >= BAT_H && battery_hysteresis ) {
    battery_hysteresis = 0;
  }

  return !battery_hysteresis;
}

void get_LOCAL_SN(){
  //if( SN[0]=='N' || SN[1]!='L' )
  //for(int i=0; i<SN.length();i++) SN[i] = (char) EEPROM.read(i);
  //.println(SN);
  

  Serial.println("Actual SN: " + SN);
  Serial.println("Getting Serial Number..");
 
  if (!wifi_conf_isSet) wifi_conf_isSet = setting_wifi();
  if ( WiFi.status() != WL_CONNECTED) starting_wifi();
  if ( WiFi.status() == WL_CONNECTED) {
      unsigned long init_SN_time= millis();
      while(!Firebase.getString(firebasedata, "SN_CHIPS/" + chipid_str + "/SN_LOCAL"))if(millis()-init_SN_time >10000)deepsleep_beep(1,1000) ;
      SN=firebasedata.stringData();
      Serial.println(SN);

      for(int i=0; i<SN.length();i++)  {
        //Serial.println(SN[i]);
        EEPROM.write(i, SN[i]);
        EEPROM.commit(); 
        //Serial.println(EEPROM.read(i));
      }

      Firebase.setInt(firebasedata, "SN_CHIPS/" + chipid_str + "/UPDATE", 0);

      Serial.println("State saved in flash memory");

      Update_LocalSN_Flag = 0; 
      deepsleep_beep(1,800);
      
    }

  Update_LocalSN_Flag = 0;

}





//------------------------------------------------------------------
void check_registered() {
  if (!wifi_conf_isSet) wifi_conf_isSet = setting_wifi();

  //if ( WiFi.status() != WL_CONNECTED) starting_wifi();

  if (!is_registered) {
    //check if is it

    if ( WiFi.status() != WL_CONNECTED) starting_wifi();

    if ( WiFi.status() == WL_CONNECTED) {
      is_registered = Firebase.getString(firebasedata, "OLDneolinks/" + SN + "/correo");
      Firebase.getInt(firebasedata, "SN_CHIPS/" + chipid_str + "/UPDATE"); // if reset, newconf is 1 by default.
      Update_LocalSN_Flag = firebasedata.intData();
      //Serial.println(Update_LocalSN_Flag);
      
    }


    if (!is_registered) {
      //Serial.println("Device " + SN + "  is not registered. Shutting down.");
      if(INCOMPATIBILIDAD_FIRMWARE_ERROR) Serial.println("[ERROR] FIRMWARE NO COMPATIBLE.");
      Serial.println("Device " + SN + "  is not registered. Shutting down.");
      beep.vbeep(100);
      delay(30);
      beep.vbeep(300);
      delay(30);
      beep.vbeep(100);
      if (prg_iteration == 0) prg_iteration = 9;
      else prg_iteration = 0;
      deepsleep(UNREGISTERED_TIME);
    } else {
      //Serial.println(" Device " + SN + "..Registered");
      Serial.println(" Device " + SN + "..Registered");
      NodeExistance();

    }

  }
  else {

    Serial.println("Device " + SN + " already registered.");

    /* if (!Start_or_Restart) {
      Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_STATUS + "NewConf"); // if reset, newconf is 1 by default.
      NewConf_flag = firebasedata.intData();} */


  }

  if (Start_or_Restart) {
    //det hora
    configTime(gmtOffset_sec + 70, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    int time_rep = 0;
    while ( (timeinfo.tm_year - 100 < 0) )
    {
      getLocalTime(&timeinfo);
      delay(100);
      time_rep ++;
      Serial.print("Time obtained error. Try: ");
      Serial.println(time_rep);
      if (time_rep > 4) deepsleep(1);
    }
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

    String _year = String(timeinfo.tm_year - 100);
    int8_t i_mon = timeinfo.tm_mon;
    int8_t i_day = timeinfo.tm_mday;
    int8_t i_hour = timeinfo.tm_hour;
    int8_t i_min = timeinfo.tm_min;
    int8_t r_i_min = i_min;
    int8_t i_secs = timeinfo.tm_sec;

    if (!half_conf_flag) {
      Firebase.getFloat(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "SLEEP_TIME");
      SLEEP_TIME = firebasedata.floatData();
      Firebase.getFloat(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "N_SLEEP_TIME");
      N_SLEEP_TIME = firebasedata.floatData();
      Firebase.getFloat (firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "N_END_HOUR");
      N_END_HOUR = firebasedata.floatData();
      Firebase.getFloat(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "N_START_HOUR");
      N_START_HOUR = firebasedata.floatData();
      half_conf_flag = 1;

    }

    float sleepy_time;
    if (i_hour > N_END_HOUR && i_hour < N_START_HOUR) sleepy_time = SLEEP_TIME;
    else sleepy_time = N_SLEEP_TIME;
    int factor_temp = sleepy_time / 60;
    

    i_min = (i_min / factor_temp) * factor_temp;

    String real_month;
    if (i_mon < 9) real_month = "0" + String(i_mon + 1);
    else real_month = String(i_mon + 1);

    String real_day;
    if (i_day < 10) real_day = "0" + String(i_day);
    else real_day = String(i_day);

    String real_hour;
    if (i_hour < 10) real_hour = "0" + String(i_hour);
    else real_hour = String(i_hour);

    String real_min;
    if (i_min < 10) real_min = "0" + String(i_min);
    else real_min = String(i_min);


    //creo cadena

    String auxiliar_string_2 = _year + "/" + real_month + "/" + real_day + " " + real_hour + ":" + real_min ;
    //Serial.println(auxiliar_string);
    Serial.print("Verifying last record @ " + auxiliar_string_2);
    //verifico si existe y doy pase
    String auxiliar_string = _year + "/" + real_month + "/" + real_day + "/" + real_hour + "/" + real_min ;

    boolean cond1 = Firebase.getFloat(firebasedata, DEVICE + PATH_DATA  + "/State/" + auxiliar_string + "/RH" );
    //verify correct sensor measure
    boolean cond2 = 1;

    

    if (cond1 && cond2 )
    { 
      Serial.println("\n Unnecessary measure.");
      UNNECESARY_MEASURE = 1;
      

      //sino chanco los valores de tiempo a las variales globales
      actual_hour = i_hour;
      actual_min = r_i_min - 1;
      actual_secs = i_secs - 10; //25 - 15 sec offset
      if (actual_secs < 0) {
        i_secs = i_secs + 60;
        i_min = i_min - 1;
      }
      if (actual_min < 0) {
        actual_min = actual_min + 60;
        actual_hour = actual_hour - 1;
        if (actual_hour < 0) actual_hour = 23;

      }

      if (actual_hour < 10) real_hour = "0" + String(actual_hour);
      else real_hour = String(actual_hour);

      if (actual_min < 10) real_min = "0" + String(actual_min);
      else real_min = String(actual_min);

      String real_sec;
      if (actual_secs < 10) real_sec = "0" + String(actual_secs);
      else real_sec = String(actual_secs);


      auxiliar_string_2 = _year + "/" + real_month + "/" + real_day + " " + real_hour + ":" + real_min + ":" + real_sec;

      Firebase.setString(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "LastRestart", auxiliar_string_2);

      delay(100);

      //deepsleep
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      Start_or_Restart = 0;
      turn_modem_off();
      deepsleep(real_sleep_time());
    }
    else Serial.println("\n Continue to measuring..");




  }

}

//------------------------------------------------------------------
int16_t setting_wifi() {


  char recipe_buf[35];
  WIFI_SSID_DEFAULT.toCharArray(recipe_buf, 35); //WIFI_SSID should refresh itself in the end

  for (int i = 0; i < WIFI_SSID_DEFAULT.length() + 1; i++) {
    WIFI_SSID[i] = recipe_buf[i];
  }


  WIFI_PSSWD_DEFAULT.toCharArray(recipe_buf, 35);
  for (int i = 0; i < WIFI_PSSWD_DEFAULT.length() + 1; i++) {
    WIFI_PSSWD[i] = recipe_buf[i];
  }

  Serial.println("Wifi is now set.");


  return 1;
}
//------------------------------------------------------------------

void check_configuration() {



  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);



  if (Start_or_Restart || UNNECESARY_MEASURE) {
    Serial.println("Reading new configuration due restart or first run.");
    NewConf_flag = 1;
    UNNECESARY_MEASURE = 0;
  } else {


  }



  if (NewConf_flag) {

    Serial.print("Checking new configuration...");



    Firebase.getFloat(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "BAT_H");
    BAT_H = firebasedata.floatData();
    Serial.print("\n BAT_H: " + String(BAT_H));

    Firebase.getFloat(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "BAT_L");
    BAT_L = firebasedata.floatData();
    Serial.print("\n BAT_L: " + String(BAT_L));

    Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "BEEP_EN");
    BEEP_EN = firebasedata.intData();
    Serial.print("\n BEEP_EN: " + String(BEEP_EN));

    //Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + String("GPS_RQ"));
    //GPS_RQ = firebasedata.intData();

    Firebase.getFloat(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "NO_WIFI");
    NO_WIFI = firebasedata.floatData();
    Serial.print("\n NO_WIFI: " + String(NO_WIFI));

    if (!half_conf_flag) {

      Firebase.getFloat (firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "N_END_HOUR");
      N_END_HOUR = firebasedata.floatData();
      Serial.print("\n N_END_HOUR: " + String(N_END_HOUR));

      Firebase.getFloat(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "N_SLEEP_TIME");
      N_SLEEP_TIME = firebasedata.floatData();
      Serial.print("\n N_SLEEP_TIME: " + String(N_SLEEP_TIME));

      Firebase.getFloat(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "N_START_HOUR");
      N_START_HOUR = firebasedata.floatData();
      Serial.print("\n N_START_HOUR: " + String(N_START_HOUR));

      Firebase.getFloat(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "SLEEP_TIME");
      SLEEP_TIME = firebasedata.floatData();
      Serial.print("\n SLEEP_TIME: " + String(SLEEP_TIME));

    }

    else Serial.print("\n SLEEP_TIME parameters already read.");

    Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "PORT_RQ");
    PORT_RQ = firebasedata.intData();
    Serial.print("\n PORT_RQ: " + String(PORT_RQ));
    delay(100);

    depth_request();

    Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "OBSERVER");
    OBSERVER = firebasedata.intData();
    Serial.print("\n OBSERVER: " + String(OBSERVER));
    delay(100);

    Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "SENSOR/P1");
    SENSOR_P1 = firebasedata.intData();
    Serial.print("\n SENSOR P1: " + String(SENSOR_P1));
    delay(100);
    
    Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "SENSOR/P2");
    SENSOR_P2 = firebasedata.intData();
    Serial.print("\n SENSOR P2: " + String(SENSOR_P2));
    delay(100);
    
    Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "SENSOR/P3");
    SENSOR_P3 = firebasedata.intData();
    Serial.print("\n SENSOR P3: " + String(SENSOR_P3));
    delay(100);

    Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "SENSOR/P4");
    SENSOR_P4 = firebasedata.intData();
    Serial.print("\n SENSOR P4: " + String(SENSOR_P4));
    delay(100);


    

    

    //Serial.print("programming mode doesnt exist in this version.");
    //Firebase.getBool(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + String("MODE_PRG"));
    //MODE_PRG = firebasedata.boolData();
    //if (!MODE_PRG) Serial.println("Negative.");
    //else Serial.println("Active.");
    Serial.println("\n Done.");
    NewConf_flag = 0;


  }
  else
  {
    Serial.println("No configuration found.");
  }



}

//------------------------------------------------------------------

void get_ports_sensor() {
  int8_t loop1_flag = 1;
  char message[500];
  int16_t port_div[50];
  int8_t sample_div[50];
  int16_t eos; //end of string
  int16_t m = 0;
  int8_t n = 0; // counter ports
  int8_t try_counter = 0;

  //String port1_msg;
  String port1_sample1;
  String port1_sample2;
  String port1_sample3;


  //String port2_msg;
  String port2_sample1;
  String port2_sample2;
  String port2_sample3;

  //String port3_msg;
  String port3_sample1;
  String port3_sample2;
  String port3_sample3;

  //String port4_msg;
  String port4_sample1;
  String port4_sample2;
  String port4_sample3;


  if (PORT_RQ) {
    Serial.println("Rebooting atmega..");
    digitalWrite(ARDUINO_RESTART, HIGH);
    delay(1);
    digitalWrite(ARDUINO_RESTART, LOW );
    delay(10);
    //restarting atmega to avoid overflow

    ArdSerial.write('s');
    Serial.println("Sensors on port requested. Atmega response: ");
    unsigned long time_sensor = millis();
    while (loop1_flag) {

      if (ArdSerial.available()) {

        message[m] = ArdSerial.read();

        if (message[m] == '$') {
          eos = m;
          //Serial.println(eos);
          break;
        }
        else if (message[m] == '%') {
          port_div[n] = m;

          //Serial.print(n);
          //Serial.print(": ");
          //Serial.println(m);

          n++;
        }



        m++;

      } else {
        if (millis() - time_sensor > 40000) {
          try_counter++;
          if (try_counter >= 2) {
            Serial.println("No sensor detected.");
            return;
          }
          Serial.println("Rebooting atmega.. again");
          time_sensor = millis();
          digitalWrite(ARDUINO_RESTART, HIGH);
          delay(1);
          digitalWrite(ARDUINO_RESTART, LOW );
          delay(10);
          //restarting atmega to avoid overflow
          ArdSerial.write('s');

        }


      }

    }
    /*
      Serial.println(port_div[0]);
      Serial.println(port_div[1]);
      Serial.println(port_div[2]);
      Serial.println(port_div[3]);
      Serial.println(eos);

      Serial.println(String(message));
    */

    port1_msg = String(message).substring(port_div[0] + 4, port_div[1]);
    port2_msg = String(message).substring(port_div[1] + 4, port_div[2]);
    port3_msg = String(message).substring(port_div[2] + 4, port_div[3]);
    port4_msg = String(message).substring(port_div[3] + 4, eos       );


    Serial.println(port1_msg);
    Serial.println(port2_msg);
    Serial.println(port3_msg);
    Serial.println(port4_msg);


    /*
      01:11:12.511 -> %P1_-3235.2 20.2_-3105.5 20.2_-2969.8 20.2_k%P2_-3036.4 20.2_-3006.5 20.2_k%P3 %P4 $
      01:11:12.511 -> -3235.2 20.2_-3105.5 20.2_-2969.8 20.2_k
      01:11:12.511 -> -3036.4 20.2_-3006.5 20.2_k

    */


    //--------------------------------PORT1-----------------------------------------------
    Serial.println("--------------------------- PORT 1 -----------------------------");

    port1_msg_len = port1_msg.length();
    sample_div[0] = 0;

    if (port1_msg_len > 0) {
      int8_t n_samples = 0;
      for (int i = 0; i <= port1_msg_len ; i++) {
        if (port1_msg[i] == '_') {
          sample_div[n_samples] = i;
          n_samples++;
        }

      }


      port1_sample1 = port1_msg.substring(0, sample_div[0]);
      port1_sample2 = port1_msg.substring(sample_div[0] + 1 , sample_div[1]);
      port1_sample3 = port1_msg.substring(sample_div[1] + 1, port1_msg_len - 2);
      port1_type = port1_msg.substring(port1_msg_len - 1, port1_msg_len);

      //Serial.println(port1_msg);

      /* int8_t n_samples = 0;
        if (port1_sample1.length() > 1) n_samples++;
        if (port1_sample2.length() > 1) n_samples++;
        if (port1_sample3.length() > 1) n_samples++;
        Serial.println("n_samples: " + String(n_samples)); */
      Port1_Active = 1;
      Serial.print("Port1_Active ");
      Serial.println(Port1_Active);

      //Serial.println("n_samples: " + String(n_samples));



      //Serial.println(port1_sample1);
      //Serial.println(port1_sample2);
      //Serial.println(port1_sample3);
      Serial.println(port1_type);


      port1_nvar = 1;
      counter_aux = 0;
      //------------------------  samples  -----------------------------
      for (int i = 0; i <= port1_msg_len ; i++) {
        if (port1_sample1[i] == ' ') {
          port1_variables[port1_nvar - 1] = (port1_sample1.substring(counter_aux, i).toFloat() + port1_sample2.substring(counter_aux, i).toFloat() + port1_sample3.substring(counter_aux, i).toFloat()) / n_samples;
          counter_aux = i + 1;
          port1_nvar++;
        }

        if (i == port1_msg_len) {
          port1_variables[port1_nvar - 1] = (port1_sample1.substring(counter_aux, i - 1).toFloat() + port1_sample2.substring(counter_aux, i - 1).toFloat() + port1_sample3.substring(counter_aux, i - 1).toFloat()) / n_samples;


        }

      }
    }
    else {
      Port1_Active = 0;
      Serial.print("Port1_Active ");
      Serial.println(Port1_Active);
    }
    for (int i = 0; i < port1_nvar; i++) {
      Serial.println(port1_variables[i]);

    }


    //--------------------------------PORT2-----------------------------------------------
    Serial.println("--------------------------- PORT 2 -----------------------------");

    port2_msg_len = port2_msg.length();
    sample_div[0] = 0;

    if (port2_msg_len > 0) {
      int8_t n_samples = 0;
      for (int i = 0; i <= port2_msg_len ; i++) {
        if (port2_msg[i] == '_') {
          sample_div[n_samples] = i;
          n_samples++;
        }

      }


      port2_sample1 = port2_msg.substring(0, sample_div[0]);
      port2_sample2 = port2_msg.substring(sample_div[0] + 1 , sample_div[1]);
      port2_sample3 = port2_msg.substring(sample_div[1] + 1, port2_msg_len - 2);
      port2_type = port2_msg.substring(port2_msg_len - 1, port2_msg_len);

      //Serial.println(port1_msg);

      /* int8_t n_samples = 0;
        if (port1_sample1.length() > 1) n_samples++;
        if (port1_sample2.length() > 1) n_samples++;
        if (port1_sample3.length() > 1) n_samples++;
        Serial.println("n_samples: " + String(n_samples)); */

      Port2_Active = 1;
      Serial.print("Port2_Active ");
      Serial.println(Port2_Active);

      //Serial.println("n_samples: " + String(n_samples));

      //Serial.println(port2_sample1);
      //Serial.println(port2_sample2);
      //Serial.println(port2_sample3);
      Serial.println(port2_type);


      port2_nvar = 1;
      counter_aux = 0;
      //------------------------  samples  -----------------------------
      for (int i = 0; i <= port2_msg_len ; i++) {
        if (port2_sample1[i] == ' ') {
          port2_variables[port2_nvar - 1] = (port2_sample1.substring(counter_aux, i).toFloat() + port2_sample2.substring(counter_aux, i).toFloat() + port2_sample3.substring(counter_aux, i).toFloat()) / n_samples;
          counter_aux = i + 1;
          port2_nvar++;
        }

        if (i == port2_msg_len) {
          port2_variables[port2_nvar - 1] = (port2_sample1.substring(counter_aux, i - 1).toFloat() + port2_sample2.substring(counter_aux, i - 1).toFloat() + port2_sample3.substring(counter_aux, i - 1).toFloat()) / n_samples;


        }

      }
    }
    else {
      Port2_Active = 0;
      Serial.print("Port1_Active ");
      Serial.println(Port2_Active);
    }
    for (int i = 0; i < port2_nvar; i++) {
      Serial.println(port2_variables[i]);

    }

    //--------------------------------PORT3-----------------------------------------------
    Serial.println("--------------------------- PORT 3 -----------------------------");

    port3_msg_len = port3_msg.length();
    sample_div[0] = 0;

    if (port3_msg_len > 0) {
      int8_t n_samples = 0;
      for (int i = 0; i <= port3_msg_len ; i++) {
        if (port3_msg[i] == '_') {
          sample_div[n_samples] = i;
          n_samples++;
        }

      }


      port3_sample1 = port3_msg.substring(0, sample_div[0]);
      port3_sample2 = port3_msg.substring(sample_div[0] + 1 , sample_div[1]);
      port3_sample3 = port3_msg.substring(sample_div[1] + 1, port3_msg_len - 2);
      port3_type = port3_msg.substring(port3_msg_len - 1, port3_msg_len);

      //Serial.println(port1_msg);

      /* int8_t n_samples = 0;
        if (port1_sample1.length() > 1) n_samples++;
        if (port1_sample2.length() > 1) n_samples++;
        if (port1_sample3.length() > 1) n_samples++;
        Serial.println("n_samples: " + String(n_samples)); */

      Port3_Active = 1;
      Serial.print("Port3_Active ");
      Serial.println(Port3_Active);

      //Serial.println("n_samples: " + String(n_samples));

      //Serial.println(port3_sample1);
      //Serial.println(port3_sample2);
      //Serial.println(port3_sample3);
      Serial.println(port3_type);


      port3_nvar = 1;
      counter_aux = 0;
      //------------------------  samples  -----------------------------
      for (int i = 0; i <= port3_msg_len ; i++) {
        if (port3_sample1[i] == ' ') {
          port3_variables[port3_nvar - 1] = (port3_sample1.substring(counter_aux, i).toFloat() + port3_sample2.substring(counter_aux, i).toFloat() + port3_sample3.substring(counter_aux, i).toFloat()) / n_samples;
          counter_aux = i + 1;
          port3_nvar++;
        }

        if (i == port3_msg_len) {
          port3_variables[port3_nvar - 1] = (port3_sample1.substring(counter_aux, i - 1).toFloat() + port3_sample2.substring(counter_aux, i - 1).toFloat() + port3_sample3.substring(counter_aux, i - 1).toFloat()) / n_samples;


        }

      }
    }
    else {
      Port3_Active = 0;
      Serial.print("Port3_Active ");
      Serial.println(Port3_Active);
    }
    for (int i = 0; i < port3_nvar; i++) {
      Serial.println(port3_variables[i]);

    }

    //--------------------------------PORT4-----------------------------------------------
    Serial.println("--------------------------- PORT 4 -----------------------------");

    port4_msg_len = port4_msg.length();
    sample_div[0] = 0;

    if (port4_msg_len > 0) {
      int8_t n_samples = 0;
      for (int i = 0; i <= port4_msg_len ; i++) {
        if (port4_msg[i] == '_') {
          sample_div[n_samples] = i;
          n_samples++;
        }

      }


      port4_sample1 = port4_msg.substring(0, sample_div[0]);
      port4_sample2 = port4_msg.substring(sample_div[0] + 1 , sample_div[1]);
      port4_sample3 = port4_msg.substring(sample_div[1] + 1, port4_msg_len - 2);
      port4_type = port4_msg.substring(port4_msg_len - 1, port4_msg_len);

      //Serial.println(port1_msg);

      /* int8_t n_samples = 0;
        if (port1_sample1.length() > 1) n_samples++;
        if (port1_sample2.length() > 1) n_samples++;
        if (port1_sample3.length() > 1) n_samples++;
        Serial.println("n_samples: " + String(n_samples)); */

      Port4_Active = 1;
      Serial.print("Port4_Active ");
      Serial.println(Port4_Active);

      //Serial.println("n_samples: " + String(n_samples));

      //Serial.println(port4_sample1);
      //Serial.println(port4_sample2);
      //Serial.println(port4_sample3);
      Serial.println(port4_type);


      port4_nvar = 1;
      counter_aux = 0;
      //------------------------  samples  -----------------------------
      for (int i = 0; i <= port4_msg_len ; i++) {
        if (port4_sample1[i] == ' ') {
          port4_variables[port4_nvar - 1] = (port4_sample1.substring(counter_aux, i).toFloat() + port4_sample2.substring(counter_aux, i).toFloat() + port4_sample3.substring(counter_aux, i).toFloat()) / n_samples;
          counter_aux = i + 1;
          port4_nvar++;
        }

        if (i == port4_msg_len) {
          port4_variables[port4_nvar - 1] = (port4_sample1.substring(counter_aux, i - 1).toFloat() + port4_sample2.substring(counter_aux, i - 1).toFloat() + port4_sample3.substring(counter_aux, i - 1).toFloat()) / n_samples;


        }

      }
    }
    else {
      Port4_Active = 0;
      Serial.print("Port4_Active ");
      Serial.println(Port4_Active);
    }


    for (int i = 0; i < port4_nvar; i++) {
      Serial.println(port4_variables[i]);

    }

    

    //-------------------------------------------------------------------------------
    Serial.println("--------------------------- END PORT -----------------------------");
    moving_average_sensor();

  }
  else Serial.println("No Port sensing  request.");
  Serial.println();
}
//------------------------------------------------------------------

void moving_average_sensor() {
  
  if (Port1_Active) {
    if (port1_type.equals("g") ) {
      if (Start_or_Restart || UNNECESARY_MEASURE || MA_port1_G_1[2] <=0 || MA_port1_G_1[1] <= 0 || MA_port1_G_1[0] <= 0 || MA_port1_G_3[2] <=0 || MA_port1_G_3[1] <= 0 || MA_port1_G_3[0] <= 0  ) {
        

        MA_port1_G_1[0] = port1_variables[0];
        MA_port1_G_1[1] = port1_variables[0];
        MA_port1_G_1[2] = port1_variables[0];

        MA_port1_G_3[0] = port1_variables[2];
        MA_port1_G_3[1] = port1_variables[2];
        MA_port1_G_3[2] = port1_variables[2];


      }
      else {
        MA_port1_G_1[0] = MA_port1_G_1[1];
        MA_port1_G_1[1] = MA_port1_G_1[2];
        MA_port1_G_1[2] = port1_variables[0];

        MA_port1_G_3[0] = MA_port1_G_3[1];
        MA_port1_G_3[1] = MA_port1_G_3[2];
        MA_port1_G_3[2] = port1_variables[2];
      }


      if(port1_variables[0]>0) port1_variables[0] = (MA_port1_G_1[0] + MA_port1_G_1[1] + MA_port1_G_1[2]) / 3;
      if(port1_variables[2]>0) port1_variables[2] = (MA_port1_G_3[0] + MA_port1_G_3[1] + MA_port1_G_3[2]) / 3;


    }

  }

  if (Port2_Active) {
    if (port2_type.equals("g")) {
      if (Start_or_Restart || UNNECESARY_MEASURE || MA_port2_G_1[2] <=0 || MA_port2_G_1[1] <= 0 || MA_port2_G_1[0] <= 0 || MA_port2_G_3[2] <=0 || MA_port2_G_3[1] <= 0 || MA_port2_G_3[0] <= 0  ) {
        MA_port2_G_1[0] = port2_variables[0];
        MA_port2_G_1[1] = port2_variables[0];
        MA_port2_G_1[2] = port2_variables[0];

        MA_port2_G_3[0] = port2_variables[2];
        MA_port2_G_3[1] = port2_variables[2];
        MA_port2_G_3[2] = port2_variables[2];


      }
      else {
        MA_port2_G_1[0] = MA_port2_G_1[1];
        MA_port2_G_1[1] = MA_port2_G_1[2];
        MA_port2_G_1[2] = port2_variables[0];

        MA_port2_G_3[0] = MA_port2_G_3[1];
        MA_port2_G_3[1] = MA_port2_G_3[2];
        MA_port2_G_3[2] = port2_variables[2];
      }

     

      if(port2_variables[0]>0)  port2_variables[0] = (MA_port2_G_1[0] + MA_port2_G_1[1] + MA_port2_G_1[2]) / 3;
      if(port2_variables[2]>0)  port2_variables[2] = (MA_port2_G_3[0] + MA_port2_G_3[1] + MA_port2_G_3[2]) / 3;


    }

  }

  if (Port3_Active) {
    if (port3_type.equals("g")) {
      if (Start_or_Restart || UNNECESARY_MEASURE || MA_port3_G_1[2] <=0 || MA_port3_G_1[1] <= 0 || MA_port3_G_1[0] <= 0 || MA_port3_G_3[2] <=0 || MA_port3_G_3[1] <= 0 || MA_port3_G_3[0] <= 0) {
        MA_port3_G_1[0] = port3_variables[0];
        MA_port3_G_1[1] = port3_variables[0];
        MA_port3_G_1[2] = port3_variables[0];

        MA_port3_G_3[0] = port3_variables[2];
        MA_port3_G_3[1] = port3_variables[2];
        MA_port3_G_3[2] = port3_variables[2];


      }
      else {
        MA_port3_G_1[0] = MA_port3_G_1[1];
        MA_port3_G_1[1] = MA_port3_G_1[2];
        MA_port3_G_1[2] = port3_variables[0];

        MA_port3_G_3[0] = MA_port3_G_3[1];
        MA_port3_G_3[1] = MA_port3_G_3[2];
        MA_port3_G_3[2] = port3_variables[2];
      }

      if(port3_variables[0]>0) port3_variables[0] = (MA_port3_G_1[0] + MA_port3_G_1[1] + MA_port3_G_1[2]) / 3;
      if(port3_variables[2]>0) port3_variables[2] = (MA_port3_G_3[0] + MA_port3_G_3[1] + MA_port3_G_3[2]) / 3;


    }

  }

  if (Port4_Active) {
    if (port4_type.equals("g")) {
      if (Start_or_Restart || UNNECESARY_MEASURE || MA_port4_G_1[2] <=0 || MA_port4_G_1[1] <= 0 || MA_port4_G_1[0] <= 0 || MA_port4_G_3[2] <=0 || MA_port4_G_3[1] <= 0 || MA_port4_G_3[0] <= 0) {
        MA_port4_G_1[0] = port4_variables[0];
        MA_port4_G_1[1] = port4_variables[0];
        MA_port4_G_1[2] = port4_variables[0];

        MA_port4_G_3[0] = port4_variables[2];
        MA_port4_G_3[1] = port4_variables[2];
        MA_port4_G_3[2] = port4_variables[2];


      }
      else {
        MA_port4_G_1[0] = MA_port4_G_1[1];
        MA_port4_G_1[1] = MA_port4_G_1[2];
        MA_port4_G_1[2] = port4_variables[0];

        MA_port4_G_3[0] = MA_port4_G_3[1];
        MA_port4_G_3[1] = MA_port4_G_3[2];
        MA_port4_G_3[2] = port4_variables[2];
      }

      if(port4_variables[0]>0) port4_variables[0] = (MA_port4_G_1[0] + MA_port4_G_1[1] + MA_port4_G_1[2]) / 3;
      if(port4_variables[2]>0) port4_variables[2] = (MA_port4_G_3[0] + MA_port4_G_3[1] + MA_port4_G_3[2]) / 3;


    }

  }



}

void get_gps() {

  char message[200];
  int8_t divider;
  int8_t eos; //end of string
  int8_t m = 0;
  unsigned long initial_t;
  int8_t error_gps = 0;



  if (GPS_RQ) {

    Serial.println("Rebooting atmega..");

    delay(50);
    digitalWrite(ARDUINO_RESTART, HIGH);
    delay(50);
    digitalWrite(ARDUINO_RESTART, LOW );
    delay(10);
    //Serial.println("GPS reading requested.");
    //restarting atmega to avoid overflow
    ArdSerial.read();
    ArdSerial.println('g');
    Serial.println("GPS requested." );
    unsigned long gps_start_time = millis();
    char gps_data[25];
    int8_t ack_gps_try = 1;
    int8_t data = 0;

    while (1)
    {
      if (ArdSerial.available()) {

        char ch = ArdSerial.read();
        //Serial.println(ch);
        if (ch == '@') {
          data = 1;
          Serial.println("GPS starts job.")  ;
          break;
        }
        else {
          Serial.println("Unexpected answer.")  ;
          deepsleep(100);
        }



      }
      else if (millis() - gps_start_time > 2000) {
        if (ack_gps_try > 2) {

          Serial.println("error GPS. Sorry");
          LATITUDE = -8.079025;
          LONGITUDE = -79.122865 ;
          break;
        }


        delay(50);
        digitalWrite(ARDUINO_RESTART, HIGH);
        delay(50);
        digitalWrite(ARDUINO_RESTART, LOW );
        delay(10);
        ArdSerial.println('g');
        ++ack_gps_try;
        Serial.println("GPS requested [" + String(ack_gps_try) + "]." );
        gps_start_time = millis();
      }

    }

    if (data) {
      gps_start_time = millis();
      while (1)
      {
        if (ArdSerial.available()) {

          ArdSerial.readBytes(gps_data, 25);
          Serial.println(gps_data);
          String gpscode = getValue(gps_data, '$', 0);
          if (gpscode.equals("GP")) {
            String sLATITUDE = getValue(gps_data, '$', 1);
            String sLONGITUDE = getValue(gps_data, '$', 2);

            Serial.println(sLATITUDE);
            Serial.println(sLONGITUDE);

            LATITUDE = sLATITUDE.toFloat();
            LONGITUDE = sLONGITUDE.toFloat();

          } else if (gpscode.equals("00"))
          {
            //LATITUDE = -8.079025;
            //LONGITUDE = -79.122865;
            Serial.println("NO GPS SUCCESS.");
          }


          break;

        }
        else if (millis() - gps_start_time > 65000) {


          Serial.println("NO DATA REACHED. Sorry");
          //LATITUDE = 0;
          //LONGITUDE = 0;
          break;
        }



      }

    }








  }
  else Serial.println("No GPS request.");


}


//------------------------------------------------------------------

void get_environment_sensor() {
  /*
    //enviroment
    //raw
    float dry_bulb_temp;
    float barometric_pressure;
    float relative_humidity;
    float pressure_altitude;
    //constitutive
    float solar_radiation;
    float vapor_pressure;
    float wet_bulb_temp;
  */

  if ( WiFi.status() != WL_CONNECTED) starting_wifi();
  // -          ---------------------------------------------------------------------------
  // it is not the best position for checking an update but it works.
  if ( WiFi.status() == WL_CONNECTED) {
    LOCAL_UPDATE = 0;
    if ( Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_STATUS + "LOCAL_UPDATE")) LOCAL_UPDATE = firebasedata.intData();

    if (LOCAL_UPDATE) {
      beep.vbeep(200);
      delay(150);
      beep.vbeep(200);
      delay(150);
      beep.vbeep(200);
      setClock();
      Firebase.setInt(firebasedata,  DEVICE + PATH_CONFIGURATION_STATUS + "LOCAL_UPDATE", 0);
      if (!FirmwareCheck()) Firebase.setInt(firebasedata,  DEVICE + PATH_CONFIGURATION_STATUS + "LOCAL_UPDATE", 1);


    }
    else
    {
      Serial.println("No update check task performed. LOCAL_UPDATE 0. ");
    }
  }
  //--------------------------------------------------------------------------------------
  if (!Start_or_Restart) {


    Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_STATUS + "NewConf"); // if reset, newconf is 1 by default.
    NewConf_flag = firebasedata.intData();
  }

  read_OpenWeather();

  int8_t status_atm = 0;
  int8_t status_atm_aux = 0;
  int8_t rep_atm = 0;



  while (1) {
    digitalWrite(ATMOS_EN, HIGH );
    //aux
    delay(750);
    


    status_atm = bme_static.begin(0x76);
    

    status_atm_aux = bme_aux.begin(0x77);

    if(!status_atm_aux) {
      
      Serial.println("Could not find a valid [AUX] BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID [AUX] was: 0x");
      Serial.println(bme_aux.sensorID(), HEX);}

    else {
      
      Serial.print("[AUX] Temp: ");
      dry_bulb_temp_aux = bme_aux.readTemperature();
      Serial.println(dry_bulb_temp_aux);
      Serial.print("[AUX] Hum: ");
      relative_humidity_aux = bme_aux.readHumidity();
      Serial.println(relative_humidity_aux);
    }
    


    if (!status_atm) {
      Serial.println("Could not find a valid [ATM] BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID [ATM] was: 0x");
      Serial.println(bme_static.sensorID(), HEX);
      digitalWrite(ATMOS_EN, LOW);
      rep_atm = rep_atm + 1;
      delay(750);
    } else {
      /* Serial.print("SensorID was: 0x");
        Serial.println(bme_static.sensorID(), HEX); */

      dry_bulb_temp = bme_static.readTemperature();
      //Serial.println(dry_bulb_temp);
      if (dry_bulb_temp_mov >= 200 && dry_bulb_temp > 0) dry_bulb_temp_mov = dry_bulb_temp;
      //Serial.println(dry_bulb_temp_mov);
      if (dry_bulb_temp < 0) {
        dry_bulb_temp_mov = 21;
      }


      else {
        if (dry_bulb_temp_mov < 0) dry_bulb_temp_mov = dry_bulb_temp;
        else dry_bulb_temp_mov = (dry_bulb_temp_mov + dry_bulb_temp) / 2;
      }



      //Serial.println(dry_bulb_temp_mov);

      barometric_pressure = round(bme_static.readPressure()) / 1000;
      relative_humidity = bme_static.readHumidity();
      pressure_altitude = bme_static.readAltitude(SEALEVELPRESSURE_HPA);
      if (pressure_altitude < 0) pressure_altitude = 0;
      
      Serial.println("[ATM] dry_bulb_temp: " + String(dry_bulb_temp,2));
      Serial.println("[ATM] barometric_pressure: " + String(barometric_pressure,2));
      Serial.println("[ATM] relative_humidity: " + String(relative_humidity,2));
      Serial.println("[ATM] pressure_altitude: " + String(pressure_altitude));
      Serial.println("[ATM] dry_bulb_temp M.A.2: " + String(dry_bulb_temp_mov, 2));

      if (dry_bulb_temp > 0 && relative_humidity > 0 && barometric_pressure < 104) {
        if (relative_humidity == 100 && relative_humidity_past != 0 && (relative_humidity - relative_humidity_past > 25) ) {
          Serial.println("RH satured. Skip.");
          rep_atm = rep_atm + 1;
          no_atmos = 1;
          if (rep_atm > 15) {
            digitalWrite(ATMOS_EN, LOW);
            break;}
        }
        else {
          Serial.print("| Past RH: ");
          Serial.println(relative_humidity_past);
          relative_humidity_past = relative_humidity;
          digitalWrite(ATMOS_EN, LOW);
          no_atmos = 0;
          break;
        }
      }
      else {
        rep_atm = rep_atm + 1;
        no_atmos = 1;
        if (rep_atm > 15) {
          if (relative_humidity_past != 0) relative_humidity = relative_humidity_past;
          break;
        }
      }

    }

  }
}


//------------------------------------------------------------------



void get_neolink_status() {

  //SOLAR

  Serial.print("Solar voltage = ");

  Serial.println(solar_voltage);


  Serial.print("Battery voltage = ");
  Serial.println(battery_voltage);

  //Internal TEMP
  digitalWrite(TEMP_EN, HIGH);
  delay(100);

  sensors.begin();
  sensors.requestTemperatures(); 
  internal_temperature_raw=sensors.getTempCByIndex(0);
  Serial.print("Internal Temperature RAW = ");
  Serial.println(internal_temperature_raw);

 

  if (internal_temperature == 0) internal_temperature = internal_temperature_raw ;
  internal_temperature = (internal_temperature_raw  + internal_temperature) / 2;
  Serial.print("Internal Temperature Mean = ");
  Serial.println(internal_temperature);

  digitalWrite(TEMP_EN, LOW);


}

//------------------------------------------------------------------

void get_neonodes_signals() {

  /* SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST_LoRa, DIO0);
    if (!LoRa.begin(BAND, false))Serial.println("Starting LoRa failed!\r\n");
    else Serial.println("LoRa Failed"); */


}

//------------------------------------------------------------------

void send_cloud() {

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);



  neoFirebaseJson json_port1;
  neoFirebaseJson json_port2;
  neoFirebaseJson json_port3;
  neoFirebaseJson json_port4;
  neoFirebaseJson json_port_status;

  //---------------------------------------------------PORT 1--------------------------------------------------------------
  if (Port1_Active) {

    json_port_status.FirebaseJson::set( "Port1_Active" , port1_type);

    for (int i = 0; i < port1_nvar; i++) {
      json_port1.set( "/P1/V" + String(i + 1), double(port1_variables[i]), 2);


    }


    //json_port1.set( "/P1/Depth", DEPTH1);

    if (port1_type == "g") {
      json_port1.set( "/P1/VWC", double(P1_humidity_m3m3), 3);
      json_port1.set( "/P1/ApPer", double(P1_aparent_perm), 3);
      json_port1.set( "/P1/PorePer", double(P1_pore_perm), 3);
      json_port1.set( "/P1/PoreCE", double(P1_water_pore_cond), 3);

    }

    /* String buff_string2;
      json_port1.toString(buff_string2,true);
      Serial.println(buff_string2); */

  }
  else  json_port_status.FirebaseJson::set( "Port1_Active", "NaN");



  //-------------------------------------------------PORT 2----------------------------------------------------------------
  if (Port2_Active) {

    json_port_status.FirebaseJson::set("Port2_Active" , port2_type);

    for (int i = 0; i < port2_nvar; i++) {
      json_port2.set( "/P2/V" + String(i + 1), double(port2_variables[i]), 2);

    }
    //json_port2.set( "/P2/Depth", DEPTH2);

    if (port2_type == "g") {
      json_port2.set( "/P2/VWC", double(P2_humidity_m3m3), 3);
      json_port2.set( "/P2/ApPer", double(P2_aparent_perm), 3);
      json_port2.set( "/P2/PorePer", double(P2_pore_perm), 3);
      json_port2.set( "/P2/PoreCE", double(P2_water_pore_cond), 3);

      /* String buff_string;
        json_port2.toString(buff_string,true);
        Serial.println(buff_string); */

    }

  }
  else  json_port_status.FirebaseJson::set( "Port2_Active", "NaN");

  //-------------------------------------------------PORT 3----------------------------------------------------------------
  if (Port3_Active) {

    json_port_status.FirebaseJson::set(  "Port3_Active" , port3_type);

    for (int i = 0; i < port3_nvar; i++) {
      json_port3.set(  "/P3/V" + String(i + 1), double(port3_variables[i]), 2);

    }
    //json_port3.set( "/P3/Depth", DEPTH3);

    if (port3_type == "g") {
      json_port3.set( "/P3/VWC", double(P3_humidity_m3m3), 3);
      json_port3.set( "/P3/ApPer", double(P3_aparent_perm), 3);
      json_port3.set( "/P3/PorePer", double(P3_pore_perm), 3);
      json_port3.set( "/P3/PoreCE", double(P3_water_pore_cond), 3);

    }

  }
  else  json_port_status.FirebaseJson::set( "Port3_Active", "NaN");

  //-----------------------------------------------PORT 4------------------------------------------------------------------
  if (Port4_Active) {

    json_port_status.FirebaseJson::set("Port4_Active" , port4_type);

    for (int i = 0; i < port4_nvar; i++) {
      json_port4.set("/P4/V" + String(i + 1), double(port4_variables[i]), 2);

    }


    //json_port4.set("/P4/Depth", DEPTH4);

    if (port4_type == "g") {
      json_port4.set( "/P4/VWC", double(P4_humidity_m3m3), 3);
      json_port4.set( "/P4/ApPer", double(P4_aparent_perm), 3);
      json_port4.set( "/P4/PorePer", double(P4_pore_perm), 3);
      json_port4.set( "/P4/PoreCE", double(P4_water_pore_cond), 3);

    }

  }
  else  json_port_status.FirebaseJson::set("Port4_Active", "NaN");

  //----------------------------------------------------END PORT------------------------------------------------------------

  float temp_factor = 0.11;
  if (internal_temperature <= 21.5) temp_factor = 0;
  //dry_bulb_temp_mov=dry_bulb_temp_mov- temp_factor*(internal_temperature_voltage - dry_bulb_temp_mov);
  dry_bulb_temp_mov = dry_bulb_temp_mov - temp_factor * (internal_temperature - dry_bulb_temp_mov);

  

  //Json State
  neoFirebaseJson json_state;
  

  //sending weather values
  if (!no_atmos) {
    json_state.set("dT", double(dry_bulb_temp_mov), 2);
    json_state.set("dT_raw", double(dry_bulb_temp), 2);
    json_state.set("BP", double(barometric_pressure), 2);
    json_state.set("RH", double(relative_humidity), 2);
    json_state.set("RH_aux", double(relative_humidity_aux), 1);
    json_state.FirebaseJson::set("AL", int(pressure_altitude));
    json_state.set("BV", double(battery_voltage), 3);
    json_state.set("SV", double(solar_voltage), 2);
    json_state.set("iT", double(internal_temperature), 1);
    json_state.set("iT_raw", double(internal_temperature_raw), 1);
    json_state.set("iT_aux", double(dry_bulb_temp_aux), 1);
    json_state.set("WS", double(wind_speed), 2);
    json_state.FirebaseJson::set("WD", int(wind_deg));
    json_state.FirebaseJson::set("OP_TIME", int(millis() / 1000));

  }
  /*
    json_state.set("BV", double(battery_voltage), 3);
    json_state.set("SV", double(solar_voltage), 2);
    json_state.set("iT", double(internal_temperature), 1);
    json_state.set("WS", double(wind_speed), 2);
    json_state.set("WD", int(wind_deg));
    json_state.set("OP_TIME", int(millis() / 1000));
  */


  //sending gps values
  //Serial.println("GPS_CLOUD" + String(GPS_CLOUD));
  //Serial.println(LATITUDE);
  neoFirebaseJson json_gps;
  if (GPS_CLOUD && LATITUDE != -8.079025 && LONGITUDE != -79.122865 ) {
    json_gps.set("LAT", LATITUDE, 5);
    json_gps.set("LONG", LONGITUDE, 5);
  }

  //enabling wifi if enable
  if (WIFI_EN) {
    if (!wifi_conf_isSet) wifi_conf_isSet = setting_wifi();
    if ( WiFi.status() != WL_CONNECTED) starting_wifi();
  }




  struct tm timeinfo_2;

  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  int time_rep = 0;
  while ( (timeinfo.tm_year - 100 < 0) )
  {
    getLocalTime(&timeinfo);
    delay(100);
    time_rep ++;
    Serial.print("Time obtained error. Try: ");
    Serial.println(time_rep);
    if (time_rep > 4) deepsleep(1);
  }


  String _year = String(timeinfo.tm_year - 100);
  int8_t i_mon = timeinfo.tm_mon;
  int8_t i_day = timeinfo.tm_mday;
  int8_t i_hour = timeinfo.tm_hour;
  int8_t i_min = timeinfo.tm_min;
  int8_t i_secs = timeinfo.tm_sec;


  String real_month;
  if (i_mon < 9) real_month = "0" + String(i_mon + 1);
  else real_month = String(i_mon + 1);

  String real_day;
  if (i_day < 10) real_day = "0" + String(i_day);
  else real_day = String(i_day);

  String real_hour;
  if (i_hour < 10) real_hour = "0" + String(i_hour);
  else real_hour = String(i_hour);

  String real_min;
  if (i_min < 10) real_min = "0" + String(i_min);
  else real_min = String(i_min);


  String real_secs;
  if (i_secs < 10) real_secs = "0" + String(i_secs);
  else real_secs = String(i_secs);


  actual_hour = i_hour;
  actual_min = i_min;
  actual_secs = i_secs;



  String auxiliar_string = _year + "/" + real_month + "/" + real_day + " " + real_hour + ":" + real_min + ":" + real_secs;

  String _hour;

  check_configuration();
  
  boolean p1_err = Port1_Active != SENSOR_P1 ;
  boolean p2_err = Port2_Active != SENSOR_P2 ;
  boolean p3_err = Port3_Active != SENSOR_P3 ;
  boolean p4_err = Port4_Active != SENSOR_P4 ;

 
  /*
  Serial.println(p1_err);
  Serial.println(p2_err);
  Serial.println(p3_err);
  Serial.println(p4_err); */

  boolean p1_err_c = p1_err ;
  boolean p2_err_c = p2_err ;
  boolean p3_err_c = p3_err ;
  boolean p4_err_c = p4_err ;

if ( Port1_Active )  p1_err_c = 0;
if ( Port2_Active )  p2_err_c = 0;
if ( Port3_Active )  p3_err_c = 0;
if ( Port4_Active )  p4_err_c = 0;


  

  /*
  Serial.println(p1_err_c);
  Serial.println(p2_err_c);
  Serial.println(p3_err_c);
  Serial.println(p4_err_c); */


  if( p1_err_c || p2_err_c ||  p3_err_c || p4_err_c) {
    Serial.println("FORCED RESET.");
    FORCED_RESET_TASK();
    
  }

  boolean p1_depth_err = 0;
  boolean p2_depth_err = 0;
  boolean p3_depth_err = 0;
  boolean p4_depth_err = 0;

   // define sensors implies depth 
  if (SENSOR_P1) p1_depth_err = DEPTH1 == 0;
  if (SENSOR_P2) p2_depth_err = DEPTH2 == 0;
  if (SENSOR_P3) p3_depth_err = DEPTH3 == 0;
  if (SENSOR_P4) p4_depth_err = DEPTH4 == 0;
  

  if (p1_depth_err || p2_depth_err || p3_depth_err || p4_depth_err) { 

    Serial.println("depth request failed.");
    depth_request(); 
    if (SENSOR_P1) p1_depth_err = DEPTH1 == 0;
    if (SENSOR_P2) p2_depth_err = DEPTH2 == 0;
    if (SENSOR_P3) p3_depth_err = DEPTH3 == 0;
    if (SENSOR_P4) p4_depth_err = DEPTH4 == 0;
    if (p1_depth_err || p2_depth_err || p3_depth_err || p4_depth_err) deepsleep(1);
    Serial.println();
  }
 

  int qmin;

  if (i_hour > N_END_HOUR &&  i_hour < N_START_HOUR) {
    qmin = SLEEP_TIME / 60;
  }


  else qmin = (N_SLEEP_TIME / 60);

  i_min = ((i_min + 1) / qmin) * qmin;

  //Serial.print("i_min");
  //Serial.println(i_min);


  if (i_min >= 60) {
    i_min = 0;

    i_hour = i_hour + 1;
    if (i_hour >= 24) {
      i_hour = 0;
      _hour = "0" + String(i_hour);
      i_day = i_day + 1;


      if (i_day > 28) {

        configTime(gmtOffset_sec + 300, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
        getLocalTime(&timeinfo_2);
        time_rep = 0;
        while ( (timeinfo_2.tm_year - 100 < 0) )
        {
          getLocalTime(&timeinfo_2);
          delay(100);
          time_rep ++;
          Serial.print("Time obtained error. Try: ");
          Serial.println(time_rep);
          if (time_rep > 4) deepsleep(1);
        }


        Serial.println(&timeinfo_2, "%A, %B %d %Y %H:%M:%S");


        i_day = timeinfo_2.tm_mday;

        i_mon = timeinfo_2.tm_mon;

        _year = String(timeinfo_2.tm_year - 100);

        i_secs = timeinfo_2.tm_sec;


      }

    }
  }

  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");


  String _month;
  if (i_mon < 9) _month = "0" + String(i_mon + 1);
  else _month = String(i_mon + 1);

  String _day;
  if (i_day < 10) _day = "0" + String(i_day);
  else _day = String(i_day);


  if (i_hour < 10) _hour = "0" + String(i_hour);
  else _hour = String(i_hour);

  String _min;
  if (i_min < 10) _min = "0" + String(i_min);
  else _min = String(i_min);


  String _secs;
  if (i_secs < 10) _secs = "0" + String(i_secs);
  else _secs = String(i_secs);
/*
  FirebaseJson json_observer;

  json_observer.set("/P1_msg", port1_msg);
  json_observer.set("/P2_msg", port2_msg);
  json_observer.set("/P3_msg", port3_msg);
  json_observer.set("/P4_msg", port4_msg);

  String buff_string_obserever;
  json_observer.toString(buff_string_obserever,true);
  Serial.println(buff_string_obserever); 

  */

  if (Port1_Active) json_port1.FirebaseJson::set("/P1/Depth", DEPTH1);
  if (Port2_Active) json_port2.FirebaseJson::set("/P2/Depth", DEPTH2);
  if (Port3_Active) json_port3.FirebaseJson::set("/P3/Depth", DEPTH3);
  if (Port4_Active) json_port4.FirebaseJson::set("/P4/Depth", DEPTH4);

  String timestamp_string2 = _year + "/" + _month + "/" + _day + " " + _hour + ":" + _min;
  Serial.println("Firebase timestamp: " + timestamp_string2);
  String timestamp_string = "/" + _year + "/" + _month + "/" + _day + "/" + _hour + "/" + _min;


  //sending Firebase
  if (PORT_RQ) {
    Firebase.updateNodeSilent(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "Port/" , json_port_status);
    if (Port1_Active){ Firebase.updateNodeSilent(firebasedata, DEVICE +  PATH_DATA  + port1_type + timestamp_string , json_port1);
    Serial.println("Send P1.");
    delay(150);}

    if (Port2_Active) {Firebase.updateNodeSilent(firebasedata, DEVICE +  PATH_DATA  + port2_type + timestamp_string , json_port2);
    Serial.println("Send P2.");
    delay(150);}

    if (Port3_Active){Firebase.updateNodeSilent(firebasedata, DEVICE +  PATH_DATA  + port3_type + timestamp_string , json_port3);
    Serial.println("Send P3.");
    delay(150);}

    if (Port4_Active) {Firebase.updateNodeSilent(firebasedata, DEVICE +  PATH_DATA  + port4_type + timestamp_string , json_port4);
    Serial.println("Send P4.");
    delay(150);}
  }
  
  if (GPS_CLOUD) Firebase.updateNodeSilent(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS  + "GPS" , json_gps);

  delay(150);
  Firebase.updateNodeSilent(firebasedata, DEVICE + PATH_DATA  + "/State" + timestamp_string, json_state);
  double LAST_OP_TIME = double(millis() / 1000);
  Firebase.setDouble(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "LastOP_TIME", LAST_OP_TIME );


  Firebase.setString(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "WiFi_rssi", String(WiFi.RSSI()));


  //Serial.print("LastUpload: ");
  //Serial.println(auxiliar_string);

  Firebase.setString(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "LastUpload", auxiliar_string );

  delay(150);

  //Firebase.updateNodeSilent(firebasedata, DEVICE + "/Observer" + timestamp_string, json_observer);

  


  Firebase.setString(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "FireTimeStamp", timestamp_string2 );
  

  if (Start_or_Restart) {
    Firebase.setString(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "LastRestart", auxiliar_string);
    Start_or_Restart = 0;
    Firebase.setString(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "Firmware", FIRMWARE_VER);
    Firebase.setString(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "WifiSSID", WIFI_SSID_DEFAULT);
  }

  int real_sleep;
  if ( !MODE_PRG) {
    Firebase.setString(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "IP", "Nope");
    real_sleep = real_sleep_time();
    Serial.print("Real Sleep Time = ");
    Serial.println(real_sleep);
    Firebase.setInt(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "real_sleep", real_sleep);

    Firebase.setInt(firebasedata,  DEVICE + PATH_CONFIGURATION_STATUS + "NewConf", 0);

    /*
      if (GPS_CLOUD) {
      //Serial.println("GPS_RQ a cero");
      Firebase.setInt(firebasedata, DEVICE + PATH_CONFIGURATION_VALUES + "GPS_RQ", 0 );
      }
    */





    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    turn_modem_off();


  }
  else Firebase.setString(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "IP", String( WiFi.localIP().toString()));

  beep.vbeep(200);
  if (BEEP_EN) beep.vbeep(150);

  if (WiFi.status() != WL_CONNECTED) Serial.println("Wi-Fi Disconnected");
  else  Serial.println("Wi-Fi Still Connected\n");

  Serial.println("OP_TIME_2: " + String(int(millis() / 1000)));

  if (!MODE_PRG) deepsleep(real_sleep);
  //start_server();

  if (BEEP_EN) {
    beep.vbeep(100);
    delay(40);
    beep.vbeep(100);
  }

}

void turn_modem_off() {
  Serial.println("Turning Modem OFF");
  digitalWrite(SIM_ON, HIGH);
  delay(750);
  digitalWrite(SIM_ON, LOW);
  delay(2000);

}

void turn_modem_on() {
  Serial.println("Turning Modem ON");
  digitalWrite(SIM_ON, HIGH);
  delay(1500);
  digitalWrite(SIM_ON, LOW);
  delay(2000);


}



//------------------------------------------------------------------
void starting_wifi() {
  int wifi_try = 1;
  if (wifi_conf_isSet) {

    //Serial.println(WIFI_SSID);
    //Serial.println(WIFI_PSSWD);

    WiFi.begin(WIFI_SSID, WIFI_PSSWD);
    Serial.print("Conectando WiFi..");
    init_timestamp = millis();
    while (1) {
      while (WiFi.status() != WL_CONNECTED && millis() - init_timestamp < WIFI_TIME_LIMIT ) {
        Serial.print(".");
        delay(500);
      }
      if (WiFi.status() == WL_CONNECTED) {
        //Serial.println(millis() - init_timestamp);
        Serial.print("\n IP: ");
        Serial.println(WiFi.localIP());
        //delay(500);
        Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
        Firebase.reconnectWiFi(true);
        //delay(1000);
        break;

      }
      else {

        init_timestamp = millis();
        WiFi.begin(WIFI_SSID, WIFI_PSSWD);
        Serial.println("\n Enabling wifi: ERROR. Try: " + String(wifi_try));
        wifi_try++;

        if (wifi_try > 3) {

          Serial.println("Nothing to do. See ya.. ");
          prg_iteration = 0;

          /* beep.vbeep(250);
            delay(20);
            beep.vbeep(25);
            delay(2);
            beep.vbeep(25);
            delay(2);
            beep.vbeep(25);
            delay(2);
            beep.vbeep(25); */
          //deepsleep(NO_WIFI);
          deepsleep(1);
        }

        else {




        }

      }
    }

  }

}





/*

  void start_server() {
  //use mdns for host name resolution
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");


  }

  Serial.println("mDNS responder started");
  //return index page which is stored in serverIndex

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });

  //handling uploading firmware file
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.on("/quickRestart", handle_quickRestart);
  server.begin();

  server.on("/RestartNoProg", handle_RestartNoProg);
  server.begin();



  }

  void handle_quickRestart() {

  server.send(200, "text/html", "<script>window.open('/','_self')</script>");

  deepsleep(1);
  }

  void handle_RestartNoProg() {
  Serial.println("MODE_PRG finished.");
  server.send(200, "text/html", "<script>window.open('/','_self')</script>");
  MODE_PRG = false;
  Firebase.setBool(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "MODE_PRG", MODE_PRG);

  deepsleep(1);
  }

*/

int16_t real_sleep_time() {
  int16_t sleep_timee;
  //Serial.println(N_END_HOUR);
  if (actual_hour > N_END_HOUR && actual_hour < N_START_HOUR) {
    sleep_timee = int(SLEEP_TIME) - (actual_min * 60 + actual_secs) % int(SLEEP_TIME) - (millis()) / 1000;
    if (sleep_timee <= 0) sleep_timee = sleep_timee + int(SLEEP_TIME);
  } else
  {
    Serial.println("N_SLEEP_TIME set.");
    sleep_timee = int(N_SLEEP_TIME) - (actual_min * 60 + actual_secs) % int(N_SLEEP_TIME) - (millis()) / 1000;
    if (sleep_timee <= 0) sleep_timee = sleep_timee + int(N_SLEEP_TIME);
  }


  //Serial.print ("Modem wakeup = ");
  SLEEP_TIME_modem = sleep_timee - SLEEP_TIME_PRE + 10;
  if (SLEEP_TIME_modem < 1) SLEEP_TIME_modem = 10; //+10

  //Serial.println (SLEEP_TIME_modem);

  Serial.print ("NeoLink wakeup = ");
  Serial.println (SLEEP_TIME_PRE);

  if (prg_iteration == 0) {
    prg_iteration = 9;
    return  SLEEP_TIME_PRE + SLEEP_TIME_modem + 10 ; //+10
  }
  else if (prg_iteration == 9)
  {
    prg_iteration = 0;
    return  SLEEP_TIME_modem;
  }



  // return sleep_timee + 10;

}


double ReadVoltage(byte pin) {

  //analogReadResolution(12);
  double reading = analogRead(pin);
  //return -0.000000000009824 * pow(reading, 3) + 0.000000016557283 * pow(reading, 2) + 0.000854596860691 * reading + 0.065440348345433;
  return reading;

}

void NodeExistance() {
  if ( WiFi.status() != WL_CONNECTED) starting_wifi();
  if ( WiFi.status() == WL_CONNECTED) {


    bool node_existance1 = false;
    bool node_existance2 = false;
    bool node_existance3 = false;
    bool temp_recipe;
    node_existance1 = Firebase.getInt(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "NewConf"); // if reset, newconf is 1 by default.
    //node_existance2 = Firebase.getBool(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "MODE_PRG", temp_recipe, 0);
    node_existance2 = Firebase.getInt(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + "LOCAL_UPDATE");
    node_existance3 = Firebase.getInt(firebasedata, DEVICE + PATH_CONFIGURATION_VALUES + "PORT_RQ");


    if (node_existance1) Serial.println("node_existance1 true.");
    else {
      Serial.println("node_existance1 error.");
    }
    if (node_existance2) Serial.println("node_existance2 true.");
    else {
      Serial.println("node_existance2 error.");
    }
    if (node_existance3) Serial.println("node_existance3 true.");
    else {
      Serial.println("node_existance3 error.");
    }

    if (!node_existance1 && !node_existance2 && !node_existance3) {
      Serial.println("Creating Config Nodes..");

      /*FirebaseJson json_null_gps;
        json_null_gps.set("LAT", double(0));
        json_null_gps.set("LONG", double(0));
        Firebase.updateNodeSilent(firebasedata,  DEVICE +  PATH_DATA  + "/State/GPS" , json_null_gps);*/


      FirebaseJson json_default_nodes;
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Firmware", FIRMWARE_VER);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "GPS/LAT", LATITUDE);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "GPS/LONG", LONGITUDE);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Hardware_ver", HARDWARE_VER);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "IP", "None");
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "LastOP_TIME", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "LastUpload", "NaN");

      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V2/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V3/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V3/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V3/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V2/bool", 0);

      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V2/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V3/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V3/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V3/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V2/bool", 0);

      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V2/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V3/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V3/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V3/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V2/bool", 0);

      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V2/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V3/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V3/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V3/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V2/bool", 0);
      //json_default_nodes.set(PATH_CONFIGURATION_STATUS + "MODE_PRG", false);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "NewConf", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "LOCAL_UPDATE", 1);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Port/Port1_Active", "NaN");
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Port/Port2_Active", "NaN");
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Port/Port3_Active", "NaN");
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Port/Port4_Active", "NaN");
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "WiFi_rssi", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "WifiSSID", WIFI_SSID_DEFAULT);

      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "BAT_L", double(BAT_L));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "BAT_H", double(BAT_H));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "BEEP_EN", int(BEEP_EN));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "GPS_RQ", int(GPS_RQ));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "NO_WIFI", double(NO_WIFI));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "PORT_RQ", int(PORT_RQ));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "SLEEP_TIME", double (SLEEP_TIME));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "N_SLEEP_TIME", double (N_SLEEP_TIME));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "N_START_HOUR", double (N_START_HOUR));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "N_END_HOUR", double (N_END_HOUR));

      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "OBSERVER", 0);


      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "DEPTH/P1", 0);
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "DEPTH/P2", 0);
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "DEPTH/P3", 0);
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "DEPTH/P4", 0);

      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "SENSOR/P1", 0);
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "SENSOR/P2", 0);
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "SENSOR/P3", 0);
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "SENSOR/P4", 0);


      //json_default_nodes.set(PATH_CONFIGURATION_VALUES + "WIFI_EN", int(WIFI_EN));
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "WIFI_SSID", WIFI_SSID_DEFAULT);
      json_default_nodes.set(PATH_CONFIGURATION_VALUES + "WIFI_PSSWD", WIFI_PSSWD_DEFAULT);



      if (Firebase.updateNode(firebasedata, DEVICE, json_default_nodes)) Serial.println("Creating Config Nodes Succeded..");
      else
      {
        Serial.println("Creating Config Nodes Failed");
        //code error here
      }

    }
    else Serial.println("Nodes already exist.");

  }

}

/*
  float wet_temp(float dry_temp, float p_atm) {
  float aw, bw, a, b, q_vs, e_s;
  //  Fórmula lineal para determinar la temperatura del bulbo
  //    húmedo
  //    Linear formula for determining the temperature of the wet bulb
  //    Mario Carnesoltas-Calvo
  //
  //    q_vs mezcla de vapor saturado

  if (dry_temp >= 0) {
    aw = 17.270;
    bw = 35.5;
    a = 7.5;
    b = 237.3;
  } else {
    aw = 21.875;
    bw = 7.5;
    a = 9.5;
    b = 265.5;
  }
  q_vs = 3800 * exp(aw * (dry_tem - 0.009) / (dry_temp + 237.65)) / p_atm;
  e_s = q_vs * p_atm / (0.62197 + q_vs);


  }*/

String httpGETRequest(const char* serverName) {
  HTTPClient http;

  // Your IP address with path or Domain name with URL path
  http.begin(serverName);

  // Send HTTP POST request
  int httpResponseCode = -1;
  int8_t httpNoResponse_try = 1;

  String payload = "{}";

  while (httpResponseCode < 0) {
    httpResponseCode = http.GET();
    if (httpResponseCode > 0) {
      /* Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode); */
      payload = http.getString();
    }
    else {
      Serial.println("--------------");
      Serial.print("Error. Code: ");
      Serial.println(httpResponseCode);
      Serial.println("--------------");
      delay(200);
      if (httpNoResponse_try >= 2) break;
      httpNoResponse_try++;
      Serial.print("Get RQ try = ");
      Serial.println(httpNoResponse_try);

    }
  }
  // Free resources
  http.end();

  return payload;
}

void transform_variables() {

  const float perm_aparent_water = 4.1;

  if (Port1_Active) {
    if (port1_type == "g") {
      //EC uS/cm . 10e3 uS/cm = 1dS/m
      //MINERAL SOILS
      //port1_variables[0] = 2400;
      //port1_variables[2] = 800;
      P1_humidity_m3m3 = 3.879 * port1_variables[0] / 10000 - 0.6956;
      if (P1_humidity_m3m3 < 0) P1_humidity_m3m3 = 0;
      //APPARENT DIELECTRIC PERMITTIVITY
      P1_aparent_perm = pow((2.887e-9 * pow(port1_variables[0], 3) - 2.08e-5 * pow(port1_variables[0], 2) + 5.276e-2 * port1_variables[0] - 43.39), 2);
      //Real portion of Dielectric permittivity of the soil pore water
      P1_pore_perm = 80.3 - 0.37 * (port1_variables[1] - 20);
      //pore water EC (dS/m)
      if (P1_humidity_m3m3 >= 0.05) P1_water_pore_cond = (P1_pore_perm * port1_variables[2] / (P1_aparent_perm - perm_aparent_water)) * 10e-3;
      else P1_water_pore_cond = 0;
      if (P1_water_pore_cond < 0) P1_water_pore_cond = 0 ;

      //Serial.println("RAW: "+ String(port1_variables[0],2));
      Serial.println("P1_humidity_m3m3: " + String(P1_humidity_m3m3, 4));

    }
  }

  if (Port2_Active) {
    if (port2_type == "g") {

//EC uS/cm . 10e3 uS/cm = 1dS/m
      //MINERAL SOILS
      //port1_variables[0] = 2400;
      //port1_variables[2] = 800;
      P2_humidity_m3m3 = 3.879 * port2_variables[0] / 10000 - 0.6956;
      if (P2_humidity_m3m3 < 0) P2_humidity_m3m3 = 0;
      //APPARENT DIELECTRIC PERMITTIVITY
      P2_aparent_perm = pow((2.887e-9 * pow(port2_variables[0], 3) - 2.08e-5 * pow(port2_variables[0], 2) + 5.276e-2 * port2_variables[0] - 43.39), 2);
      //Real portion of Dielectric permittivity of the soil pore water
      P2_pore_perm = 80.3 - 0.37 * (port2_variables[1] - 20);
      //pore water EC (dS/m)
      if (P2_humidity_m3m3 >= 0.05) P2_water_pore_cond = (P2_pore_perm * port2_variables[2] / (P2_aparent_perm - perm_aparent_water)) * 10e-3;
      else P2_water_pore_cond = 0;
      if (P2_water_pore_cond < 0) P2_water_pore_cond = 0 ;

      //Serial.println("RAW: "+ String(port1_variables[0],2));
      Serial.println("P2_humidity_m3m3: " + String(P2_humidity_m3m3, 4));

    }
  }

  if (Port3_Active) {
    if (port3_type == "g") {


      //EC uS/cm . 10e3 uS/cm = 1dS/m
      //MINERAL SOILS
      //port1_variables[0] = 2400;
      //port1_variables[2] = 800;
      P3_humidity_m3m3 = 3.879 * port3_variables[0] / 10000 - 0.6956;
      if (P3_humidity_m3m3 < 0) P3_humidity_m3m3 = 0;
      //APPARENT DIELECTRIC PERMITTIVITY
      P3_aparent_perm = pow((2.887e-9 * pow(port3_variables[0], 3) - 2.08e-5 * pow(port3_variables[0], 2) + 5.276e-2 * port3_variables[0] - 43.39), 2);
      //Real portion of Dielectric permittivity of the soil pore water
      P3_pore_perm = 80.3 - 0.37 * (port3_variables[1] - 20);
      //pore water EC (dS/m)
      if (P3_humidity_m3m3 >= 0.05) P3_water_pore_cond = (P3_pore_perm * port3_variables[2] / (P3_aparent_perm - perm_aparent_water)) * 10e-3;
      else P3_water_pore_cond = 0;
      if (P3_water_pore_cond < 0) P3_water_pore_cond = 0 ;

      //Serial.println("RAW: "+ String(port1_variables[0],2));
      Serial.println("P3_humidity_m3m3: " + String(P3_humidity_m3m3, 4));

    }
  }

  if (Port4_Active) {
    if (port4_type == "g") {

      //EC uS/cm . 10e3 uS/cm = 1dS/m
      //MINERAL SOILS
      //port1_variables[0] = 2400;
      //port1_variables[2] = 800;
      P4_humidity_m3m3 = 3.879 * port4_variables[0] / 10000 - 0.6956;
      if (P4_humidity_m3m3 < 0) P4_humidity_m3m3 = 0;
      //APPARENT DIELECTRIC PERMITTIVITY
      P4_aparent_perm = pow((2.887e-9 * pow(port4_variables[0], 3) - 2.08e-5 * pow(port4_variables[0], 2) + 5.276e-2 * port4_variables[0] - 43.39), 2);
      //Real portion of Dielectric permittivity of the soil pore water
      P4_pore_perm = 80.3 - 0.37 * (port4_variables[1] - 20);
      //pore water EC (dS/m)
      if (P4_humidity_m3m3 >= 0.05) P4_water_pore_cond = (P4_pore_perm * port4_variables[2] / (P4_aparent_perm - perm_aparent_water)) * 10e-3;
      else P4_water_pore_cond = 0;
      if (P4_water_pore_cond < 0) P4_water_pore_cond = 0 ;

      //Serial.println("RAW: "+ String(port1_variables[0],2));
      Serial.println("P4_humidity_m3m3: " + String(P4_humidity_m3m3, 4));

    }
  }





}

void read_OpenWeather() {
  if (WiFi.status() == WL_CONNECTED) {
    String WEATHER_SERVER_PATH_3 = "lat=" + String(LATITUDE, 5) + "&lon=" + String(LONGITUDE, 5);
    String WEATHER_SERVER_PATH = WEATHER_SERVER_PATH_1 + WEATHER_SERVER_PATH_3 + WEATHER_SERVER_PATH_2;
    //Serial.println(WEATHER_SERVER_PATH);
    jsonWeatherBuffer = httpGETRequest(WEATHER_SERVER_PATH.c_str());
    JSONVar OpenWeatherObj = JSON.parse(jsonWeatherBuffer);

    if (JSON.typeof(OpenWeatherObj) == "undefined") {
      Serial.println("Parsing input failed!");
      return;
    }

    //Serial.println(JSON.typeof(OpenWeatherObj));

    wind_speed = double(OpenWeatherObj["current"]["wind_speed"]);
    wind_deg = int(OpenWeatherObj["current"]["wind_deg"]);

    Serial.print("Wind Speed: ");
    Serial.println(wind_speed);
    Serial.print("Wind Direction: ");
    Serial.println(wind_deg);



  }
  else Serial.println("WiFi Disconnected. No OpenWeather API requested.");


}


int FirmwareCheck() {
  Serial.println("--------------------------------");
  Serial.println("Looking for a new firmware...");

  HTTPClient http;
  http.begin(UPDATE_JSON_URL);
  //http.begin(UPDATE_JSON_URL, ca);
  int httpCode = http.GET();

  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    Serial.println("[HTTP] GET... code: " + String( httpCode));

    if (httpCode == HTTP_CODE_OK) {
      Serial.println("[HTTP] CODE OK");


      String payload = http.getString();
      //Serial.println(payload);
      payload.toCharArray(rcv_buffer, payload.length() + 1);

      cJSON *json = cJSON_Parse(rcv_buffer);
      if (json == NULL) printf("downloaded file is not a valid json, aborting...\n");
      else {
        Serial.println("[HTTP] JSON OK");

        cJSON *device = cJSON_GetObjectItemCaseSensitive(json, HARDWARE_VER);

        if (device == NULL) {
          printf("There is no any FW for this HW version. Aborting...\n");
          http.end();
          return 1; //should be 0?
          }

        cJSON *firmware_version = cJSON_GetObjectItemCaseSensitive(device, "firmware_ver");
        cJSON *built = cJSON_GetObjectItemCaseSensitive(device, "built");
        cJSON *chk = cJSON_GetObjectItemCaseSensitive(device, "chk");
        cJSON *file = cJSON_GetObjectItemCaseSensitive(device, "file");

        String new_firmware_ver = firmware_version->valuestring;

        //Major.Minor.patch
        //Major. different devices
        //Minor. new functionalities
        //Patch. bugs
        // built = 19*Major + 17*Minor + 13 * Patch
        // chk = 3* Major + 5* Minor + 7 * Patch
        // just to prevent.. hacking. . .

        http.end();

        if (verify_json ( new_firmware_ver, FIRMWARE_VER, built->valueint, chk->valueint)) update_firmware(file->valuestring);
        else Serial.println("No firmware upgrade is necessary");


      }

    }

  }
  else {
    Serial.println("[HTTP] GET... failed, error: " +  http.errorToString(httpCode));
    return 0;
    //ESP.restart();


  }

  Serial.println("--------------------------------");

}


bool verify_json ( String new_firmware_ver, String current_firmware_ver, int built, int chk) {
  Serial.print ("checking firmware.. ");
  bool flag = false;
  String new_major = getValue(new_firmware_ver, '.', 0);
  String new_minor = getValue(new_firmware_ver, '.', 1);
  String new_patch = getValue(new_firmware_ver, '.', 2);

  int new_built = 19 * new_major.toInt() + 17 * new_minor.toInt() + 13 * new_patch.toInt();
  int new_chk = 3 * new_major.toInt() + 5 * new_minor.toInt() + 7 * new_patch.toInt();


  if (built == new_built && chk == new_chk) {
    Serial.println ("checked");
    String current_major = getValue(current_firmware_ver, '.', 0);
    String current_minor = getValue(current_firmware_ver, '.', 1);
    String current_patch = getValue(current_firmware_ver, '.', 2);

    if (current_major < new_major || current_minor < new_minor || current_patch < new_patch) {
      flag = true; // should upgrade

      Serial.print ("Current Firmware: ");
      Serial.println(FIRMWARE_VER);
      Serial.print ("Higher Firmware found: ");
      Serial.println(new_firmware_ver);



    }
    else  Serial.println ("Lower or equal Firmware found (" + new_firmware_ver + ")");


  } else {
    Serial.println ("firmware with wrong parameters");
  }



  return flag;
}


void update_firmware( String file) {
  Serial.println ("Starting firmware upgrade. Downloading.. ");
  Serial.println(file);
  delay(2000);

  WiFiClientSecure client;
  client.setCACert(ca);
  t_httpUpdate_return ret = httpUpdate.update(client, file);
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }


}



String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}




void setClock() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");  // UTC

  Serial.print(F("Waiting for NTP time sync: "));
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    yield();
    delay(500);
    Serial.print(F("."));
    now = time(nullptr);
  }

  Serial.println(F(""));
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print(F("Current time: "));
  Serial.print(asctime(&timeinfo));
}



void depth_request() {

  Serial.print("\n Requesting Depths:");

  Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "DEPTH/P1");
  DEPTH1 = firebasedata.intData();
  Serial.print("\n DEPTH1: " + String(DEPTH1));
  delay(100);

  Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "DEPTH/P2");
  DEPTH2 = firebasedata.intData();
  Serial.print("\n DEPTH2: " + String(DEPTH2));
  delay(100);

  Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "DEPTH/P3");
  DEPTH3 = firebasedata.intData();
  Serial.print("\n DEPTH3: " + String(DEPTH3));
  delay(100);

  Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + "DEPTH/P4");
  DEPTH4 = firebasedata.intData();
  Serial.print("\n DEPTH4: " + String(DEPTH4));
  delay(100);




}



void FORCED_RESET_TASK(){

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    turn_modem_off();


    unsigned long task_start_time;
    int i=0;
    //1. Reset Atmega328
    digitalWrite(FORCED_ATMEGA_RESET, HIGH);
    delay(500);
    digitalWrite(FORCED_ATMEGA_RESET, LOW);
    delay(2000);

    //2. Reset ESP32
    
    ArdSerial.read();
    task_start_time = millis();

    
    

    while (ArdSerial.read()!= '@' && millis()-task_start_time < 2000 )
    {
      ArdSerial.write('e'); 
      ++i;
      delay(200);
    }

    //Serial.println(i);    
    
    Serial.println("resetting ESP32.");

    delay(1000);
}

void deepsleep(int time2sleep) {
  esp_sleep_enable_timer_wakeup(time2sleep * uS_TO_S_FACTOR);
  Serial.println("Going to sleep for " + String(time2sleep) + " Seconds");
  Serial.println();
  //LoRa.end();
  //LoRa.sleep();
  delay(100);
  esp_deep_sleep_start();

}

void deepsleep_beep(int deepsleep_time, unsigned long beep_time){
  Serial.println("Resetting.. ");
  beep.vbeep(beep_time);
  deepsleep(deepsleep_time);
  
}

void deepsleep_several_beep(int deepsleep_time, unsigned long beep_time, int times, int between){
  
  for(int i=0; i<= times; i++){
    beep.vbeep(beep_time);
    delay(between);
  }
 
  deepsleep(deepsleep_time);
  
}

void compatibility_error(){
  Serial.println("NOT ABLE DUE COMPATIBILITY. ");
  deepsleep_several_beep(120, 500, 3, 1000); // seg, ms, times, ms

}
void config_LoRa(){


  Heltec.begin(false /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/,
               false /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  LoRa.setTxPower(20,RF_PACONFIG_PASELECT_PABOOST);
  /*
  //register the receive callback.
  LoRa.onReceive(onReceive);
  //put the radio into receive mode.
  LoRa.receive();
  */

}

void sendMessage(String outgoing)
{
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add Neolink address
  LoRa.write(localAddress);             // add Neonode address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add message length
  LoRa.print(outgoing);                 // add message
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

String onReceive(int packetSize)
{

  if (packetSize == 0) return incoming;          // if there's no packet, return
  
  incoming="";
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length



  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
  {   // check length for error
    Serial.println("error: message length does not match length");
    return "";                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return "";                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  return incoming;
}




void Transform_NN_variables(String message){
dry_bulb_temp_mov_nn=getValue(message,':',1).toFloat();
dry_bulb_temp_nn=getValue(message,':',2).toFloat();
barometric_pressure_nn=getValue(message,':',3).toFloat();
relative_humidity_nn=getValue(message,':',4).toFloat();
relative_humidity_aux_nn=getValue(message,':',5).toFloat();
pressure_altitude_nn=getValue(message,':',6).toFloat();
battery_voltage_nn=getValue(message,':',7).toFloat();
solar_voltage_nn=getValue(message,':',8).toFloat();
internal_temperature_nn=getValue(message,':',9).toFloat();
internal_temperature_raw_nn=getValue(message,':',10).toFloat();
dry_bulb_temp_aux_nn=getValue(message,':',11).toFloat();

P1_humidity_m3m3_nn=getValue(message,':',12).toFloat();
P1_aparent_perm_nn=getValue(message,':',13).toFloat();
P1_pore_perm_nn=getValue(message,':',14).toFloat();
P1_water_pore_cond_nn=getValue(message,':',15).toFloat();

P2_humidity_m3m3_nn=getValue(message,':',16).toFloat();
P2_aparent_perm_nn=getValue(message,':',17).toFloat();
P2_pore_perm_nn=getValue(message,':',18).toFloat();
P2_water_pore_cond_nn=getValue(message,':',19).toFloat();

P3_humidity_m3m3_nn=getValue(message,':',20).toFloat();
P3_aparent_perm_nn=getValue(message,':',21).toFloat();
P3_pore_perm_nn=getValue(message,':',22).toFloat();
P3_water_pore_cond_nn=getValue(message,':',23).toFloat();

P4_humidity_m3m3_nn=getValue(message,':',24).toFloat();
P4_aparent_perm_nn=getValue(message,':',25).toFloat();
P4_pore_perm_nn=getValue(message,':',26).toFloat();
P4_water_pore_cond_nn=getValue(message,':',27).toFloat();

} 
String reciveBig(){
  long start=millis();
  String msg;
  String msg_prev;
  String message;
  long start_time;
  int packetsize=0;
  bool control=true;
  bool flag;

  while(control){
    if(Comparator_msg(msg,msg_prev)){
    message=message+msg;
    Serial.println(message);
    }
    msg_prev=msg;
    Serial.println(message);
    while(packetsize==0){
      packetsize=LoRa.parsePacket();
    }
    msg=onReceive(packetsize);
    Serial.println(msg);
    control=Comparator(msg);
    packetsize=0;
  }
 Serial.println(message);
 return message;
}
bool Comparator_msg(String msg1,String msg2){
  int length1=msg1.length();
  int length2=msg2.length();
  char msga[length1];
  char msgb[length2];
  msg1.toCharArray(msga,length1+1);
  msg2.toCharArray(msgb,length2+1);
  if(strcmp(msga,msgb)==0){
    Serial.print("iguales");
    return false;
  } else {
    Serial.println("diferentes");
    return true;
  }
}
bool Comparator(String msg){
  char compare[]="FIN";
  int length=msg.length();
  char msg2[length];
  msg.toCharArray(msg2,length+1);
  if(strcmp(msg2,compare)==0){
    return false;
  }else 
  {
    return true;
  }
}
void LoRa_Communication(){
    long start=millis();
  config_LoRa();
  String sms;
  String msg;
  int16_t msg_length;
  long start_time;
  String message="DATA";
  int packetsize=0;
  delay(500);
  while(millis()-start_time<=1000){
    sendMessage(message);
    Serial.println(message);
    Serial.println(packetsize);
  }
  sms=reciveBig();
  Serial.println(sms);
  Transform_NN_variables(sms);
  sendDeepSleep();
}
void sendDeepSleep(){
  int16_t sleep_timee;
   if (actual_hour > N_END_HOUR && actual_hour < N_START_HOUR) {
    sleep_timee = int(SLEEP_TIME) - (actual_min * 60 + actual_secs) % int(SLEEP_TIME) - (millis()) / 1000;
    if (sleep_timee <= 0) sleep_timee = sleep_timee + int(SLEEP_TIME);
  } else
  {
    Serial.println("N_SLEEP_TIME set.");
    sleep_timee = int(N_SLEEP_TIME) - (actual_min * 60 + actual_secs) % int(N_SLEEP_TIME) - (millis()) / 1000;
    if (sleep_timee <= 0) sleep_timee = sleep_timee + int(N_SLEEP_TIME);
  }
  SLEEP_TIME_modem = sleep_timee - SLEEP_TIME_PRE + 10;
  sendMessage(String(SLEEP_TIME_modem));
}