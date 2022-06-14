// Compatiblee con Peripheral v2.0.0
// V3.0.0 Date 02 Jun 22 by V.R 



//------------- Web Debug ------------------------------------------------
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

//------------- OTA ------------------------------------------------

#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
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





//_____________________________________________________________________//
//                                                                      //
//                  NEOLINK - AIDA                                      //                                                                      //
//______________________________________________________________________//

const String FIRMWARE_VER = "3.0.0";

const String DEVICE_TYPE = "NeoLink";
const char* DEVICE_HEADER = "NL";
const char* HARDWARE_VERSION = "03";
char SN_HEADER [20];
String SN_CORRELATIVE ;
String DATE_CORRELATIVE;
String SN;

#define FIRMWARE_MODE 'DEV'

RTC_DATA_ATTR int8_t HARDWARE_AVAILABLE = 0;

#if FIRMWARE_MODE == 'PRO'
  #define FIREBASE_HOST "https://neolink-934b4.firebaseio.com"
  #define FIREBASE_AUTH "IroB3fdbcPb9vxPlJKDJcqmfJgs0KouJGe0sUBKN"
  #define UPDATE_JSON_URL  "https://firmware-neolink.s3-sa-east-1.amazonaws.com/firmware_pro.json"
  const String WIFI_SSID = "LINUX5"; //modem default
  const String WIFI_PSSWD = "1a23456789abc";
 

#elif FIRMWARE_MODE == 'DEV'
  #define FIREBASE_HOST "https://aidadev-71837-default-rtdb.firebaseio.com/"
  #define FIREBASE_AUTH "RbsWJ3F5EsLGLvpRefgTeyGQhEFHFp5pJfECurTE"
  #define UPDATE_JSON_URL  "https://firmware-neolink.s3-sa-east-1.amazonaws.com/firmware_pro.json"
  const String WIFI_SSID = "BandSteering";
  const String WIFI_PSSWD = "A543PR4G";

#endif



//------------- OTA HTTPS  ---------------------------------------------

const char* ca = \
                 "-----BEGIN CERTIFICATE-----\n" \
                "MIIESTCCAzGgAwIBAgITBn+UV4WH6Kx33rJTMlu8mYtWDTANBgkqhkiG9w0BAQsF\n" \
                "ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n" \
                "b24gUm9vdCBDQSAxMB4XDTE1MTAyMjAwMDAwMFoXDTI1MTAxOTAwMDAwMFowRjEL\n" \
                "MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEVMBMGA1UECxMMU2VydmVyIENB\n" \
                "IDFCMQ8wDQYDVQQDEwZBbWF6b24wggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK\n" \
                "AoIBAQDCThZn3c68asg3Wuw6MLAd5tES6BIoSMzoKcG5blPVo+sDORrMd4f2AbnZ\n" \
                "cMzPa43j4wNxhplty6aUKk4T1qe9BOwKFjwK6zmxxLVYo7bHViXsPlJ6qOMpFge5\n" \
                "blDP+18x+B26A0piiQOuPkfyDyeR4xQghfj66Yo19V+emU3nazfvpFA+ROz6WoVm\n" \
                "B5x+F2pV8xeKNR7u6azDdU5YVX1TawprmxRC1+WsAYmz6qP+z8ArDITC2FMVy2fw\n" \
                "0IjKOtEXc/VfmtTFch5+AfGYMGMqqvJ6LcXiAhqG5TI+Dr0RtM88k+8XUBCeQ8IG\n" \
                "KuANaL7TiItKZYxK1MMuTJtV9IblAgMBAAGjggE7MIIBNzASBgNVHRMBAf8ECDAG\n" \
                "AQH/AgEAMA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUWaRmBlKge5WSPKOUByeW\n" \
                "dFv5PdAwHwYDVR0jBBgwFoAUhBjMhTTsvAyUlC4IWZzHshBOCggwewYIKwYBBQUH\n" \
                "AQEEbzBtMC8GCCsGAQUFBzABhiNodHRwOi8vb2NzcC5yb290Y2ExLmFtYXpvbnRy\n" \
                "dXN0LmNvbTA6BggrBgEFBQcwAoYuaHR0cDovL2NydC5yb290Y2ExLmFtYXpvbnRy\n" \
                "dXN0LmNvbS9yb290Y2ExLmNlcjA/BgNVHR8EODA2MDSgMqAwhi5odHRwOi8vY3Js\n" \
                "LnJvb3RjYTEuYW1hem9udHJ1c3QuY29tL3Jvb3RjYTEuY3JsMBMGA1UdIAQMMAow\n" \
                "CAYGZ4EMAQIBMA0GCSqGSIb3DQEBCwUAA4IBAQCFkr41u3nPo4FCHOTjY3NTOVI1\n" \
                "59Gt/a6ZiqyJEi+752+a1U5y6iAwYfmXss2lJwJFqMp2PphKg5625kXg8kP2CN5t\n" \
                "6G7bMQcT8C8xDZNtYTd7WPD8UZiRKAJPBXa30/AbwuZe0GaFEQ8ugcYQgSn+IGBI\n" \
                "8/LwhBNTZTUVEWuCUUBVV18YtbAiPq3yXqMB48Oz+ctBWuZSkbvkNodPLamkB2g1\n" \
                "upRyzQ7qDn1X8nn8N8V7YJ6y68AtkHcNSRAnpTitxBKjtKPISLMVCx7i4hncxHZS\n" \
                "yLyKQXhw2W2Xs0qLeC1etA+jTGDK4UfLeC0SF7FSi8o5LL21L8IzApar2pR/\n" \
                "-----END CERTIFICATE-----\n" ;


char* log1 = 
  "<script>t=document.createElement('div');"
  "t.appendChild( document.createElement('br').appendChild(document.createTextNode('hola perro2')));"
  "document.body.appendChild(t)</script>";

const char* serverIndex =
//"<body onload = loadlog()>"
"<body>"
  "<img src='https://static.wixstatic.com/media/5ab796_d078782c657b49f485a897ef2457a084~mv2.png/v1/fill/w_118,h_116,al_c,q_85,usm_0.66_1.00_0.01,enc_auto/IMG_3641_PNG.png' style='vertical-align:middle'>"
  "<img src='https://static.wixstatic.com/media/5ab796_bdead9c6276049e3afe6ddc5ea2080e5~mv2.png' height='55' style='vertical-align:middle'>"

 "<div><b> Greenbird is watching you! </b> </div>"
  "<div><br> Please, select a .ino </div>"
  "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
  "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
  "<input type='file' name='update'>"
  "<input type='submit' value='Update and Restart'>"
  "</form>"
  "<div id='prg'>progress: 0%</div>"
  "<div><br> Or use this options </div>"
  "<a  href=\"/quickRestart\"><button  name=\"quickRestart\">Quick Restart and reload</button></a>"
  "<a  href=\"/HWReset\"><button  name=\"HWReset\">Reset by Hardware </button></a><br>"
  "<a  href=\"/Exit_PRG_Mode\"><button  name=\"Exit_PRG_Mode\">Exit PRG_MODE</button></a>"
  "<a  href=\"/Intense_Mode\"><button  name=\"Intense_Mode\">Modo Intenso</button></a>"
  "<button type=\"button\" onclick=\"loadlog()\"> Get_data</button>"

  
  "<div><br><b>Read data: </b><br></div>"
  "<div><p id='log'></p></div>"



  "<script>"

    "var gateway = `ws://esp32.local/ws`;"
    "var websocket;"
    "window.addEventListener('load', initWebSocket);"

    "function initWebSocket(event) {"
      "console.log('Trying to open a WebSocket connection...');"
      "websocket = new WebSocket(gateway);"
      "websocket.onopen    = onOpen;"
      "websocket.onclose   = onClose;" 
      "websocket.onmessage = onMessage; "
      "}"

    "function onOpen(event) {"
    "console.log('Connection opened'); }"

    "function onClose(event) {"
    "console.log('Connection closed');"
    "setTimeout(initWebSocket, 2000);"
    "}"

    
    "let count=0;"
    "function onMessage(event) {"
     "console.log(event.data);"
     "count++;"
      "t=document.createElement('pre');"
      "t.setAttribute('id',count);"
      "t.appendChild( document.createElement('br').appendChild(document.createTextNode(event.data)));"
      "document.body.appendChild(t)"
  "}"


"</script>"

/*
  "<script>"
    "let count=0;"
    "function loadlog(){"
    "var xhttp = new XMLHttpRequest();"
    "xhttp.open('GET', '/log', false);"
    "xhttp.send();"
     "if (xhttp.readyState == 4 && xhttp.status == 200){"
      "count++;"
      "t=document.createElement('pre');"
      "t.setAttribute('id',count);"
      "t.appendChild( document.createElement('br').appendChild(document.createTextNode(xhttp.responseText)));"
      "document.body.appendChild(t)"
    " }"    
   "}"
   
  "</script>"*/

  "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')"
  "},"
  "error: function (a, b, c) {"
  "}"
  "});"
  "});"
  "</script>"
  
  "</body>";


//New functions v3


String string_Log="";
String tlog ="";

class vprint{

public: 


void logw(String e){
  String str;
  str.concat(millis());
  str.concat(": ");
  str.concat(e);
  string_Log.concat(str);
  Serial.print(str);
}

void logq(String e){
  String str;
  str.concat(millis());
  str.concat(": ");
  str.concat(e);
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}

void logq(int e){
  String str;
  str.concat(millis());
  str.concat(": ");
  str.concat(String(e));
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}

void logq(double e){
  String str;
  str.concat(millis());
  str.concat(": ");
  str.concat(String(e));
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}


void logq(String text , String e){
  String str;
  str.concat(millis());
  str.concat(": ");
  str.concat(text);
  str.concat(e);
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}

void logq(String text , int e){
  String str;
  str.concat(millis());
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}

void logq(String text , double e){
  String str;
  str.concat(millis());
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}

void logqq(String e){
  String str;
  str.concat("\t");
  str.concat(millis());
  str.concat(": ");
  str.concat(e);
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}

void logqq(String text , String e){
  String str;
  str.concat("\t");
  str.concat(millis());
  str.concat(": ");
  str.concat(text);
  str.concat(e);
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}

void logqq(String text , int e){
  String str;
  str.concat("\t");
  str.concat(millis());
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}

void logqq(String text , double e){
  String str;
  str.concat("\t");
  str.concat(millis());
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  string_Log.concat(str);
  Serial.print(str);
}

};


AsyncWebServer server(80);
AsyncWebSocket ws("/ws");



// receive buffer
char rcv_buffer[300];

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
#define PIN_WIFI_STATUS 39
#define TEMP_VALUE 33
#define SOLAR_VALUE 37
#define BAT_VALUE 36

#define SIM_ON 4
#define ATMEGA_FORCED_RESET_PIN 2


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
neoFirebaseJson json_port1;
neoFirebaseJson json_port2;
neoFirebaseJson json_port3;
neoFirebaseJson json_port4;
neoFirebaseJson json_port_status;
neoFirebaseJson json_state;
String timestamp_string2;
String timestamp_string;
String auxiliar_string1;
String DEVICE_NN;


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
int8_t LA_NN = 0;

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
int8_t counter_aux;
unsigned long start_MODE_PRG = 0;
int8_t half_conf_flag = 0;

RTC_DATA_ATTR int8_t no_atmos = 1;



//------------------  non-volatile configuration states ----------------
RTC_DATA_ATTR int8_t INCOMPATIBILIDAD_FIRMWARE_ERROR = 0;
RTC_DATA_ATTR int8_t Start_or_Restart = 1;
RTC_DATA_ATTR int8_t Update_LocalSN_Flag = 0;
RTC_DATA_ATTR int8_t NewConf_flag = 1;
RTC_DATA_ATTR int8_t battery_available = 1; 
RTC_DATA_ATTR int8_t is_registered = 0;
RTC_DATA_ATTR int8_t WIFI_EN = 1;
RTC_DATA_ATTR int8_t wifi_default = 1;
RTC_DATA_ATTR int8_t wifi_conf_isSet = 0;  //1 if parameters are defined
RTC_DATA_ATTR int8_t sim_enable = 0;
RTC_DATA_ATTR int8_t GPS_RQ = 0;
RTC_DATA_ATTR int8_t PORT_RQ = 0;
RTC_DATA_ATTR int8_t BEEP_EN = 0;
RTC_DATA_ATTR int8_t GPS_CLOUD = 0;
RTC_DATA_ATTR int8_t OBSERVER = 0;
RTC_DATA_ATTR int8_t SENSOR_P1 = 0;
RTC_DATA_ATTR int8_t SENSOR_P2 = 0;
RTC_DATA_ATTR int8_t SENSOR_P3 = 0;
RTC_DATA_ATTR int8_t SENSOR_P4 = 0;

RTC_DATA_ATTR int8_t cant_NN = 0;
RTC_DATA_ATTR int8_t cantNN_receive = 0;
RTC_DATA_ATTR int8_t cantNN_send = 0;
RTC_DATA_ATTR int config_begin = 11;
RTC_DATA_ATTR int long_mess = 0;
int cont_prg = 0;

RTC_DATA_ATTR int DEPTH1;
RTC_DATA_ATTR int DEPTH2;
RTC_DATA_ATTR int DEPTH3;
RTC_DATA_ATTR int DEPTH4;

RTC_DATA_ATTR int UNNECESARY_MEASURE = 0;
RTC_DATA_ATTR int NL_NN = 0;

RTC_DATA_ATTR char sensado[400] ;

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
String message_Json_port1;
String message_Json_port2;
String message_Json_port3;
String message_Json_port4;
String sms_upload;
String sms_general="";
String sms_neolink="";
String message_aux;
String sms_config;
bool error_message = false;

const String flag_adrr = "/Services_controllers_Flags/Devices";

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
RTC_DATA_ATTR float BAT_L = 3.2;
RTC_DATA_ATTR float BAT_H = 3.4;
RTC_DATA_ATTR int8_t Stage = 1;
RTC_DATA_ATTR float SLEEP_TIME_modem;
int SLEEP_TIME_STAGE_1 = 18;


RTC_DATA_ATTR int update_local_info_flag ;
int update_config_flag ;
int local_update_firmware;
int global_update_firmware;

// to make temporary storage for the average

RTC_DATA_ATTR float MA_port1_G_1[3] = {0,0,0};
RTC_DATA_ATTR float MA_port2_G_1[3] = {0,0,0};
RTC_DATA_ATTR float MA_port3_G_1[3] = {0,0,0};
RTC_DATA_ATTR float MA_port4_G_1[3] = {0,0,0};

RTC_DATA_ATTR float MA_port1_G_3[3] = {0,0,0};
RTC_DATA_ATTR float MA_port2_G_3[3] = {0,0,0};
RTC_DATA_ATTR float MA_port3_G_3[3] = {0,0,0};
RTC_DATA_ATTR float MA_port4_G_3[3] = {0,0,0};



//------------------  -----------------------------------  -----------------------------------  -----------------------------------  -----------------
//------------------  -----------------------------------  -----------------------------------  -----------------------------------  -----------------
//------------------  -----------------------------------  -----------------------------------  -----------------------------------  -----------------
//------------------  -----------------------------------  -----------------------------------  -----------------------------------  -----------------
//--------------------------------Funciones del sistema------------------------------------

double ReadVoltage(byte pin);
void deepsleep(int time2sleep);
int checking_battery();


void turn_modem_on();

int16_t setting_wifi();
void Upload(String message, int valo);
void starting_wifi();
void deepsleep_beep(int deepsleep_time, unsigned long beep_time);
void get_LOCAL_SN();
String message_node(String name , float val);

void NodeExistance();
void turn_modem_off();
int16_t real_sleep_time();
String read_eeprom();
void check_registered();
void save_eeprom(String config);
void val_config(String config_tempo);

void depth_request(String direc);
void check_configuration(String direction);

void moving_average_sensor();
void  get_ports_sensor2();
void get_ports_sensor();

void update_firmware( String file);
bool verify_json ( String new_firmware_ver, String current_firmware_ver, int built, int chk);
int FirmwareCheck();
void read_OpenWeather();
void get_environment_sensor();

void get_neolink_status();



void get_neonodes_signals();

void compatibility_error();
void force_hardware_reset(vprint print);
void config_LoRa0();
void sendMessage(String outgoing);
String onReceive(int packetSize);
String reciveBig();
bool Comparator_msg(String msg1,String msg2);
bool Comparator(String msg);
void sendDeepSleep();
String Transform_Variables(String msg,int n_var,int Port_Active);
void LoRa_Communication();
void sendd(vprint print);


String httpGETRequest(const char* serverName);
void setClock();
String getValue(String data, char separator, int index);
void send_cloud_NN(String sms);

//--------------------------------------------------------
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

  pinMode(ATMEGA_FORCED_RESET_PIN, OUTPUT);

  pinMode(PIN_WIFI_STATUS, INPUT);

  
 
  //SE DEBE CREAR UNA VARIABLE PARA HABILITAR LA PANTALLA?
  //Heltec.begin(false /*DisplayEnable Enable*/, false /*Heltec.LoRa Disable*/, false /*Serial Enable*/, false /*PABOOST Enable*/, BAND /*long BAND*/);
  //delay(1000);
  
  Serial.begin(115200);
  vprint print;

  EEPROM.begin(150);
  
  Serial.println("___________________________________________________");
  print.logq("Greenbird AG - AIDA | NeoLink Firmware Version: ", FIRMWARE_VER);

  if (Start_or_Restart) beep.vbeep(250);

  chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  chipid_str = String((uint16_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);
  print.logq("Chip ID: ", chipid_str);
  
  
  if(HARDWARE_AVAILABLE) print.logq("Running with Hadware available");
  else  print.logq("Running without Hadware. Just ESP32 board.");
  
  if (Stage == 1) {
    //verificar bateria, resetear arduino?, verificar si el wifi esta encendido y encenderlo. Cambiar a etapa 2

    print.logq("[STAGE 1]: ");
    if (!checking_battery(print)) deepsleep(POWERLESS_TIME,print);

    //physical forced reset Atmega328pu. Why?
    atmega_force_reset(print);

    
    if (!check_WiFi(print,3000)) turn_modem_on(print);
    
    

    Stage = 2; //Habilito el siguiente paso


    if(FIRMWARE_MODE == 'DEV') deepsleep(10,print);
    else deepsleep(SLEEP_TIME_STAGE_1,print);

  }

  else if (Stage == 2) {
  // Descargar le Local Serial Number. Verificar si hay nuevo Firmware. Descargar nueva configuracion. 

    print.logq("[STEP 2]:" );
    ArdSerial.begin(57600);

    if(!check_WiFi(print,1)) starting_wifi(print);

    Update_Local_Info_Chip(print, update_local_info_flag);

    Firebase.getInt(firebasedata, "Services_controllers_Flags/Devices/" + SN + "/local_update_firmware"); 
    local_update_firmware = firebasedata.intData();
    Firebase.getInt(firebasedata, "Services_controllers_Flags/Devices/" + SN + "/update_config_flag"); 
    update_config_flag = firebasedata.intData();
    Firebase.getInt(firebasedata, "Services_controllers_Flags/Devices/" + SN + "/update_local_info_flag"); 
    update_local_info_flag = firebasedata.intData();

    print.logq(local_update_firmware);
    print.logq(update_config_flag);
    print.logq(update_local_info_flag);
    

    
    
    


    
    
    
    
 

  }

    /*

    if(Update_LocalSN_Flag || char(EEPROM.read(0)) != 'N' || char(EEPROM.read(1)) != 'L') get_LOCAL_SN();
    else{
      SN = "";
      for(int i=0; i<11;i++) {
        SN = SN + char(EEPROM.read(i));
      }
      Serial.println("SN: " + SN);

    
      Serial.println("CHIP ID: " + chipid_str);
      Serial.println("SN FOUND: " + SN+"");
    }

    

    if (SN[0] == DEVICE_HEADER[0] && SN[1] == DEVICE_HEADER[1] ) INCOMPATIBILIDAD_FIRMWARE_ERROR =0;
    else INCOMPATIBILIDAD_FIRMWARE_ERROR = 1;
    if (INCOMPATIBILIDAD_FIRMWARE_ERROR) compatibility_error();

    DEVICE = "/" + DEVICE_TYPE + "/" + SN ;
    
    Serial.println("DEVICE: " + DEVICE);
    Serial.println("NeoLink version=" + FIRMWARE_VER);

    //---------------------------    Checking battery to start---------------

    if (checking_battery()) check_registered(); //checking if neolink is registered;
    //if (1) check_registered();
    else {
      Serial.println(" Not enought energy. Shutting down until battery_hysteresis ");
      deepsleep(POWERLESS_TIME);
    }

    

    
    if (is_registered) {
      check_configuration(DEVICE);
      get_ports_sensor2();
      get_environment_sensor();
      get_neolink_status();
      if(cant_NN != 0)  Stage = 4;
      else  {
        send_cloud_NN(sms_neolink);
        sendd();
      }
    }

  }



  else if(Stage == 4){
    LoRa_Communication();
    Serial.println("cont_prg" + cont_prg);
    if (cantNN_receive == cant_NN)  {
      Stage = 7;

    }
    if(cont_prg == 3) {
      Stage = 0;
      deepsleep(20);
    }
  }

  else if(Stage == 7){
    String message_send = getValue(sensado,'#',cantNN_send);
    Serial.println("message_send: " + message_send);

    //Upload(message_send, cantNN_send);
    send_cloud_NN(message_send);
    sendd();
    check_configuration(DEVICE);   
  }

  */
}

void loop() {
  if ( MODE_PRG) {
          //server.handleClient();
          delay(1);
         }
}

//---------------------------------------------------------

void Update_Local_Info_Chip(vprint print, int update_local_info_flag){

//Verifica que se encuentre el tipo de equipo en la eeprom no sea XX o vacio. Si lo es descarga la Local Info y guarda en la eeprom. 
//verifica la que la bandera update_local_info_chip se encuentre en 1 para descargar nuevamente la data

print.logq("Checking Local Info: ");

for( int i = 0; i <= 3 ; i++ ) SN_HEADER [i] = char(EEPROM.read (i));

print.logqq("SN_HEADER in EEPROM is: ", SN_HEADER);

EEPROM.write(0,'X');
EEPROM.commit();

print.logqq("EEPROM: ", char(EEPROM.read (0)) );

for( int i = 0; i <= 3 ; i++ ) SN_HEADER [i] = char(EEPROM.read (i));

print.logqq("SN_HEADER in EEPROM is: ", SN_HEADER);















}

int check_WiFi(vprint print, int millis_delay){
  int WiFi_HW_Status = 0;
  print.logq("Checking WiFi Modem Status");
  delay(millis_delay);
  WiFi_HW_Status = digitalRead(PIN_WIFI_STATUS);

  print.logqq("WiFi_HW_Status = ", WiFi_HW_Status );

  if(WiFi_HW_Status) print.logqq("WiFi is already ON.");
  else print.logqq("WiFi is OFF." , WiFi_HW_Status);


  return WiFi_HW_Status;
}

void turn_modem_off(vprint print) {
  print.logq("Turning Modem OFF");
  digitalWrite(SIM_ON, HIGH);
  delay(750);
  digitalWrite(SIM_ON, LOW);
  delay(2000);
  print.logqq("Done.");

}

void turn_modem_on(vprint print) {



  print.logq("Turning Modem ON.");
  digitalWrite(SIM_ON, HIGH);
  delay(1200);
  digitalWrite(SIM_ON, LOW);
  delay(2000);
  print.logqq("Done.");


}

//------------------------------------------------------------------
void starting_wifi(vprint print) {
  int wifi_try = 1;
  if (wifi_conf_isSet) {

    print.logq("Starting WiFi:");

    WiFi.begin(WIFI_SSID, WIFI_PSSWD);
    
    init_timestamp = millis();
    while (1) {
      while (WiFi.status() != WL_CONNECTED && millis() - init_timestamp < WIFI_TIME_LIMIT ) {
        Serial.print(".");
        delay(500);
      }
      if (WiFi.status() == WL_CONNECTED) {

        print.logqq("IP: ", String(WiFi.localIP()));

        Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
        Firebase.reconnectWiFi(true);
     
        break;

      }
      else {

        init_timestamp = millis();
        WiFi.begin(WIFI_SSID, WIFI_PSSWD);
        Serial.println("\n Enabling wifi: ERROR. Try: " + String(wifi_try));
        wifi_try++;

        if (wifi_try > 3) {

          Serial.println("Nothing to do. See ya.. ");
          Stage = 0;

           deepsleep(1,print);
        }

       

      }
    }

  }

}


int checking_battery(vprint print) {
  
  print.logq("Checking Battery: ");

  pinMode(BAT_VALUE, INPUT_PULLDOWN);
  pinMode(SOLAR_VALUE, INPUT_PULLDOWN);

  digitalWrite(BAT_SOLAR_EN, HIGH);
  delay(20);

  float solar_voltage_temp;
  float battery_voltage_temp;

  solar_voltage_temp  = ReadVoltage(SOLAR_VALUE) ;
  battery_voltage_temp = ReadVoltage(BAT_VALUE) ;

  digitalWrite(BAT_SOLAR_EN, LOW);

  if (battery_voltage <= 0 )battery_voltage = battery_voltage_temp;
  if ( solar_voltage <= 0) solar_voltage = solar_voltage_temp ;
    
  battery_voltage = (battery_voltage_temp + battery_voltage ) / 2 ;
  solar_voltage = (solar_voltage_temp + solar_voltage) / 2;

  solar_voltage   = solar_voltage * 0.003692945;
  print.logqq("Solar voltage: ", solar_voltage);

  battery_voltage = battery_voltage * 0.00395528142;

  if (battery_voltage <= BAT_L && battery_available ) battery_available = 0;
  if (battery_voltage >= BAT_H && !battery_available ) battery_available = 1; 



  if (battery_available) print.logqq("Available energy. Battery Voltage: ", battery_voltage);
  else print.logqq("Not enought energy. Shutting down. Battery Voltage: ", battery_voltage);

  if(!HARDWARE_AVAILABLE) battery_available = 1;

  return battery_available;
  
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
      //while(!Firebase.getString(firebasedata, "SN_CHIPS/" + chipid_str + "/SN_LOCAL"))if(millis()-init_SN_time >10000)deepsleep_beep(1,1000) ;
      Firebase.getString(firebasedata, "SN_CHIPS/" + chipid_str + "/SN_LOCAL");
      SN=firebasedata.stringData();
      Serial.println(SN);

      for(int i=0; i<SN.length();i++)  {
        //Serial.println(SN[i]);
        EEPROM.write(i, SN[i]);
        EEPROM.commit(); 
        //Serial.println(EEPROM.read(i));
      }

      Firebase.setInt(firebasedata, "SN_CHIPS/" + chipid_str + "/UPDATE", 0);

      Serial.println("State saved in flash memory: " + SN);

      
    }
  else  deepsleep_beep(1,800);
  Update_LocalSN_Flag = 0;

}





//------------------------------------------------------------------
void check_registered(vprint print) {
  if (!wifi_conf_isSet) wifi_conf_isSet = setting_wifi();
  if ( WiFi.status() != WL_CONNECTED) starting_wifi();
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
      //if(INCOMPATIBILIDAD_FIRMWARE_ERROR) Serial.println("[ERROR] FIRMWARE NO COMPATIBLE.");
      Serial.println("Device " + SN + "  is not registered. Shutting down.");
      beep.vbeep(100);
      delay(30);
      beep.vbeep(300);
      delay(30);
      beep.vbeep(100);
      /*if (Stage == 0) Stage = 9;
      else Stage = 0;
      */
      Stage = 0;
      deepsleep(UNREGISTERED_TIME,print);
    } else {
      //Serial.println(" Device " + SN + "..Registered");
      Serial.println(" Device " + SN + "..Registered");
      NodeExistance();

    }

  }
  else {
    Serial.println("Device " + SN + " already registered.");

    if (!Start_or_Restart) {
      Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_STATUS + "NewConf"); // if reset, newconf is 1 by default.
      NewConf_flag = firebasedata.intData();
      Serial.println("NewConf_flag: " + String(NewConf_flag));
      if(NewConf_flag)  config_begin = 11;
    }
  }


  if (NewConf_flag) {
    //check if is it

    check_configuration(DEVICE);

    if ( WiFi.status() != WL_CONNECTED) starting_wifi();

    if ( WiFi.status() == WL_CONNECTED) {
      String SN_NN;
      String LA_eeprom;
      while(SN_NN != "NaN"){
        //DEVICE = "/" + DEVICE_TYPE + "/" + SN ;

        SN_NN = "";
        Firebase.getString(firebasedata, "OLDneolinks/" + SN + "/neonodos/" + String(cant_NN)); 
        SN_NN = firebasedata.stringData();
        Serial.print("SN_NN: ");
        Serial.println(SN_NN);
        if(SN_NN != "NaN"){

          Firebase.getInt(firebasedata, DEVICE_TYPE + "/" + SN_NN + "/LA"); // if reset, newconf is 1 by default.
          LA_NN = firebasedata.intData();
          Serial.println("LA_NN: " + String(LA_NN));
          check_configuration(DEVICE_TYPE + "/" + SN_NN);
          cant_NN++;
        }
        else Serial.println("No tiene asociado ningun Neonodo");
      }
      NewConf_flag = 0;
    }
  }
  else {
    Serial.println("No se registran nuevos Neonodos!!");
  }




  if (Start_or_Restart) {
    Serial.println("entro Start_or_Restart");
    //det hora
    if (WIFI_EN) {
      if (!wifi_conf_isSet) wifi_conf_isSet = setting_wifi();
      if ( WiFi.status() != WL_CONNECTED) starting_wifi();
    }
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
      if (time_rep > 4) deepsleep(1, print);
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
      deepsleep(real_sleep_time(),print);
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

void check_configuration(String direction) {
  
  if (WIFI_EN) {
    if (!wifi_conf_isSet) wifi_conf_isSet = setting_wifi();
    if ( WiFi.status() != WL_CONNECTED) starting_wifi();
  }
  Serial.println("direction: " + direction);
  
  if (Start_or_Restart && UNNECESARY_MEASURE) {
    Serial.println("Reading new configuration due restart or first run.");
    NewConf_flag = 1;
    UNNECESARY_MEASURE = 0;
  } 


  String config_temp = "";
  if (NewConf_flag) {

    Serial.print("Checking new configuration...");



    Firebase.getFloat(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "BAT_H");
    BAT_H = firebasedata.floatData();
    sms_config = sms_config + "$" + String(BAT_H);
    Serial.print("\n BAT_H: " + String(BAT_H));

    Firebase.getFloat(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "BAT_L");
    BAT_L = firebasedata.floatData();
    sms_config = sms_config + "$" + String(BAT_L);
    Serial.print("\n BAT_L: " + String(BAT_L));

    Firebase.getInt(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "BEEP_EN");
    BEEP_EN = firebasedata.intData();
    sms_config = sms_config + "$" + String(BEEP_EN);
    Serial.print("\n BEEP_EN: " + String(BEEP_EN));

    //Firebase.getInt(firebasedata,  DEVICE + PATH_CONFIGURATION_VALUES + String("GPS_RQ"));
    //GPS_RQ = firebasedata.intData();

    Firebase.getFloat(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "NO_WIFI");
    NO_WIFI = firebasedata.floatData();
    sms_config = sms_config + "$" + String(NO_WIFI);
    Serial.print("\n NO_WIFI: " + String(NO_WIFI));

    if (!half_conf_flag) {

      Firebase.getFloat (firebasedata,  direction + PATH_CONFIGURATION_VALUES + "N_END_HOUR");
      N_END_HOUR = firebasedata.floatData();
      Serial.print("\n N_END_HOUR: " + String(N_END_HOUR));

      Firebase.getFloat(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "N_SLEEP_TIME");
      N_SLEEP_TIME = firebasedata.floatData();
      Serial.print("\n N_SLEEP_TIME: " + String(N_SLEEP_TIME));

      Firebase.getFloat(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "N_START_HOUR");
      N_START_HOUR = firebasedata.floatData();
      Serial.print("\n N_START_HOUR: " + String(N_START_HOUR));

      Firebase.getFloat(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "SLEEP_TIME");
      SLEEP_TIME = firebasedata.floatData();
      Serial.print("\n SLEEP_TIME: " + String(SLEEP_TIME));

    }

    else Serial.print("\n SLEEP_TIME parameters already read.");

    Firebase.getInt(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "PORT_RQ");
    PORT_RQ = firebasedata.intData();
    sms_config = sms_config + "$" + String(PORT_RQ);
    Serial.print("\n PORT_RQ: " + String(PORT_RQ));
    delay(100);

    depth_request(direction);

    Firebase.getInt(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "OBSERVER");
    OBSERVER = firebasedata.intData();
    sms_config = sms_config + "$" + String(OBSERVER);
    Serial.print("\n OBSERVER: " + String(OBSERVER));
    delay(100);

    Firebase.getInt(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "SENSOR/P1");
    SENSOR_P1 = firebasedata.intData();
    sms_config = sms_config + "$" + String(SENSOR_P1);
    Serial.print("\n SENSOR P1: " + String(SENSOR_P1));
    delay(100);
    
    Firebase.getInt(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "SENSOR/P2");
    SENSOR_P2 = firebasedata.intData();
    sms_config = sms_config + "$" + String(SENSOR_P2);
    Serial.print("\n SENSOR P2: " + String(SENSOR_P2));
    delay(100);
    
    Firebase.getInt(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "SENSOR/P3");
    SENSOR_P3 = firebasedata.intData();
    sms_config = sms_config + "$" + String(SENSOR_P3);
    Serial.print("\n SENSOR P3: " + String(SENSOR_P3));
    delay(100);

    Firebase.getInt(firebasedata,  direction + PATH_CONFIGURATION_VALUES + "SENSOR/P4");
    SENSOR_P4 = firebasedata.intData();
    sms_config = sms_config + "$" + String(SENSOR_P4);
    Serial.print("\n SENSOR P4: " + String(SENSOR_P4));
    delay(100);


    

    

    //Serial.print("programming mode doesnt exist in this version.");
    //Firebase.getBool(firebasedata, DEVICE + PATH_CONFIGURATION_STATUS + String("MODE_PRG"));
    //MODE_PRG = firebasedata.boolData();
    //if (!MODE_PRG) Serial.println("Negative.");
    //else Serial.println("Active.");
    Serial.println("\n Done.");

    sms_config = "#" + sms_config;
    if(LA_NN != 0) sms_config = "#" + String(LA_NN) + sms_config;

    save_eeprom(sms_config);
  

    Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++");
    Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++");
    Serial.print("sms_config: ");
    Serial.println(sms_config);
    Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++");
    Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++");
    sms_config="";

  }
  else
  {
    if(!error_message){
      Serial.println("No configuration found.");
      sms_config = read_eeprom();
      Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++");
      Serial.print("sms_config: ");
      Serial.println(sms_config);
      Serial.print("direction: ");
      Serial.println(direction);

      config_temp=getValue(sms_config,'#',2*cantNN_send + 1);
      Serial.print("config_temp: ");
      Serial.println(config_temp);
      Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++");
      val_config(config_temp);
    }

  }

}

//------------------------------------------------------------------


void  get_ports_sensor2() {
  int8_t loop1_flag = 1;
  char message[500];
  memset(message,0,500);
  int16_t port_div[50];
  int8_t sample_div[50];
  int16_t eos; //end of string
  int16_t m = 0;
  int8_t n = 0; // counter ports
  int8_t try_counter = 0;

  char recipe_aux;
  int stop=0;
  if (PORT_RQ) {
    Serial.println("Rebooting atmega..");
    digitalWrite(ARDUINO_RESTART, HIGH);
    delay(1);
    digitalWrite(ARDUINO_RESTART, LOW );
    delay(10);
    //restarting atmega to avoid overflo

    ArdSerial.write('s');
    Serial.println("Sensors on port requested. Atmega response: ");
    unsigned long time_sensor = millis();
    while (loop1_flag) {

      if (ArdSerial.available()) { 
        recipe_aux=ArdSerial.read();
        if(recipe_aux=='$'){
              stop=stop+1;
              if(stop==2)break;     
        }
 
        message[m]=recipe_aux;
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
    
    String message1=getValue(message,'$',1);
    Serial.println("-------------- Mensaje Recibido --------------");
    Serial.println(message1);
    port1_msg=getValue(message1,'%',1);
    port2_msg=getValue(message1,'%',2);
    port3_msg=getValue(message1,'%',3);
    port4_msg=getValue(message1,'%',4);
    
    Serial.println("");
    Serial.println(port1_msg);
    Serial.println(port2_msg);
    Serial.println(port3_msg);
    Serial.println(port4_msg);



    /*
      01:11:12.511 -> %P1_-3235.2 20.2_-3105.5 20.2_-2969.8 20.2_k%P2_-3036.4 20.2_-3006.5 20.2_k%P3 %P4 $
      01:11:12.511 -> -3235.2 20.2_-3105.5 20.2_-2969.8 20.2_k
      01:11:12.511 -> -3036.4 20.2_-3006.5 20.2_k

    */


    Serial.println("--------------------------- PORT 1 -----------------------------");

  
    if(getValue(port1_msg,'_',1).length()>4){

      port1_msg_len = port1_msg.length();
      int8_t n_samples = 0;
      int8_t n_var_init=0;
      for (int i = 0; i <= port1_msg_len ; i++) {
        if (port1_msg[i] == '_') {
          n_samples++;
        }
      }
      for (int i = 0; i <= port1_msg_len ; i++) {
        if (port1_msg[i] == ' ') {
          n_var_init++;
        }
      }
      
      int8_t n_var=(n_var_init/(n_samples-1))+1;
      String P1_to_transform=getValue(port1_msg,'_',0);
      for(int i=0; i<n_var;i++){
        float average=0;
         for(int j=1;j<=n_samples-1;j++){
          average=average+getValue(getValue(port1_msg,'_',j),' ',i).toFloat();
         }
         average=average/(n_samples-1);
         P1_to_transform=P1_to_transform+":"+"V"+String(i+1)+":"+String(average);
      }
      P1_to_transform=P1_to_transform+":"+getValue(port1_msg,'_',n_samples);
      Serial.println(P1_to_transform);

      Port1_Active = 1;
      Serial.print("Port1_Active ");
      Serial.println(Port1_Active);

      Serial.println(getValue(port1_msg,'_',n_samples));
      message_Json_port1=Transform_Variables(P1_to_transform,n_var,Port1_Active);
      
    }
    else {
      Port1_Active = 0;
      Serial.print("Port1_Active ");
      Serial.println(Port1_Active);
    }
    Serial.println(message_Json_port1);

    //--------------------------------PORT2-----------------------------------------------
    Serial.println("--------------------------- PORT 2 -----------------------------");

   
    if (getValue(port2_msg,'_',1).length()>4) {
      port2_msg_len = port2_msg.length();
      int8_t n_samples = 0;
      int8_t n_var_init=0;
      for (int i = 0; i <= port2_msg_len ; i++) {
        if (port2_msg[i] == '_') {
          n_samples++;
        }
      }
      for (int i = 0; i <= port2_msg_len ; i++) {
        if (port2_msg[i] == ' ') {
          n_var_init++;
        }
      }
      int8_t n_var=n_var_init/(n_samples-1)+1;
      String P2_to_transform=getValue(port2_msg,'_',0);
      for(int i=0; i<n_var;i++){
        float average=0;
         for(int j=1;j<=n_samples-1;j++){
          average=average+getValue(getValue(port2_msg,'_',j),' ',i).toFloat();
         }
         average=average/(n_samples-1);
         P2_to_transform=P2_to_transform+":"+"V"+String(i+1)+":"+String(average);
      }
      P2_to_transform=P2_to_transform+":"+getValue(port2_msg,'_',n_samples);
      Serial.println(P2_to_transform);

      Port2_Active = 1;
      Serial.print("Port2_Active ");
      Serial.println(Port2_Active);


      Serial.println(getValue(port2_msg,'_',n_samples));
      message_Json_port2=Transform_Variables(P2_to_transform,n_var,Port2_Active);
    }
    else {
      Port2_Active = 0;
      Serial.print("Port2_Active ");
      Serial.println(Port2_Active);
    }
    Serial.println(message_Json_port2);
    

    //--------------------------------PORT3-----------------------------------------------
    Serial.println("--------------------------- PORT 3 -----------------------------");

   
    if (getValue(port3_msg,'_',1).length()>4) {
      port3_msg_len = port3_msg.length();
      int8_t n_samples = 0;
      int8_t n_var_init=0;
      for (int i = 0; i <= port3_msg_len ; i++) {
        if (port3_msg[i] == '_') {
          n_samples++;
        }
      }
      for (int i = 0; i <= port3_msg_len ; i++) {
        if (port3_msg[i] == ' ') {
          n_var_init++;
        }
      }
      int8_t n_var=n_var_init/(n_samples-1)+1;
      String P3_to_transform=getValue(port3_msg,'_',0);
      for(int i=0; i<n_var;i++){
        float average=0;
         for(int j=1;j<=n_samples-1;j++){
          average=average+getValue(getValue(port3_msg,'_',j),' ',i).toFloat();
         }
         average=average/(n_samples-1);
         P3_to_transform=P3_to_transform+":"+"V"+String(i+1)+":"+String(average);
      }
      P3_to_transform=P3_to_transform+":"+getValue(port3_msg,'_',n_samples);
      Serial.println(P3_to_transform);

      Port3_Active = 1;
      Serial.print("Port3_Active ");
      Serial.println(Port3_Active);


      Serial.println(getValue(port3_msg,'_',n_samples));
      message_Json_port3=Transform_Variables(P3_to_transform,n_var,Port3_Active);
    }
    else {
      Port3_Active = 0;
      Serial.print("Port3_Active ");
      Serial.println(Port3_Active);
    }
    Serial.println("-----------------------");
    Serial.println(message_Json_port3);
     Serial.println("-----------------------");
    //--------------------------------PORT4-----------------------------------------------
    Serial.println("--------------------------- PORT 4 -----------------------------");

    
    if (getValue(port4_msg,'_',1).length()>4){
      port4_msg_len = port4_msg.length();
      int8_t n_samples = 0;
      int8_t n_var_init=0;
      for (int i = 0; i <= port4_msg_len ; i++) {
        if (port4_msg[i] == '_') {
          n_samples++;
        }
      }
      for (int i = 0; i <= port4_msg_len ; i++) {
        if (port4_msg[i] == ' ') {
          n_var_init++;
        }
      }
      int8_t n_var=n_var_init/(n_samples-1)+1;
      String P4_to_transform=getValue(port4_msg,'_',0);
      for(int i=0; i<n_var;i++){
        float average=0;
         for(int j=1;j<=n_samples-1;j++){
          average=average+getValue(getValue(port4_msg,'_',j),' ',i).toFloat();
         }
         average=average/(n_samples-1);
         P4_to_transform=P4_to_transform+":"+"V"+String(i+1)+":"+String(average);
      }
      P4_to_transform=P4_to_transform+":"+getValue(port4_msg,'_',n_samples);
      Serial.println(P4_to_transform);

      Port4_Active = 1;
      Serial.print("Port4_Active ");
      Serial.println(Port4_Active);


      Serial.println(getValue(port4_msg,'_',n_samples));
      message_Json_port4=Transform_Variables(P4_to_transform,n_var,Port4_Active);
    }
    else {
      Port4_Active = 0;
      Serial.print("Port4_Active ");
      Serial.println(Port4_Active);
    }
    Serial.println(message_Json_port4);

    /*
      beep.vbeep(250);
      delay(100);
      beep.vbeep(250);
      delay(100);
      beep.vbeep(250);
*/
    //-------------------------------------------------------------------------------
    Serial.println("--------------------------- END PORT -----------------------------");
    moving_average_sensor();

  }
  else Serial.println("No Port sensing  request.");
  Serial.println();
}



void get_ports_sensor() {





  
  int8_t loop1_flag = 1;
  char message[500];
  int16_t port_div[50];
  int8_t sample_div[50];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  div[50];
  int16_t eos; //end of string
  int16_t m = 0;
  int8_t n = 0; // counter ports
  int8_t try_counter = 0;
  float average;
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

    port1_msg=getValue(message,'%',1);
    port2_msg=getValue(message,'%',2);
    port3_msg=getValue(message,'%',3);
    port4_msg=getValue(message,'%',4);
    
   
    Serial.println(port1_msg);
    Serial.println(port2_msg);
    Serial.println(port3_msg);
    Serial.println(port4_msg);



    /*
      01:11:12.511 -> %P1_-3235.2 20.2_-3105.5 20.2_-2969.8 20.2_k%P2_-3036.4 20.2_-3006.5 20.2_k%P3 %P4 $
      01:11:12.511 -> -3235.2 20.2_-3105.5 20.2_-2969.8 20.2_k
      01:11:12.511 -> -3036.4 20.2_-3006.5 20.2_k

    */


    Serial.println("--------------------------- PORT 1 -----------------------------");

    port1_msg_len = port1_msg.length();

    if (port1_msg_len > 0) {
      int8_t n_samples = 0;
      int8_t n_var_init=0;
      for (int i = 0; i <= port1_msg_len ; i++) {
        if (port1_msg[i] == '_') {
          n_samples++;
        }
      }
      for (int i = 0; i <= port1_msg_len ; i++) {
        if (port1_msg[i] == ' ') {
          n_var_init++;
        }
      }
      int8_t n_var=n_var_init/(n_samples-1)+1;
      String P1_to_transform=getValue(port1_msg,'_',0);
      for(int i=0; i<n_var;i++){
        average=0;
         for(int j=1;j<=n_samples-1;j++){
          average=average+getValue(getValue(port1_msg,'_',j),' ',i).toFloat();
         }
         average=average/(n_samples-1);
         P1_to_transform=P1_to_transform+":"+"V"+String(i+1)+":"+String(average);
      }
      P1_to_transform=P1_to_transform+":"+getValue(port1_msg,'_',n_samples);
      Serial.println(P1_to_transform);

      /*port1_sample1 = port1_msg.substring(0, sample_div[0]);
      port1_sample2 = port1_msg.substring(sample_div[0] + 1 , sample_div[1]);
      port1_sample3 = port1_msg.substring(sample_div[1] + 1, port1_msg_len - 2);
      port1_type = port1_msg.substring(port1_msg_len - 1, port1_msg_len);*/

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
      Serial.println(getValue(port1_msg,'_',n_samples));
      message_Json_port1=Transform_Variables(P1_to_transform,n_var,Port1_Active);
    }
    else {
      Port1_Active = 0;
      Serial.print("Port1_Active ");
      Serial.println(Port1_Active);
    }
    Serial.println(message_Json_port1);

    //--------------------------------PORT2-----------------------------------------------
   port2_msg_len = port2_msg.length();

    if (port2_msg_len > 0) {
      int8_t n_samples = 0;
      int8_t n_var_init=0;
      for (int i = 0; i <= port2_msg_len ; i++) {
        if (port2_msg[i] == '_') {
          n_samples++;
        }
      }
      for (int i = 0; i <= port2_msg_len ; i++) {
        if (port2_msg[i] == ' ') {
          n_var_init++;
        }
      }
      int8_t n_var=n_var_init/(n_samples-1)+1;
      String P2_to_transform=getValue(port2_msg,'_',0);
      for(int i=0; i<n_var;i++){
        average=0;
         for(int j=1;j<=n_samples-1;j++){
          average=average+getValue(getValue(port2_msg,'_',j),' ',i).toFloat();
         }
         average=average/(n_samples-1);
         P2_to_transform=P2_to_transform+":"+"V"+String(i+1)+":"+String(average);
      }
      P2_to_transform=P2_to_transform+":"+getValue(port2_msg,'_',n_samples);
      Serial.println(P2_to_transform);

      Port2_Active = 1;
      Serial.print("Port2_Active ");
      Serial.println(Port2_Active);


      Serial.println(getValue(port2_msg,'_',n_samples));
      message_Json_port2=Transform_Variables(P2_to_transform,n_var,Port2_Active);
    }
    else {
      Port2_Active = 0;
      Serial.print("Port2_Active ");
      Serial.println(Port2_Active);
    }
    Serial.println(message_Json_port2);

    //--------------------------------PORT3-----------------------------------------------
    Serial.println("--------------------------- PORT 3 -----------------------------");

    port3_msg_len = port3_msg.length();
    if (port3_msg_len > 0) {
      int8_t n_samples = 0;
      int8_t n_var_init=0;
      for (int i = 0; i <= port3_msg_len ; i++) {
        if (port3_msg[i] == '_') {
          n_samples++;
        }
      }
      for (int i = 0; i <= port3_msg_len ; i++) {
        if (port3_msg[i] == ' ') {
          n_var_init++;
        }
      }
      int8_t n_var=n_var_init/(n_samples-1)+1;
      String P3_to_transform=getValue(port3_msg,'_',0);
      for(int i=0; i<n_var;i++){
        average=0;
         for(int j=1;j<=n_samples-1;j++){
          average=average+getValue(getValue(port3_msg,'_',j),' ',i).toFloat();
         }
         average=average/(n_samples-1);
         P3_to_transform=P3_to_transform+":"+"V"+String(i+1)+":"+String(average);
      }
      P3_to_transform=P3_to_transform+":"+getValue(port3_msg,'_',n_samples);
      Serial.println(P3_to_transform);

      Port3_Active = 1;
      Serial.print("Port3_Active ");
      Serial.println(Port3_Active);


      Serial.println(getValue(port3_msg,'_',n_samples));
      message_Json_port3=Transform_Variables(P3_to_transform,n_var,Port3_Active);
    }
    else {
      Port3_Active = 0;
      Serial.print("Port3_Active ");
      Serial.println(Port3_Active);
    }
    Serial.println(message_Json_port3);

    //--------------------------------PORT4-----------------------------------------------
    Serial.println("--------------------------- PORT 4 -----------------------------");

    port4_msg_len = port4_msg.length();
    if (port4_msg_len > 0) {
      int8_t n_samples = 0;
      int8_t n_var_init=0;
      for (int i = 0; i <= port4_msg_len ; i++) {
        if (port4_msg[i] == '_') {
          n_samples++;
        }
      }
      for (int i = 0; i <= port4_msg_len ; i++) {
        if (port4_msg[i] == ' ') {
          n_var_init++;
        }
      }
      int8_t n_var=n_var_init/(n_samples-1)+1;
      String P4_to_transform=getValue(port4_msg,'_',0);
      for(int i=0; i<n_var;i++){
        average=0;
         for(int j=1;j<=n_samples-1;j++){
          average=average+getValue(getValue(port4_msg,'_',j),' ',i).toFloat();
         }
         average=average/(n_samples-1);
         P4_to_transform=P4_to_transform+":"+"V"+String(i+1)+":"+String(average);
      }
      P4_to_transform=P4_to_transform+":"+getValue(port4_msg,'_',n_samples);
      Serial.println(P4_to_transform);

      Port4_Active = 1;
      Serial.print("Port4_Active ");
      Serial.println(Port4_Active);


      Serial.println(getValue(port4_msg,'_',n_samples));
      message_Json_port4=Transform_Variables(P4_to_transform,n_var,Port4_Active);
    }
    else {
      Port4_Active = 0;
      Serial.print("Port4_Active ");
      Serial.println(Port4_Active);
    }
    Serial.println(message_Json_port4);

    //-------------------------------------------------------------------------------
    Serial.println("--------------------------- END PORT -----------------------------");
    moving_average_sensor();

  }
  else Serial.println("No Port sensing  request.");
  Serial.println();
}
//------------------------------------------------------------------

void moving_average_sensor(vprint print) {
  
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

void get_gps(vprint print) {

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
          deepsleep(100,print);
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
      Serial.println(bme_aux.sensorID(), HEX);
    }

    else {
      
      Serial.print("[AUX] Temp: ");
      dry_bulb_temp_aux = bme_aux.readTemperature();
      Serial.println(dry_bulb_temp_aux);
      message_node("iT_aux",dry_bulb_temp_aux);
      Serial.print("[AUX] Hum: ");
      relative_humidity_aux = bme_aux.readHumidity();
      Serial.println(relative_humidity_aux);
      message_node("RH_aux",relative_humidity_aux);
    }
    


    if (!status_atm) {
      Serial.println("Could not find a valid [ATM] BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID [ATM] was: 0x");
      Serial.println(bme_static.sensorID(), HEX);
      digitalWrite(ATMOS_EN, LOW);
      rep_atm = rep_atm + 1;
      delay(750);
    } 
    
    else {
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

      
      message_node("dT",dry_bulb_temp_mov);
      message_node("dT_raw",dry_bulb_temp);
      message_node("AL",pressure_altitude);
      message_node("BP",barometric_pressure);

      if (dry_bulb_temp > 0 && relative_humidity > 0 && barometric_pressure < 104) {
        if (relative_humidity == 100 && relative_humidity_past != 0 && (relative_humidity - relative_humidity_past > 25) ) {
          Serial.println("RH satured. Skip.");
          rep_atm = rep_atm + 1;
          no_atmos = 1;
          if (rep_atm > 15) {
            digitalWrite(ATMOS_EN, LOW);
            message_node("RH",relative_humidity);
            break;}
        }
        else {
          Serial.print("| Past RH: ");
          Serial.println(relative_humidity_past);
          relative_humidity_past = relative_humidity;
          digitalWrite(ATMOS_EN, LOW);
          no_atmos = 0;
          message_node("RH",relative_humidity);
          break;
        }
      }
      else {
        rep_atm = rep_atm + 1;
        no_atmos = 1;
        if (rep_atm > 15) {
          if (relative_humidity_past != 0) relative_humidity = relative_humidity_past;
          message_node("RH",relative_humidity);
          break;
        }
      }
    }
    break;
    if(rep_atm > 15){
      no_atmos = 0;
      Serial.println("no_atmos: " + String(no_atmos));
      break;
    }
  }
}


//------------------------------------------------------------------



void get_neolink_status() {
  int check_temp = 0;

  //SOLAR

  Serial.print("Solar voltage = ");

  Serial.println(solar_voltage);
  message_node("SV",solar_voltage);


  Serial.print("Battery voltage = ");
  Serial.println(battery_voltage);
  message_node("BV",battery_voltage);

  //Internal TEMP
  digitalWrite(TEMP_EN, HIGH);
  delay(1000);
  sensors.begin();

  do{
    sensors.requestTemperatures(); 
    internal_temperature_raw = sensors.getTempCByIndex(0);
    delay(500);
    if (internal_temperature_raw < -126)  Serial.println("error: " + String(internal_temperature_raw));
    check_temp++;
  }while(internal_temperature_raw < -126.00 && check_temp < 5);
  if(internal_temperature_raw < -126.00)  internal_temperature_raw = -90;

  Serial.print("Internal Temperature RAW = ");
  Serial.println(internal_temperature_raw);
  message_node("iT_raw",internal_temperature_raw);

 

  if (internal_temperature == 0) internal_temperature = internal_temperature_raw ;
  internal_temperature = (internal_temperature_raw  + internal_temperature) / 2;
  Serial.print("Internal Temperature Mean = ");
  Serial.println(internal_temperature);
  message_node("iT",internal_temperature);

  digitalWrite(TEMP_EN, LOW);

  sms_neolink=SN+'$'+message_Json_port1+'$'+message_Json_port2+'$'+message_Json_port3+'$'+message_Json_port4+sms_general+'$'+'#';
  Serial.println("////////////////////////////////////////////////////////////");
  Serial.println(sms_neolink);
  Serial.println("////////////////////////////////////////////////////////////");
  sms_general="";

  if(cant_NN != 0){
    for(int i=long_mess ; i<sms_neolink.length() ; i++){
    sensado[i] = sms_neolink[i] ;
    }
    long_mess = sms_neolink.length();
  }
  

}








String message_node(String name , float val){
  
  sms_general=sms_general+"$"+name+"$"+String(val);
  
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
  SLEEP_TIME_modem = sleep_timee - SLEEP_TIME_STAGE_1 + 10;
  if (SLEEP_TIME_modem < 1) SLEEP_TIME_modem = 10; //+10

  //Serial.println (SLEEP_TIME_modem);

  Serial.print ("NeoLink wakeup = ");
  Serial.println (SLEEP_TIME_STAGE_1);

  if (Stage == 0) {
    Stage = 9;
    return  SLEEP_TIME_STAGE_1 + SLEEP_TIME_modem + 10 ; //+10
  }
  else if (cantNN_send < cant_NN){
    Stage = 7;
    return  SLEEP_TIME_modem;
  }
  else if (cantNN_send == cant_NN){
    Stage = 0;
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
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Hardware_version", HARDWARE_VERSION);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "IP", "None");
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "LastOP_TIME", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "LastUpload", "NaN");

      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/PoreCer/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/PoreCer/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/PoreCer/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V2/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V3/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V3/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/V3/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/vwc/AU", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/vwc/CC", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/vwc/PMP", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/g/vwc/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port1/k/V2/bool", 0);

      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/PoreCer/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/PoreCer/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/PoreCer/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V2/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V3/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V3/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/V3/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/vwc/AU", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/vwc/CC", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/vwc/PMP", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/g/vwc/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port2/k/V2/bool", 0);

      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/PoreCer/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/PoreCer/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/PoreCer/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V2/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V3/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V3/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/V3/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/vwc/AU", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/vwc/CC", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/vwc/PMP", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/g/vwc/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port3/k/V2/bool", 0);

      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/PoreCer/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/PoreCer/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/PoreCer/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V2/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V3/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V3/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/V3/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/vwc/AU", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/vwc/CC", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/vwc/PMP", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/g/vwc/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V1/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V1/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V1/bool", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V2/Max", 100);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V2/Min", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "Limits/Port4/k/V2/bool", 0);
      //json_default_nodes.set(PATH_CONFIGURATION_STATUS + "MODE_PRG", false);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "NewConf", 0);
      json_default_nodes.set(PATH_CONFIGURATION_STATUS + "LOCAL_UPDATE", 0);
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
    message_node("WS",wind_speed);
    Serial.print("Wind Direction: ");
    Serial.println(wind_deg);



  }
  else Serial.println("WiFi Disconnected. No OpenWeather API requested.");


}


int FirmwareCheck() {
  Serial.println("--------------------------------");
  Serial.println("Looking for a new firmware...");
  String firmware__ver = FIRMWARE_VER;
  
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

        cJSON *device = cJSON_GetObjectItemCaseSensitive(json, HARDWARE_VERSION);

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
        Serial.println("new_firmware_ver: "+ new_firmware_ver);
        Serial.println("FIRMWARE_VER: "+ firmware__ver);

        if (verify_json ( new_firmware_ver, firmware__ver, built->valueint, chk->valueint)) update_firmware(file->valuestring);
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
    
    if(current_major < new_major) flag =true;
    else if (current_major == new_major)
    {
      if(current_minor < new_minor) flag =true;
      else if (current_minor == new_minor)
      {
        if(current_patch < new_patch) flag =true;
        else  Serial.println ("Lower or equal Firmware found (" + new_firmware_ver + ")");
      }
      else  Serial.println ("Lower or equal Firmware found (" + new_firmware_ver + ")");
    }
    else  Serial.println ("Lower or equal Firmware found (" + new_firmware_ver + ")");
    
    if(flag){
      Serial.print ("Current Firmware: ");
      Serial.println(current_firmware_ver);
      Serial.print ("Higher Firmware found: ");
      Serial.println(new_firmware_ver);
    }

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



void depth_request(String direc) {

  Serial.print("\n Requesting Depths:");

  Firebase.getInt(firebasedata,  direc + PATH_CONFIGURATION_VALUES + "DEPTH/P1");
  DEPTH1 = firebasedata.intData();
  sms_config = sms_config + "$" + String(DEPTH1);
  Serial.print("\n DEPTH1: " + String(DEPTH1));
  delay(100);

  Firebase.getInt(firebasedata,  direc + PATH_CONFIGURATION_VALUES + "DEPTH/P2");
  DEPTH2 = firebasedata.intData();
  sms_config = sms_config + "$" + String(DEPTH2);
  Serial.print("\n DEPTH2: " + String(DEPTH2));
  delay(100);

  Firebase.getInt(firebasedata,  direc + PATH_CONFIGURATION_VALUES + "DEPTH/P3");
  DEPTH3 = firebasedata.intData();
  sms_config = sms_config + "$" + String(DEPTH3);
  Serial.print("\n DEPTH3: " + String(DEPTH3));
  delay(100);

  Firebase.getInt(firebasedata,  direc + PATH_CONFIGURATION_VALUES + "DEPTH/P4");
  DEPTH4 = firebasedata.intData();
  sms_config = sms_config + "$" + String(DEPTH4);
  Serial.print("\n DEPTH4: " + String(DEPTH4));
  delay(100);




}

void atmega_force_reset(vprint print){

  print.logq("Forcing Atmega to reset.");

  digitalWrite(ATMEGA_FORCED_RESET_PIN, HIGH);
  delay(500);
  digitalWrite(ATMEGA_FORCED_RESET_PIN, LOW);
  delay(2000);

  print.logqq("Reseting Atmega: Done.");



}

void force_hardware_reset(vprint print){

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);


    unsigned long task_start_time;
    int i=0;
    //1. Reset Atmega328
    atmega_force_reset(print);

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

void deepsleep(int time2sleep, vprint print) {
 
  esp_sleep_enable_timer_wakeup(time2sleep * uS_TO_S_FACTOR);
  print.logq("Going to sleep for " + String(time2sleep), " seconds");
  
  delay(100);
  esp_deep_sleep_start();

}

void deepsleep_beep(int deepsleep_time, unsigned long beep_time, vprint print){
  print.logq("Beep reset. ");
  beep.vbeep(beep_time);
  deepsleep(deepsleep_time,print);
  
}

void deepsleep_several_beep(int deepsleep_time, unsigned long beep_time, int times, int between, vprint print){
  
  for(int i=0; i<= times; i++){
    beep.vbeep(beep_time);
    delay(between);
  }
 
  deepsleep(deepsleep_time,print);
  
}

void compatibility_error(vprint print){
  print.logq("[ERROR] FIRMWARE NO COMPATIBLE.");
  deepsleep_several_beep(120, 500, 3, 1000, print); // seg, ms, times, ms

}




void Transform_NN_variables(String message){
String mess_1=getValue(message,'$',0);
String mess_2=getValue(message,'$',1);
String mess_3=getValue(message,'$',2);
String mess_4=getValue(message,'$',3);
String mess_general=getValue(message,'$',4);

Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
Firebase.reconnectWiFi(true);



  neoFirebaseJson json_port1;
  neoFirebaseJson json_port2;
  neoFirebaseJson json_port3;
  neoFirebaseJson json_port4;
  neoFirebaseJson json_port_status;

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
    Serial.println(msg_prev);
    Serial.println(message);
    long start_time=millis();
    while(packetsize==0){
      packetsize=LoRa.parsePacket();
      if(millis()-start_time>2000){
        message_aux=message_aux+message;
        Serial.println("°°°°°°°°°°°°°°°°°°° Repeat °°°°°°°°°°°°°°°°°");
        return "Repeat";
      }
    }
    msg=onReceive(packetsize);
    Serial.println(msg);
    //control=0;
    control=Comparator(msg);//mejorar
    packetsize=0;
  }
 Serial.println("--1--");
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
    Serial.println("iguales");
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

  SLEEP_TIME_modem = sleep_timee - SLEEP_TIME_STAGE_1 + 10;
  if (SLEEP_TIME_modem < 1) SLEEP_TIME_modem = 10; //+10
  
  long start_time=millis();
  while(millis()-start_time<2000){
    sendMessage(String(SLEEP_TIME_modem));
    Serial.println("a dormir: " + String(SLEEP_TIME_modem));
  }
}


String Transform_Variables(String msg,int n_var,int Port_Active){
 
  //String variables_g="%P_humidity_m3m3%P_aparent_perm%P_pore_perm%P_water_pore_cond";
  Serial.println(n_var);
  String msg_Json="";
  float datos[n_var];
  for(int i=1;i<=n_var;i++){
    datos[i-1]=getValue(msg,':',2*i).toFloat();
    Serial.println(datos[i-1]);
  }  
  if(Port_Active){
    if(getValue(msg,':',2*n_var+1)=="g"){
      //String variables_g="%/P/VWC%/P/ApPer%/P/PorePer%/P/PoreCE";
      String variables_g="%VWC%ApPer%PorePer%PoreCE";
      int n_separator=0;
      for (int i = 0; i <= variables_g.length(); i++) {
        if (variables_g[i]== '%') {
          n_separator++;
        }
      }
      float valor[n_separator];
      //valor[0]=P_humidity_m3m3;
      //valor[1]=P_aparent_perm;
      //valor[2]=P_pore_perm;
      //valor[3]=P_water_pore_cond;
      const float perm_aparent_water=4.1;
      valor[0] = 3.879 * datos[0] / 10000 - 0.6956;
      if (valor[0] < 0) valor[0] = 0;
      //APPARENT DIELECTRIC PERMITTIVITY
      valor[1] = pow((2.887e-9 * pow(datos[0], 3) - 2.08e-5 * pow(datos[0], 2) + 5.276e-2 * datos[0] - 43.39), 2);
      //Real portion of Dielectric permittivity of the soil pore water
      valor[2] = 80.3 - 0.37 * (datos[1] - 20);
      //pore water EC (dS/m)
      if (valor[0] >= 0.05) valor[3] = (valor[2] * datos[2] / (valor[1] - perm_aparent_water)) * 10e-3;
      else valor[3] = 0;
      if (valor[3] < 0) valor[3] = 0 ;

      //Serial.println("RAW: "+ String(datos[0],2));
      for(int i=0;i<n_var;i++){
       msg_Json=msg_Json+":"+"/"+getValue(msg,':',0)+"/"+"V"+String(i+1)+":"+String(datos[i],4);
      }
      for(int i=0;i<n_separator;i++){
        msg_Json=msg_Json+":"+"/"+getValue(msg,':',0)+"/"+getValue(variables_g,'%',i+1)+":"+String(valor[i],4);
      }
      msg_Json=msg_Json+"_"+getValue(msg,':',2*n_var+1);
      //Serial.println(msg_Json);
    }
    else{
        for(int i=0;i<n_var;i++){
       msg_Json=msg_Json+":"+"/"+getValue(msg,':',0)+"/"+"V"+String(i+1)+":"+String(datos[i],4);
      }
      msg_Json=msg_Json+"_"+getValue(msg,':',2*n_var+1);
    }
  } else msg_Json="NaN";
  return msg_Json;
}

void Upload(String message, int valo) {

  if ( WiFi.status() != WL_CONNECTED) starting_wifi();

  if ( WiFi.status() == WL_CONNECTED) {
    Firebase.setString(firebasedata, "/PRUEBA/Mensaje/" + String(valo) , message);
    /*
    beep.vbeep(250);
    delay(100);
    beep.vbeep(250);
    delay(100);
    beep.vbeep(250);
    delay(100);
    beep.vbeep(250);
    delay(100);
    beep.vbeep(250);
    */
  }

}


void  send_cloud_NN(String sms, vprint print){

  Port1_Active=0;
  Port2_Active=0;
  Port3_Active=0;
  Port4_Active=0;


  String SN_NN=getValue(sms,'$',0);
  DEVICE_NN = "/" + DEVICE_TYPE + "/" + SN_NN ;
  //---------------------------------------------------PORT 1--------------------------------------------------------------
    if (getValue(getValue(sms,'$',1),'_',1).length()>0) {
      Port1_Active=1;
      message_Json_port1=getValue(sms,'$',1);
      Serial.println(message_Json_port1);
      json_port_status.FirebaseJson::set( "Port1_Active" , getValue(message_Json_port1,'_',1));
      
      String message=getValue(message_Json_port1,'_',0);
      Serial.println(message);
      int8_t n_var_init=0;
      for (int i = 0; i <= message.length() ; i++) {
        if (message[i]== ':') {
         n_var_init++;
        }
      }
        int8_t n_var=n_var_init/2;
      for(int i=0;i<n_var;i++){
        json_port1.set(getValue(message,':',2*i+1),double(getValue(message,':',2*i+2).toFloat()),2);
      }


  } else  json_port_status.FirebaseJson::set( "Port1_Active", "NaN");
  //-------------------------------------------------PORT 2----------------------------------------------------------------
  if (getValue(getValue(sms,'$',2),'_',1).length()>0) {
      Port2_Active=1;
      message_Json_port2=getValue(sms,'$',2);
      json_port_status.FirebaseJson::set( "Port2_Active" , getValue(message_Json_port2,'_',1));
      String message=getValue(message_Json_port2,'_',0);
      int8_t n_var_init=0;
      for (int i = 0; i <= message.length() ; i++) {
        if (message[i]== ':') {
         n_var_init++;
        }
      }
        int8_t n_var=n_var_init/2;
      for(int i=0;i<n_var;i++){
        json_port2.set(getValue(message,':',2*i+1),double(getValue(message,':',2*i+2).toFloat()),2);
      }
    

  }
  else  json_port_status.FirebaseJson::set( "Port2_Active", "NaN");
  //-------------------------------------------------PORT 3----------------------------------------------------------------
  if (getValue(getValue(sms,'$',3),'_',1).length()>0) {
      Port3_Active=1;
      message_Json_port3=getValue(sms,'$',3);
      json_port_status.FirebaseJson::set( "Port3_Active" , getValue(message_Json_port3,'_',1));
      String message=getValue(message_Json_port3,'_',0);
      int8_t n_var_init=0;
      for (int i = 0; i <= message.length() ; i++) {
        if (message[i]== ':') {
         n_var_init++;
        }
      }
        int8_t n_var=n_var_init/2;
      for(int i=0;i<n_var;i++){
        json_port3.set(getValue(message,':',2*i+1),double(getValue(message,':',2*i+2).toFloat()),2);
      }
  }
  else  json_port_status.FirebaseJson::set( "Port3_Active", "NaN");

  //-----------------------------------------------PORT 4------------------------------------------------------------------
  if (getValue(getValue(sms,'$',4),'_',1).length()>0) {
      Port4_Active=1;
      message_Json_port4=getValue(sms,'$',4);
      Serial.println(message_Json_port4);
      json_port_status.FirebaseJson::set( "Port4_Active" , getValue(message_Json_port4,'_',1));
      String message=getValue(message_Json_port4,'_',0);
      Serial.println(message);
      int8_t n_var_init=0;
      for (int i = 0; i <= message.length() ; i++) {
        if (message[i]== ':') {
         n_var_init++;
        }
      }
        int8_t n_var=n_var_init/2;
      for(int i=0;i<n_var;i++){
        json_port4.set(getValue(message,':',2*i+1),double(getValue(message,':',2*i+2).toFloat()),2);
      }
  }
  else  json_port_status.FirebaseJson::set("Port4_Active", "NaN");

  //----------------------------------------------------END PORT------------------------------------------------------------

  if(!no_atmos){
    int j=0;
    int cont=5;
    for(int i=0; i<sms.length(); i++ ){
      if(sms[i] == '$')  j++;
    }
    String ms = "";
    String valor = "";

    while(cont < j){
      ms = getValue(sms,'$',cont);
      Serial.println("ms: " + ms);
      valor = getValue(sms,'$',cont+1);
      Serial.println("valor: " + valor);
      if(ms == "AL" || ms == "WD" || ms == "OP_TIME"){
        //json_state.FirebaseJson::set(ms, int(valor));
      }
      else{
        json_state.set(ms,valor.toDouble(),2);
      }
      cont = cont + 2;
    }
    json_state.FirebaseJson::set("AL", int(pressure_altitude));
    json_state.FirebaseJson::set("WD", int(wind_deg));
    json_state.FirebaseJson::set("OP_TIME", int(millis() / 1000));
  }


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
    force_hardware_reset(print);
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
    depth_request(DEVICE_NN); 
    if (SENSOR_P1) p1_depth_err = DEPTH1 == 0;
    if (SENSOR_P2) p2_depth_err = DEPTH2 == 0;
    if (SENSOR_P3) p3_depth_err = DEPTH3 == 0;
    if (SENSOR_P4) p4_depth_err = DEPTH4 == 0;
    if (p1_depth_err || p2_depth_err || p3_depth_err || p4_depth_err) deepsleep(1,print);
    Serial.println();
  }
 
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
    if (time_rep > 4) deepsleep(1,print);
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



  auxiliar_string1 = _year + "." + real_month + "." + real_day + "." + real_hour + "." + real_min + "." + real_secs;

  String _hour; 

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
          if (time_rep > 4) deepsleep(1,print);
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


  timestamp_string2 ="/" + _year + "/" + _month + "/" + _day + "/" + _hour + "/" + _min;
  Serial.println("Firebase timestamp: " + timestamp_string2);
  timestamp_string = "/" + _year + "/" + _month + "/" + _day + "/" + _hour + "/" + _min;
}


void sendd(vprint print){

  check_WiFi(print, 1);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  if (WIFI_EN) {
    if (!wifi_conf_isSet) wifi_conf_isSet = setting_wifi();
    if ( WiFi.status() != WL_CONNECTED) starting_wifi();
  }


  //sending Firebase
  if (PORT_RQ) {
    Firebase.updateNodeSilent(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "Port/" , json_port_status);
    
    if (Port1_Active){Firebase.updateNodeSilent(firebasedata, DEVICE_NN +  PATH_DATA  + getValue(message_Json_port1,'_',1) + timestamp_string , json_port1);
    Serial.println("Send P1.");
    delay(150);}

    if (Port2_Active){Firebase.updateNodeSilent(firebasedata, DEVICE_NN +  PATH_DATA  + getValue(message_Json_port2,'_',1) + timestamp_string , json_port2);
    Serial.println("Send P2.");
    delay(150);}

    if (Port3_Active){Firebase.updateNodeSilent(firebasedata, DEVICE_NN +  PATH_DATA  + getValue(message_Json_port3,'_',1) + timestamp_string , json_port3);
    Serial.println("Send P3.");
    delay(150);}

    if (Port4_Active){Firebase.updateNodeSilent(firebasedata, DEVICE_NN +  PATH_DATA  + getValue(message_Json_port4,'_',1) + timestamp_string , json_port4);
    Serial.println("Send P4.");
    delay(150);}
  }
  
  //if (GPS_CLOUD) Firebase.updateNodeSilent(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS  + "GPS" , json_gps);

  delay(150);
  Firebase.updateNodeSilent(firebasedata, DEVICE_NN + PATH_DATA  + "/State" + timestamp_string, json_state);
  double LAST_OP_TIME = double(millis() / 1000);
  Firebase.setDouble(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "LastOP_TIME", LAST_OP_TIME );


  Firebase.setString(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "WiFi_rssi", String(WiFi.RSSI()));


  //Serial.print("LastUpload: ");
  //Serial.println(auxiliar_string);

  Firebase.setString(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "LastUpload", auxiliar_string1 );

  delay(150);

  //Firebase.updateNodeSilent(firebasedata, DEVICE + "/Observer" + timestamp_string, json_observer);

  


  Firebase.setString(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "FireTimeStamp", timestamp_string2 );
  

  if (Start_or_Restart) {
    Firebase.setString(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "LastRestart", auxiliar_string1);
    Start_or_Restart = 0;
    Firebase.setString(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "Firmware", FIRMWARE_VER);
    Firebase.setString(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "WifiSSID", WIFI_SSID_DEFAULT);
  }

  int real_sleep;
  if ( !MODE_PRG) {
    Firebase.setString(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "IP", "Nope");
    real_sleep = real_sleep_time();
    Serial.print("Real Sleep Time = ");
    Serial.println(real_sleep);
    Firebase.setInt(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "real_sleep", real_sleep);

    Firebase.setInt(firebasedata,  DEVICE_NN + PATH_CONFIGURATION_STATUS + "NewConf", 0);

    /*
      if (GPS_CLOUD) {
      //Serial.println("GPS_RQ a cero");
      Firebase.setInt(firebasedata, DEVICE + PATH_CONFIGURATION_VALUES + "GPS_RQ", 0 );
      }
    */




    if (cantNN_send == cant_NN){
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      turn_modem_off();
    }



  }
  else Firebase.setString(firebasedata, DEVICE_NN + PATH_CONFIGURATION_STATUS + "IP", String( WiFi.localIP().toString()));

  //beep.vbeep(200);
  if (BEEP_EN) beep.vbeep(150);

  if (WiFi.status() != WL_CONNECTED) Serial.println("Wi-Fi Disconnected");
  else  Serial.println("Wi-Fi Still Connected\n");

  Serial.println("OP_TIME_2: " + String(int(millis() / 1000)));

  Serial.println("cant_NN: " + String(cant_NN));

  Serial.println("cantNN_receive: " + String(cantNN_receive));

  Serial.println("cantNN_send: " + String(cantNN_send));

  Serial.println("Stage: " + String(Stage));
/*
  if (MODE_PRG && NL_NN == 1){
    NL_NN = 0;
    long start_time=millis();
    String message=String(real_sleep);
    int packetsize=0;    
    while(millis()-start_time<1000){
      sendMessage(message);
    }
    deepsleep(real_sleep-1);

  } 
  */



  if (cantNN_send == cant_NN){
    cantNN_send = 0;
    cantNN_receive = 0;
    long_mess = 0;
    memset(sensado,0,300);
    //beep.vbeep(1000);
    deepsleep(real_sleep-1,print);
  }
  else {
    cantNN_send++;
  }


  //start_server();

  if (BEEP_EN) {
    beep.vbeep(100);
    delay(40);
    beep.vbeep(100);
  }
}


void save_eeprom(String config){
  String eeprom_tempo="";

  eeprom_tempo = read_eeprom();

  Serial.print("eeprom_tempo: ");
  Serial.println(eeprom_tempo);

  String eeprom;

  eeprom = eeprom_tempo + config ;
  config_begin=eeprom.length();

  for(int i=0; i<config_begin;i++)  {
    EEPROM.write(0x00 + i, eeprom[i]);
  } 
  EEPROM.commit(); 

  Serial.println("eeprom: " + eeprom);
}

String read_eeprom(){
  //NL0000-0022#2$3#NL0000-0022#0$1$26$6$........#NN0000-0002#6$6$4$5$.......
  //NL0000-0022#1#2#NL0000-0022#$$3.60$3.25$0$20.00$0$0$0$0$0$0$0$0$0$0#NN0000-0002#$$3.60$3.25$0$120.00$1$1$0$0$10$0$1$0$0$1
  String eeprom_temp="";

  for(int i=0; i<config_begin ;i++){
    eeprom_temp = eeprom_temp + char(EEPROM.read(i));
  }

  return eeprom_temp;
}

void val_config(String config_tempo){
  Serial.println("-----------------------------------------");
  Serial.println("-----------------------------------------");
  BAT_H =     (getValue(config_tempo,'$',1)).toFloat();
  BAT_L =     (getValue(config_tempo,'$',2)).toFloat();
  BEEP_EN =   (getValue(config_tempo,'$',3)).toInt();
  NO_WIFI =   (getValue(config_tempo,'$',4)).toFloat();
  PORT_RQ =   (getValue(config_tempo,'$',5)).toInt();
  DEPTH1 =    (getValue(config_tempo,'$',6)).toInt();
  DEPTH2 =    (getValue(config_tempo,'$',7)).toInt();
  DEPTH3 =    (getValue(config_tempo,'$',8)).toInt();
  DEPTH4 =    (getValue(config_tempo,'$',9)).toInt();
  OBSERVER =  (getValue(config_tempo,'$',10)).toInt();
  SENSOR_P1 = (getValue(config_tempo,'$',11)).toInt();
  SENSOR_P2 = (getValue(config_tempo,'$',12)).toInt();
  SENSOR_P3 = (getValue(config_tempo,'$',13)).toInt();
  SENSOR_P4 = (getValue(config_tempo,'$',14)).toInt();
  Serial.print("BAT_H: ");
  Serial.println(BAT_H);
  Serial.print("BAT_L: ");
  Serial.println(BAT_L);
  Serial.print("BEEP_EN: ");
  Serial.println(BEEP_EN);
  Serial.print("NO_WIFI: ");
  Serial.println(NO_WIFI);
  Serial.print("PORT_RQ: ");
  Serial.println(PORT_RQ);
  Serial.print("DEPTH1: ");
  Serial.println(DEPTH1);
  Serial.print("DEPTH2: ");
  Serial.println(DEPTH2);
  Serial.print("DEPTH3: ");
  Serial.println(DEPTH3);
  Serial.print("DEPTH4: ");
  Serial.println(DEPTH4);
  Serial.print("OBSERVER: ");
  Serial.println(OBSERVER);
  Serial.print("SENSOR_P1: ");
  Serial.println(SENSOR_P1);
  Serial.print("SENSOR_P2: ");
  Serial.println(SENSOR_P2);
  Serial.print("SENSOR_P3: ");
  Serial.println(SENSOR_P3);
  Serial.print("SENSOR_P4: ");
  Serial.println(SENSOR_P4);
  Serial.println("-----------------------------------------");
  Serial.println("-----------------------------------------");
  
}


