
//                                                                      
// NEOLINK - AIDA v3.0.0                                                                                                       //
//
// Compatiblee con Peripheral v2.0.0
// Date 02 Jun 22 by V.R 
//
//______________________________________________________________________
//
// Librerias
//______________________________________________________________________


//------------- Web Debug ------------------------------------------------
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>//include webserver modified. 
#include <ESPmDNS.h>
#include <Update.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
//#include <String.h>
#include "Constants.h"
//------------- OTA HTTPS ------------------------------------------------
#include "cJSON.h"
#include <WiFiClientSecure.h>
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include <HTTPUpdate.h>
//------------- Serials----------------------------------------------------------
#include <SoftwareSerial.h>
//------------OWN---------------------------------
#include <beep.h>
//--------- INTERNAL TEMP ----------------
#include <OneWire.h>
#include <DallasTemperature.h>

//------------- I2C ------------------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//--------- EEPROM -----------------------    
#include <EEPROM.h>
//-----------------------------Firebase & Servers-------------------------------------
#include "FirebaseESP32.h"
#include "FirebaseJson.h"
#include "neoFirebaseJson.h"
#include <Arduino_JSON.h>
#include <HTTPClient.h>

#include <stdlib.h>
#include <Time.h>

//______________________________________________________________________
//
// Pinout
//______________________________________________________________________


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

//https://resource.heltec.cn/download/WiFi_LoRa_32/WIFI_LoRa_32_V2.pdf

//______________________________________________________________________
//
// Controllers
//______________________________________________________________________

#define FIRMWARE_MODE 'DEV'
RTC_DATA_ATTR int16_t MODE_PRG = 1;
RTC_DATA_ATTR int16_t LOCAL_SERVER = 1;
RTC_DATA_ATTR int8_t HARDWARE_AVAILABLE = 0;



//______________________________________________________________________
//
// Constantes Globales
//______________________________________________________________________


const String firmware_version = "3.0.0";
const char* host = "esp32";

//#define BAND    433E6

#define uS_TO_S_FACTOR 1000000

char SN_HEADER [20];
String DEVICE_HEADER_FIRMWARE = "NL";
String HARDWARE_VERSION_FIRMWARE = "02";


#if FIRMWARE_MODE == 'PRO'
  #define FIREBASE_HOST "https://neolink-934b4.firebaseio.com"
  #define FIREBASE_AUTH "IroB3fdbcPb9vxPlJKDJcqmfJgs0KouJGe0sUBKN"
  #define UPDATE_JSON_URL  "https://firmware-neolink.s3-sa-east-1.amazonaws.com/firmware_pro.json"
  const char* WIFI_SSID = "LINUX5"; //modem default
  const char* WIFI_PSSWD = "1a23456789abc";
  
 

#elif FIRMWARE_MODE == 'DEV'
  #define FIREBASE_HOST "https://aidadev-71837-default-rtdb.firebaseio.com/"
  #define FIREBASE_AUTH "RbsWJ3F5EsLGLvpRefgTeyGQhEFHFp5pJfECurTE"
  #define UPDATE_JSON_URL  "https://neolink-firmware-manager.s3.amazonaws.com/"
  const char* WIFI_SSID = "MOVISTAR_9F86";
  const char* WIFI_PSSWD = "9Qt6DFyaXZUG7SPkgZzn";
  //const char* WIFI_SSID = "LINUX5"; //modem default
  //const char* WIFI_PSSWD = "1a23456789abc";
  

#endif

//Time Variables

#define POWERLESS_TIME 600   //seconds to sleep when battery is low.
#define WIFI_TIME_LIMIT 6000 //seconds to connect wifi

#define NO_PING_TIME 180      // when doesnt exist first internet connection
#define NO_INTERNET_DESPITE_PING 60 //  when doesnt exist internet connection before the first one

#define DEFAULT_SN_SLEEPTIME 60

//EEPROM const address

const int ATMOS_RQ_eeprom = 15;
const  int BAT_H_eeprom = 16;
const  int BAT_L_eeprom = 20;
const  int BEEP_END_eeprom = 24;
const  int BEEP_INIT_eeprom = 25;
const  int DAY_SLEEPTIME_eeprom = 26;
const  int DAY_SLEEP_HOUR_eeprom = 30; 
const  int LATITUDE_eeprom =34;
const  int LONGITUDE_eeprom = 38;
const  int NIGHT_SLEEPTIME_eeprom =42;
const  int NIGHT_SLEEP_HOUR_eeprom = 46; 
const  int PORT_RQ_eeprom = 50;
const  int SD_ENABLE_eeprom = 51;
const  int PORT_1_ENABLE_eeprom = 66;
const  int DEPTH_1A_eeprom = 67;
const  int DEPTH_1B_eeprom = 71;
const  int PORT_2_ENABLE_eeprom = 75;
const  int DEPTH_2A_eeprom = 76;
const  int DEPTH_2B_eeprom = 80;
const  int PORT_3_ENABLE_eeprom = 84;
const  int DEPTH_3A_eeprom = 85;
const  int DEPTH_3B_eeprom = 89;
const  int PORT_4_ENABLE_eeprom = 93;
const  int DEPTH_4A_eeprom = 94;
const  int DEPTH_4B_eeprom = 98;
const  int PORT_5_ENABLE_eeprom = 102;
const  int DEPTH_5A_eeprom = 103;
const  int DEPTH_5B_eeprom = 107;
const  int PORT_6_ENABLE_eeprom = 111;
const  int DEPTH_6A_eeprom = 112;
const  int DEPTH_6B_eeprom = 116;





//______________________________________________________________________
//
//Variable globales
//______________________________________________________________________



AsyncWebServer server(80);
AsyncWebSocket ws("/ws");



String DEVICE_HEADER;
String HARDWARE_VERSION;
String ENVIRONMENT;
String DATE_CORRELATIVE;
String SN_CORRELATIVE;


String SN;
String chipid_str;

//firebase
FirebaseData firebasedata;

// Serials
SoftwareSerial ArdSerial(esp32_RX___ard_TX, esp32_TX___ard_RX);

//Time Server
const long  gmtOffset_sec =  -18000;
const int   daylightOffset_sec = 0;

//Own
Beep beep(BEEP);

//I2C
Adafruit_BME280 bme_static;

//Internal Temperature
OneWire oneWire(TEMP_VALUE);
DallasTemperature sensors(&oneWire);

// Settings
RTC_DATA_ATTR bool ATMOS_RQ = false;
RTC_DATA_ATTR float BAT_H = 3.4;
RTC_DATA_ATTR float BAT_L = 3.2;
RTC_DATA_ATTR bool BEEP_END = false;
RTC_DATA_ATTR bool BEEP_INIT = false;
RTC_DATA_ATTR int DAY_SLEEPTIME = 15;  //en minutos
RTC_DATA_ATTR int DAY_SLEEP_HOUR = 8; // en horas
RTC_DATA_ATTR float LATITUDE = -8.078078;
RTC_DATA_ATTR float LONGITUDE = -79.122082;
RTC_DATA_ATTR int NIGHT_SLEEPTIME = 15;
RTC_DATA_ATTR int NIGHT_SLEEP_HOUR = 20;
RTC_DATA_ATTR bool PORT_RQ = false;
RTC_DATA_ATTR bool SD_ENABLE = false;

RTC_DATA_ATTR bool PORT_1_ENABLE = false;
RTC_DATA_ATTR int DEPTH_1A = 0;
RTC_DATA_ATTR int DEPTH_1B = 0;

RTC_DATA_ATTR bool PORT_2_ENABLE = false;
RTC_DATA_ATTR int DEPTH_2A = 0;
RTC_DATA_ATTR int DEPTH_2B = 0;

RTC_DATA_ATTR bool PORT_3_ENABLE = false;
RTC_DATA_ATTR int DEPTH_3A = 0;
RTC_DATA_ATTR int DEPTH_3B = 0;

RTC_DATA_ATTR bool PORT_4_ENABLE = false;
RTC_DATA_ATTR int DEPTH_4A = 0;
RTC_DATA_ATTR int DEPTH_4B = 0;

RTC_DATA_ATTR bool PORT_5_ENABLE = false;
RTC_DATA_ATTR int DEPTH_5A = 0;
RTC_DATA_ATTR int DEPTH_5B = 0;

RTC_DATA_ATTR bool PORT_6_ENABLE = false;
RTC_DATA_ATTR int DEPTH_6A = 0;
RTC_DATA_ATTR int DEPTH_6B = 0;



//Time between stages
int SLEEP_TIME_STAGE_1 = 18;

//______________________________________________________________________
//
//Variables globales auxiliares
//______________________________________________________________________

String string_Log=""; //para concatenar todos los mensajes de consola
unsigned long init_timestamp;
RTC_DATA_ATTR int8_t Stage = 1;
RTC_DATA_ATTR int8_t Start_or_Restart = 1;

RTC_DATA_ATTR float battery_voltage = -10;
RTC_DATA_ATTR float solar_voltage = -10;
RTC_DATA_ATTR float internal_temperature = -100;
RTC_DATA_ATTR int8_t battery_available = 1; 
RTC_DATA_ATTR int update_sn_flag = 0 ;

RTC_DATA_ATTR float dry_bulb_temp = -100;
RTC_DATA_ATTR float barometric_pressure = -100;
RTC_DATA_ATTR float relative_humidity = -100;
RTC_DATA_ATTR float pressure_altitude = -100;

RTC_DATA_ATTR int last_restart_timestamp;



//______________________________________________________________________
//
// Clases propias. Convertirlas en librerias.
//______________________________________________________________________

class vprint{

public: 


void SendMessageWeb(String message) {
  ws.textAll(message);
}


void log(String e){
  String str;
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(e);
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logq(String e){
  String str;
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(e);
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logq(int e){
  String str;
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(String(e));
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logq(double e){
  String str;
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(String(e));
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}


void logq(String text , String e){
  String str;
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(text);
  str.concat(e);
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logq(String text , int e){
  String str;
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logq(String text , double e){
  String str;
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logqq(String e){
  String str;
  str.concat("\t");
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(e);
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logqq(String text , String e){
  String str;
  str.concat("\t");
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(text);
  str.concat(e);
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logqq(String text , int e){
  String str;
  str.concat("\t");
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logqq(String text , double e){
  String str;
  str.concat("\t");
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}



void logqqq(String e){
  String str;
  str.concat("\t\t");
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(e);
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logqqq(String text , String e){
  String str;
  str.concat("\t\t");
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(text);
  str.concat(e);
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logqqq(String text , int e){
  String str;
  str.concat("\t\t");
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

void logqqq(String text , double e){
  String str;
  str.concat("\t\t");
  str.concat(String(millis()/1000.000,3));
  str.concat(": ");
  str.concat(text);
  str.concat(String(e));
  str.concat("\n");
  SendMessageWeb(str);
  string_Log.concat(str);
  Serial.print(str);
}

};

vprint print;


//______________________________________________________________________
//
// Headers
//______________________________________________________________________

int checking_battery(vprint print);
void deepsleep(int time2sleep, vprint print);
int check_WiFi(vprint print, int millis_delay);
void turn_modem_on(vprint print);
void handleCommands(String data);
void handleCommands(String data, int timeToSleep);
String get_value_json_str(String * response, String variable_required);
void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
void start_server(vprint print);
void starting_wifi(vprint print);
void check_configuration(vprint print);
void write_eeprom(String string_to_eeprom, int position, vprint print);
String read_eeprom(int init, int len);
bool get_SN_n_MAC(vprint print);
bool ping(vprint print);
void print_controller_status(vprint print);
void create_SN_nodes(vprint print);
void create_nodes(vprint print) ;
String* get_firebase_json_str(vprint print);
void deepsleep(int time2sleep, vprint print, String message);
void update_SN(vprint print, bool compatible_fw_flag);
bool str_to_bool(String bool_str, vprint print);
void get_firebase_configuration(vprint print); 

void  get_ports(vprint print);

void force_hardware_reset(vprint print);
void atmega_soft_reset(vprint print);
void atmega_force_reset(vprint print);

bool send_peripheral_command(char command , vprint print);
void get_atmos(vprint print);

bool send_cloud(vprint print);

void sd_saving(vprint print);

bool firmware_update_manager(int update_mode, String version, vprint print);
bool update_firmware(String file_name, vprint print, String certificate);

//______________________________________________________________________
//
// Codigo principal
//______________________________________________________________________


void setup(void) {

  pinMode(ARDUINO_RESTART, OUTPUT);
  digitalWrite(ARDUINO_RESTART, LOW);

  pinMode(BAT_SOLAR_EN, OUTPUT);
  digitalWrite(BAT_SOLAR_EN, LOW);

  pinMode(TEMP_EN, OUTPUT);
  digitalWrite(TEMP_EN, HIGH);

  pinMode(ATMOS_EN, OUTPUT);
  digitalWrite(ATMOS_EN, HIGH);

  pinMode(SIM_ON, OUTPUT);
  digitalWrite(SIM_ON, HIGH);

  pinMode(ATMEGA_FORCED_RESET_PIN, OUTPUT);
  digitalWrite(ATMEGA_FORCED_RESET_PIN, LOW);

  pinMode(PIN_WIFI_STATUS, INPUT);

 
  //SE DEBE CREAR UNA VARIABLE PARA HABILITAR LA PANTALLA?
  //Heltec.begin(false /*DisplayEnable Enable*/, false /*Heltec.LoRa Disable*/, false /*Serial Enable*/, false /*PABOOST Enable*/, BAND /*long BAND*/);
  //delay(1000);

  Serial.begin(115200);
  ArdSerial.begin(57600);
  EEPROM.begin(150);
  
  //Stage =1;
  if (Stage == 1) {
    //beep.vbeep(250);
    Serial.println();
    Serial.println("__________________________________________________________");
    print.logq("Greenbird AG - AIDA | NeoLink Firmware Version: ", firmware_version);

    //physical forced reset Atmega328pu. Prevention
    atmega_force_reset(print);

    get_SN_n_MAC(print);
    
    print_controller_status(print);    
  
    //verificar bateria, resetear arduino?, verificar si el wifi esta encendido y encenderlo. Cambiar a etapa 2

    print.logq("[STAGE 1]: ");
    if (!checking_battery(print)) deepsleep(POWERLESS_TIME,print);
    
    if (!check_WiFi(print,3000)) turn_modem_on(print);
    
    

    Stage = 2; //Habilito el siguiente paso

    

    if(MODE_PRG) deepsleep(1,print);
    else deepsleep(SLEEP_TIME_STAGE_1, print);

  }

  else if (Stage == 2) {
    Serial.println("__________________________________________________________");
    print.logq("[STAGE 2]: ");

    //activate sensors read
    if(send_peripheral_command('s',print)) print.logq("Reading ports started.");

    // Connect to WiFi network
    if(check_WiFi(print,500)) starting_wifi(print);
    else { Stage = 1 ; turn_modem_on(print); handleCommands("SoftReset"); }
    
    // Enabling local server if is required
    if(LOCAL_SERVER){ws.onEvent(onEvent); server.addHandler(&ws); start_server(print); }

    //checking first if there is internet
    if(!ping(print)) deepsleep(1,print,"Deepsleep because no ping."); //handleCommands("SoftReset",NO_PING_TIME);

    checking_battery(print);
    
    //getting SN an MAC, again..
    bool compatible_fw_flag = get_SN_n_MAC(print); 

    update_SN(print , compatible_fw_flag);

    read_controllers_flags(print); // read update flags and OTA update. 

    read_configuration(print);

    get_atmos(print);

    get_ports(print);

    sd_saving(print);

    send_cloud(print);

    
    Stage = 1;


    delay(500);

}

}

void loop(void) {
  if(LOCAL_SERVER) ws.cleanupClients();
  if(MODE_PRG) handleCommands("SoftReset",60);

}


//______________________________________________________________________
//
// Functions
//______________________________________________________________________


void sd_saving(vprint print){
  print.logq("No Sd routine defined.");

}

bool  send_cloud(vprint print){

 
  
  neoFirebaseJson data_json;
  neoFirebaseJson status_json;

  print.logq("Getting timestamp:");

  configTime(gmtOffset_sec, daylightOffset_sec, "0.pool.ntp.org", "1.pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  int time_attemps = 0;
  

  do
  {
    getLocalTime(&timeinfo);
    time_attemps ++;
    print.logqq("Attemps: ", time_attemps);
    if (time_attemps > 3) deepsleep(1, print);
    delay(200);

  } while (timeinfo.tm_year - 100 < 0);


  char time_formated[40];
  strftime(time_formated, 40, "%m-%d-%Y %H:%M:%S", &timeinfo); 
  print.logqq("Current Time: " + String(time_formated));



  if (Start_or_Restart) {
    last_restart_timestamp = int(mktime(&timeinfo));
    Start_or_Restart = 0;  
    
  }

  
  tm last_restart_tm;
  time_t last_restart_time = last_restart_timestamp;
  localtime_r(&last_restart_time, &last_restart_tm);
  char restart_time_formated[40];
  strftime(restart_time_formated, 40, "%m-%d-%Y %H:%M:%S", &last_restart_tm); 
  print.logqq("Restart_time_formated: " + String(restart_time_formated));

  

  int _year = timeinfo.tm_year + 1900;
  int8_t _mon = timeinfo.tm_mon;
  int8_t _day = timeinfo.tm_mday;
  int8_t _hour = timeinfo.tm_hour;
  int8_t _min = timeinfo.tm_min;
  int8_t _secs = timeinfo.tm_sec;

  /*print.logq(timeinfo.tm_year);
  print.logq(_mon);
  print.logq(_day);
  print.logq(_hour);
  print.logq(_min);
  print.logq(_secs);*/


  _min = _min -_min % 5;

  
  //print.logq(_min);
  struct tm roundtime_timestamp = timeinfo;   //timestamp
  roundtime_timestamp.tm_min = _min ;
  roundtime_timestamp.tm_sec = 0 ;

  char roundtime_formated[40];
  strftime(roundtime_formated, 40, "%m-%d-%Y %H:%M", &roundtime_timestamp); 
 
  int roundtime_timestamp_int = int(mktime(&roundtime_timestamp));
  print.logqq("round timestamp: ", roundtime_timestamp_int);
  print.logqq(String(roundtime_formated));

  



  if(ping(print)){
    print.logq("Making json");

  
    //time header

    String time_header = String(_year) + "/" + String(roundtime_formated)+ "/" ;

    //atmospheric
    data_json.set("Atmospheric/DryTemperature", double(dry_bulb_temp),2);
    data_json.set("Atmospheric/RelativeHumidity", double(relative_humidity),2);
    data_json.set("Atmospheric/BarometricPressure", double(barometric_pressure)/1000.00,2);
    data_json.set("Atmospheric/PressureAltitude", double(pressure_altitude),1);

    //NeoLinkState

    data_json.set("NeoLinkState/SolarVoltage",double(solar_voltage),2);
    data_json.set("NeoLinkState/BatteryVoltage", double(battery_voltage),3);
    data_json.set("NeoLinkState/InternalTemperature", double(internal_temperature),2);

    //timestamp

    data_json.FirebaseJson::set("RoundTs", roundtime_timestamp_int);
    data_json.FirebaseJson::set("RealTs/.sv", "timestamp");

    
    
    String buff_string;
    data_json.toString(buff_string,true);
    Serial.println(buff_string); 
    
    
    //ServicesStatus

  
  status_json.FirebaseJson::set("LastUpdate", String(roundtime_formated) );
  status_json.FirebaseJson::set("FirmwareVersion", firmware_version);
  status_json.set("SolarVoltage",double(solar_voltage),2);
  status_json.set("BatteryVoltage", double(battery_voltage),3);
  status_json.set("InternalTemperature", double(internal_temperature),2);

  
  status_json.toString(buff_string,true);
  Serial.println(buff_string); 

  print.logqq("Done.");

  print.logq("sending jsons to cloud");
        
  //status_json.FirebaseJson::set("LastRestart", String(restart_time_formated) );

  Firebase.setJSON(firebasedata , "/ServicesDataset/NeoLink/" + SN + "/" + time_header , data_json );
  Firebase.setJSON(firebasedata , "/ServicesStatus/NeoLink/" + SN , status_json );

  print.logqq("Done.");
    
   return true;

  }

  else print.logqq("send to cloud failed.");

  return false;
  


}



#define SEALEVELPRESSURE_HPA 1015.85

void get_atmos(vprint print){

  print.logq("Reading atmosferical sensor.");

  long unsigned timestamp = millis();
  bool bme_status = bme_static.begin(0x76);
  if(!bme_status) print.logqq("Atmos failed. Sensor ID is: 0x", String(int(bme_static.sensorID()),HEX) );
  
  while(bme_static.readPressure()<5000 && millis() - timestamp < 15000) delay(200); 

  if ( isnan(bme_static.readHumidity())) {
    print.logqqq("NaN detected"); 
    digitalWrite(TEMP_EN, LOW);
    delay(1000);
    digitalWrite(TEMP_EN, HIGH);
    delay(1000);
    bme_status = bme_static.begin(0x76);
    if(!bme_status) print.logqq("Atmos failed. Sensor ID is: 0x", String(int(bme_static.sensorID()),HEX) );
    while(bme_static.readPressure()<5000 && millis() - timestamp < 15000) delay(200); 
  } 

  if(isnan(bme_static.readHumidity())) { 
    print.logqq("ERROR Atmos NaN"); 
    digitalWrite(TEMP_EN, LOW); 
    delay(500);
    deepsleep(1,print);}

  if(dry_bulb_temp == -100 || barometric_pressure == -100 || relative_humidity == -100 || pressure_altitude == -100  )
      { 
        dry_bulb_temp = bme_static.readTemperature();
        barometric_pressure = bme_static.readPressure() ;
        relative_humidity = bme_static.readHumidity();
        pressure_altitude = bme_static.readAltitude(SEALEVELPRESSURE_HPA);}

  else {  
    dry_bulb_temp = (bme_static.readTemperature() + dry_bulb_temp)/2;
    barometric_pressure = (bme_static.readPressure() + barometric_pressure )/2;
    relative_humidity = (bme_static.readHumidity() + relative_humidity)/2;
    pressure_altitude = (bme_static.readAltitude(SEALEVELPRESSURE_HPA) + pressure_altitude)/2;}

  print.logqq("dry_bulb_temp: ", String(dry_bulb_temp,3));
  print.logqq("barometric_pressure: ", String(barometric_pressure/1000.000,3));
  print.logqq("relative_humidity: ", String(relative_humidity,3));
  print.logqq("pressure_altitude: ", String(pressure_altitude,1));

  digitalWrite(TEMP_EN, LOW);
  
  return;
      
      
}
 
String* get_firebase_json_str(vprint print){
  /*get json from firebase as ordered string*/
  
  String buffer;
  
  FirebaseJson  &firebase_json = firebasedata.jsonObject();
  
  firebase_json.toString(buffer,true);
  //print.logqqq("json: \n",buffer);
   
  size_t len = firebase_json.iteratorBegin();
  String* result = new String[2*len+1];

  String key, value = "";
  int type = 0;
  int i1,i2;
  for (size_t i = 0; i < len; i++)
        {   i1=2*i+1;
            i2=i1+1;
            firebase_json.iteratorGet(i, type, key, value);
            result [i1] =  key;
            result [i2] = value;
          
        }
  firebase_json.iteratorEnd();
  result[0] = 2*len;
return result;

}

String get_value_json_str(String* response, String variable_required){


  for (size_t i = 1; i < response[0].toInt(); i++)
  {
      //Serial.println("THIS > " +response[i]);
      
    if(response[i] == variable_required) {
      
      return response[i+1];}
      
  }

}

bool get_SN_n_MAC(vprint print){
//return true if SN is compatible; otherwise False

bool compatible_flag;

  print.logq("Reading SN and MAC:");

  DEVICE_HEADER = read_eeprom( 0 , 2 );
  HARDWARE_VERSION = read_eeprom( 2 , 2);
  ENVIRONMENT = read_eeprom( 4 , 1 );
  DATE_CORRELATIVE = read_eeprom( 5 , 4);
  SN_CORRELATIVE = read_eeprom( 9 , 4);

  /*Serial.println(DEVICE_HEADER);
  Serial.println(HARDWARE_VERSION);
  Serial.println(ENVIRONMENT);
  Serial.println(DATE_CORRELATIVE);
  Serial.println(SN_CORRELATIVE);
   
  Serial.println(DEVICE_HEADER_FIRMWARE);
  Serial.println(HARDWARE_VERSION_FIRMWARE);*/

  SN = DEVICE_HEADER+HARDWARE_VERSION+ENVIRONMENT+DATE_CORRELATIVE+SN_CORRELATIVE;

  //Serial.println(SN);
  
  if((DEVICE_HEADER + HARDWARE_VERSION)==(DEVICE_HEADER_FIRMWARE+HARDWARE_VERSION_FIRMWARE)) {

    print.logqq("Firmware compatible with this device version.");
    print.logqq("SN: ", SN);
    compatible_flag = true;

    
    }
  else {
    print.logqq("SN not for this firmware. Updating SN in a while. SN: ", SN );
    compatible_flag = false;
  }

  uint64_t chipid;
  chipid = ESP.getEfuseMac(); //The chip ID is its MAC address(length: 6 bytes).
  chipid_str = String((uint16_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);

  print.logqq("Chip ID: ", chipid_str);

  return compatible_flag;


} 

bool ping(vprint print){

  print.logq("ping...");
  if(Firebase.getBool(firebasedata, "/SN_Chips/Ping")){
    print.logqq("success.");
    return true;

  } else {
    print.logqq("Fail.");

    print.logq("ping again...");
    if(Firebase.getBool(firebasedata, "/SN_Chips/Ping")){
        print.logqq("success.");
        return true;

      } else {
        print.logqq("Fail. Resetting NO_PING. ");
        return false;}



    return false;}


}  

void create_SN_nodes(vprint print) {
  // must executed in a different program
  print.logq("Creating SN nodes for " + chipid_str);

  neoFirebaseJson json_SN_nodes;
  json_SN_nodes.FirebaseJson::set("DEVICE_HEADER", chipid_str); 
  json_SN_nodes.FirebaseJson::set("HARDWARE_VERSION","00");
  json_SN_nodes.FirebaseJson::set("ENVIRONMENT", "D");
  json_SN_nodes.FirebaseJson::set("DATE_CORR_AAMM", "2200");
  json_SN_nodes.FirebaseJson::set("SN_CORR", "9999");

  String buff_string_SN_nodes;
  json_SN_nodes.toString(buff_string_SN_nodes,true);
  print.logqq(buff_string_SN_nodes); 

  Firebase.updateNode(firebasedata, "/SN_Chips/" + chipid_str + "/", json_SN_nodes );
  print.logqq("Done.");
  handleCommands("SoftReset");



}

void create_nodes(vprint print) {

  neoFirebaseJson json_node_controllers_flag;
  neoFirebaseJson json_node_services_config;
  //checking if default nodes exist, otherwise...
  //19.06 15:19 is absurd. I decided to create the default nodes in every case if SN changes.
  print.logq("Creating nodes with default values for " + SN);

  
  json_node_controllers_flag.FirebaseJson::set("local_specific_firmware_target", false); 
  json_node_controllers_flag.FirebaseJson::set("local_specific_firmware_version", firmware_version);

  json_node_controllers_flag.FirebaseJson::set("update_config_flag", true); 

  
  json_node_services_config.FirebaseJson::set("LATITUDE", -8.078078); 
  json_node_services_config.FirebaseJson::set("LONGITUDE", -79.122082); 
  json_node_services_config.FirebaseJson::set("BAT_H", 3.4); 
  json_node_services_config.FirebaseJson::set("BAT_L", 3.2); 
  json_node_services_config.FirebaseJson::set("BEEP_INIT", false); 
  json_node_services_config.FirebaseJson::set("BEEP_END", false); 
  json_node_services_config.FirebaseJson::set("DAY_SLEEP_HOUR", 8); 
  json_node_services_config.FirebaseJson::set("DAY_SLEEPTIME", 15); //minutes
  json_node_services_config.FirebaseJson::set("NIGHT_SLEEP_HOUR", 20); 
  json_node_services_config.FirebaseJson::set("NIGHT_SLEEPTIME", 15); //minutes
  json_node_services_config.FirebaseJson::set("PORT_RQ", false); 
  json_node_services_config.FirebaseJson::set("ATMOS_RQ", false); 
  json_node_services_config.FirebaseJson::set("SD_ENABLE", false); 
  
  json_node_services_config.FirebaseJson::set("PORTS/PORT_1/PORT_1_ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_1/DEPTH_1A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_1/DEPTH_1B", 0); 

  json_node_services_config.FirebaseJson::set("PORTS/PORT_2/PORT_2_ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_2/DEPTH_2A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_2/DEPTH_2B", 0); 

  json_node_services_config.FirebaseJson::set("PORTS/PORT_3/PORT_3_ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_3/DEPTH_3A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_3/DEPTH_3B", 0); 

  json_node_services_config.FirebaseJson::set("PORTS/PORT_4/PORT_4_ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_4/DEPTH_4A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_4/DEPTH_4B", 0); 

  json_node_services_config.FirebaseJson::set("PORTS/PORT_5/PORT_5_ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_5/DEPTH_5A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_5/DEPTH_5B", 0); 

  json_node_services_config.FirebaseJson::set("PORTS/PORT_6/PORT_6_ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_6/DEPTH_6A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_6/DEPTH_6B", 0); 


  
  Firebase.updateNode(firebasedata, "/Services_config/NeoLink/" + SN + "/", json_node_services_config );

  print.logqq("Creating nodes in Services_controllers.");

  for (int i = 1; !Firebase.updateNode(firebasedata, "/Services_controllers_Flags/NeoLink/" + SN + "/", json_node_controllers_flag ); i++)  i>3 ? deepsleep(NO_INTERNET_DESPITE_PING, print, "Attemps Failed.") : print.logq(" Updating failed. Attemp: ", i);
  
  print.logqq("Done.");
  print.logqq("Creating nodes in Services_config.");

  for (int i = 1; !Firebase.updateNode(firebasedata,"/Services_config/NeoLink/" + SN + "/", json_node_services_config ); i++)  i>3 ? deepsleep(NO_INTERNET_DESPITE_PING, print, "Attemps Failed.") : print.logq(" Updating failed. Attemp: ", i);
  print.logqq("Done.");  


}

void update_SN(vprint print , bool compatible_fw_flag){

  print.logq("Checking for SN json updates.");

  //getting sn_update_flag 
  //if update_sn_flag = 1 and SN ! = SN from firebase. Update eeprom
  //A problem appears when get json in firebase data fail and it create new nodes. Is better to have a
  //initial settings program. 
  int sn_update_flag;
  String default_sn = "0000000000000";
  int sn_as_default = 0;

  String json_definition_origin =  "/SN_Chips/" + chipid_str + "/";

  

  if(SN == default_sn ) {
    print.logqq ("Current SN is default_sn. Checking if sn_update_flag is set. Otherwise will reset.");
    sn_as_default = 1;
  } 
  
  //else{

  for (int i = 1; ! Firebase.getInt(firebasedata, json_definition_origin + "SN_UPDATE_FLAG") ; i++) i>4 ? deepsleep(NO_INTERNET_DESPITE_PING, print, "Attemps Failed.") : print.logq("Getting local_update_firmware failed. Attemp: ", i);
   sn_update_flag = firebasedata.intData();
  
  //}

  if( !sn_update_flag && sn_as_default ) {
     print.logqq("SN has the default value and sn_update is not set. Resetting..");
     if(MODE_PRG) handleCommands("SoftReset");
     else deepsleep( DEFAULT_SN_SLEEPTIME, print, "Sleeping DEFAULT_SN_SLEEPTIME");
  }

  if(sn_update_flag ){
    
    for (int i = 1; !Firebase.get(firebasedata, json_definition_origin) ; i++) i>3 ? deepsleep(NO_INTERNET_DESPITE_PING, print, "Attemps Failed.") : print.logq("Getting jsondata failed. Attemp: ", i);
  
    String* response = get_firebase_json_str(print);
    String fb_DEVICE_HEADER = get_value_json_str(response, "DEVICE_HEADER");
    String fb_HARDWARE_VERSION= get_value_json_str(response, "HARDWARE_VERSION");
    String fb_ENVIRONMENT= get_value_json_str(response, "ENVIRONMENT");
    String fb_DATE_CORR_AAMM= get_value_json_str(response, "DATE_CORR_AAMM");
    String fb_SN_CORR= get_value_json_str(response, "SN_CORR");

    if(fb_DEVICE_HEADER == "00") print.logqq("Not update because the SN in Firebase has the default value chipid");
   
    else{

    String fb_SN = fb_DEVICE_HEADER + fb_HARDWARE_VERSION + fb_ENVIRONMENT + fb_DATE_CORR_AAMM + fb_SN_CORR;

    
    if(fb_SN == SN) print.logqq("Nothing change because SN parameters in firebase is the same.") ;

    else{
      print.logqq("Firebase SN: ", fb_SN);
      print.logqq("EEPROM SN: ", SN);
      print.logqq("Updating SN and saving in EEPROM.");
      SN = fb_SN;
      write_eeprom(SN, 0, print);
          
      print.logqq("Creating nodes:");  
      create_nodes(print); 
     

    }

    
    }

  print.logqq("Setting SN_UPDATE_FLAG to 0");
  for (int i = 1; !Firebase.setInt(firebasedata,  json_definition_origin + "SN_UPDATE_FLAG", 0) ; i++) i>3 ? deepsleep(NO_INTERNET_DESPITE_PING, print, "Attemps Failed.") : print.logq(" Updating failed. Attemp: ", i);
  

  }
  else print.logqq("SN update not required.");

}

void read_controllers_flags(vprint print){

print.logq("Reading Services_controller_Flags:");

  
  
  String global_specific_firmware_version;
  const String Global_update_path = "/Services_controllers_Flags/Global/NeoLink";
  bool global_json_obtained = Firebase.get(firebasedata, Global_update_path);
   

  
  print.logqq("Reading global flags.");

  if( global_json_obtained ){
    
    String* response = get_firebase_json_str(print);

    global_specific_firmware_version = get_value_json_str(response, "global_specific_firmware_version");

    print.logqqq("Done.");
   
  } else print.logqq("Unsuccessful.");


  bool local_specific_firmware_target = false;
  String local_specific_firmware_version;
  String Local_update_path = "/Services_controllers_Flags/NeoLink/" + SN;
  bool local_json_obtained = Firebase.get(firebasedata, Local_update_path);
  bool update_config_flag = false;
  

  print.logqq("Reading local controller flags.");

  if( local_json_obtained ){
    
    String* response = get_firebase_json_str(print);
    
    local_specific_firmware_target = str_to_bool( get_value_json_str(response, "local_specific_firmware_target"), print);
    local_specific_firmware_version = get_value_json_str(response, "local_specific_firmware_version");
    update_config_flag = str_to_bool(get_value_json_str(response, "update_config_flag"), print);

    print.logqqq("Done.");
   
  } else print.logqqq("Unsuccessful.");

  print.logqq("update_config_flag is " + update_config_flag? "True":"False");

  if(update_config_flag){
      print.logqq("Reading and Saving in eeprom new configuration from Services_config.");
      get_firebase_configuration(print);
      
    } else print.logqq("No configuration updating will performed.");



  

  if(!local_specific_firmware_target){
    print.logqq("Firmware will target the global version.");
    firmware_update_manager(0, global_specific_firmware_version, print); // Update_mode 1.

  }

  else if (local_specific_firmware_target){
    print.logqq("Firmware will target the specific version.");
    firmware_update_manager(1, local_specific_firmware_version, print); // Update_mode 3.

  }

  else print.logqq("No update will perform.");

  
  


}

void read_configuration(vprint print){

  print.logq("Reading configurations in eeprom.");

  ATMOS_RQ = EEPROM.readBool(ATMOS_RQ_eeprom);
  BAT_H = EEPROM.readFloat(BAT_H_eeprom);
  BAT_L = EEPROM.readFloat(BAT_L_eeprom);
  BEEP_END = EEPROM.readBool(BEEP_END_eeprom);
  BEEP_INIT = EEPROM.readBool(BEEP_INIT_eeprom);
  DAY_SLEEPTIME = EEPROM.readInt(DAY_SLEEPTIME_eeprom);
  DAY_SLEEP_HOUR = EEPROM.readInt(DAY_SLEEP_HOUR_eeprom);
  LATITUDE = EEPROM.readFloat(LATITUDE_eeprom);
  LONGITUDE = EEPROM.readFloat(LONGITUDE_eeprom);
  NIGHT_SLEEPTIME = EEPROM.readInt(NIGHT_SLEEPTIME_eeprom);
  NIGHT_SLEEP_HOUR = EEPROM.readInt(NIGHT_SLEEP_HOUR_eeprom);
  PORT_RQ = EEPROM.readBool(PORT_RQ_eeprom);
  SD_ENABLE = EEPROM.readBool(SD_ENABLE_eeprom);
  
  PORT_1_ENABLE = EEPROM.readBool(PORT_1_ENABLE_eeprom); 
  DEPTH_1A = EEPROM.readInt(DEPTH_1A_eeprom);
  DEPTH_1B = EEPROM.readInt(DEPTH_1B_eeprom);

  PORT_2_ENABLE = EEPROM.readBool(PORT_2_ENABLE_eeprom);
  DEPTH_2A = EEPROM.readInt(DEPTH_2A_eeprom);
  DEPTH_2B = EEPROM.readInt(DEPTH_2B_eeprom);

  PORT_3_ENABLE = EEPROM.readBool(PORT_3_ENABLE_eeprom);
  DEPTH_3A = EEPROM.readInt(DEPTH_3A_eeprom);
  DEPTH_3B = EEPROM.readInt(DEPTH_3B_eeprom);

  PORT_4_ENABLE = EEPROM.readBool(PORT_4_ENABLE_eeprom);
  DEPTH_4A = EEPROM.readInt(DEPTH_4A_eeprom);
  DEPTH_4B = EEPROM.readInt(DEPTH_4B_eeprom);

  PORT_5_ENABLE = EEPROM.readBool(PORT_5_ENABLE_eeprom);
  DEPTH_5A = EEPROM.readInt(DEPTH_5A_eeprom);
  DEPTH_5B = EEPROM.readInt(DEPTH_5B_eeprom);

  PORT_6_ENABLE = EEPROM.readBool(PORT_6_ENABLE_eeprom); 
  DEPTH_6A = EEPROM.readInt(DEPTH_6A_eeprom);
  DEPTH_6B = EEPROM.readInt(DEPTH_6B_eeprom);

  /*
  print.logqq("ATMOS_RQ: ", ATMOS_RQ ? "true":"false" );
  delay(20);
  print.logqq("BAT_H: ", String(BAT_H,3));
  delay(20);
  print.logqq("BAT_L: ", String(BAT_L,3));
  delay(20);
  print.logqq("BEEP_END: ", BEEP_END ? "true":"false" );
  delay(20);
  print.logqq("BEEP_INIT: ", BEEP_INIT ? "true":"false");
  delay(20);
  print.logqq("DAY_SLEEPTIME: ", String(DAY_SLEEPTIME));
  delay(20);
  print.logqq("DAY_SLEEP_HOUR: ", String(DAY_SLEEP_HOUR));
  delay(20);
  print.logqq("LATITUDE: ", String(LATITUDE,8));
  delay(20);
  print.logqq("LONGITUDE: ", String(LONGITUDE,8));
  delay(20);
  print.logqq("NIGHT_SLEEPTIME: ", String(NIGHT_SLEEPTIME));
  delay(20);
  print.logqq("NIGHT_SLEEP_HOUR: ", String(NIGHT_SLEEP_HOUR));
  delay(20);
  print.logqq("PORT_RQ: ", PORT_RQ ? "true":"false" );
  delay(20);
  print.logqq("SD_ENABLE: ", SD_ENABLE ? "true":"false" );
  delay(20);//because WS_MAX_QUEUED_MESSAGES in asyn TCP
  print.logqq("PORT_1_ENABLE: ", PORT_1_ENABLE ? "true":"false");
  delay(20);
  print.logqq("DEPTH_1A: ", String(DEPTH_1A));
  delay(20);
  print.logqq("DEPTH_1B: ", String(DEPTH_1B));
  delay(20);
  print.logqq("PORT_2_ENABLE: ", PORT_2_ENABLE ? "true":"false");
  delay(20);
  print.logqq("DEPTH_2A: ", String(DEPTH_2A));
  delay(20);
  print.logqq("DEPTH_2B: ", String(DEPTH_2B));
  delay(20);
  print.logqq("PORT_3_ENABLE: ", PORT_3_ENABLE ? "true":"false");
  delay(20);
  print.logqq("DEPTH_3A: ", String(DEPTH_3A));
  delay(20);
  print.logqq("DEPTH_3B: ", String(DEPTH_3B));
  delay(20);
  print.logqq("PORT_4_ENABLE: ", PORT_4_ENABLE ? "true":"false");
  delay(20);
  print.logqq("DEPTH_4A: ", String(DEPTH_4A));
  delay(20);
  print.logqq("DEPTH_4B: ", String(DEPTH_4B));
  delay(20);
  print.logqq("PORT_5_ENABLE: ", PORT_5_ENABLE ? "true":"false");
  delay(20);
  print.logqq("DEPTH_5A: ", String(DEPTH_5A));
  delay(20);
  print.logqq("DEPTH_5B: ", String(DEPTH_5B));
  delay(20);
  print.logqq("PORT_6_ENABLE: ", PORT_6_ENABLE ? "true":"false");
  delay(20);
  print.logqq("DEPTH_6A: ", String(DEPTH_6A));
  delay(20);
  print.logqq("DEPTH_6B: ", String(DEPTH_6B));
  delay(20);
  
  delay(20);*/
  print.logqq("Success.");

}

void get_firebase_configuration(vprint print){

  //see https://docs.google.com/spreadsheets/d/1y-FEoacuGQVNXTLEPXn3_XlkA6E7z7mzDyGfZqTUZtc/edit#gid=1587879252
  // TODO: must create const variables for the eeprom constat locations

const String configurations_path = "/Services_config/NeoLink/" + SN;


if( Firebase.get(firebasedata, configurations_path)) {
  print.logq("New configuration received.");
  String* response = get_firebase_json_str(print);

  //for (size_t i = 1; i < response[0].toInt(); i++) print.logqq( String(int(i)), response[i]);
  
  ATMOS_RQ = str_to_bool( get_value_json_str(response, "ATMOS_RQ"), print);
  BAT_H = get_value_json_str(response, "BAT_H").toFloat();
  BAT_L = get_value_json_str(response, "BAT_L").toFloat();
  BEEP_END = str_to_bool( get_value_json_str(response, "BEEP_END"), print);
  BEEP_INIT = str_to_bool( get_value_json_str(response, "BEEP_INIT"), print);
  DAY_SLEEPTIME = get_value_json_str(response, "DAY_SLEEPTIME").toInt();
  DAY_SLEEP_HOUR = get_value_json_str(response, "DAY_SLEEP_HOUR").toInt();
  LATITUDE = get_value_json_str(response, "LATITUDE").toFloat();  
  LONGITUDE = get_value_json_str(response, "LONGITUDE").toFloat();
  NIGHT_SLEEPTIME = get_value_json_str(response, "NIGHT_SLEEPTIME").toInt();
  NIGHT_SLEEP_HOUR = get_value_json_str(response, "NIGHT_SLEEP_HOUR").toInt();
  PORT_RQ = str_to_bool( get_value_json_str(response, "PORT_RQ"), print);
  SD_ENABLE = str_to_bool( get_value_json_str(response, "SD_ENABLE"), print);
    
  PORT_1_ENABLE = str_to_bool( get_value_json_str(response, "PORT_1_ENABLE"), print);
  DEPTH_1A = get_value_json_str(response, "DEPTH_1A").toInt();
  DEPTH_1B = get_value_json_str(response, "DEPTH_1B").toInt();

  PORT_2_ENABLE = str_to_bool( get_value_json_str(response, "PORT_2_ENABLE"), print);
  DEPTH_2A = get_value_json_str(response, "DEPTH_2A").toInt();
  DEPTH_2B = get_value_json_str(response, "DEPTH_2B").toInt();

  PORT_3_ENABLE = str_to_bool( get_value_json_str(response, "PORT_3_ENABLE"), print);
  DEPTH_3A = get_value_json_str(response, "DEPTH_3A").toInt();
  DEPTH_3B = get_value_json_str(response, "DEPTH_3B").toInt();

  PORT_4_ENABLE = str_to_bool( get_value_json_str(response, "PORT_4_ENABLE"), print);
  DEPTH_4A = get_value_json_str(response, "DEPTH_4A").toInt();
  DEPTH_4B = get_value_json_str(response, "DEPTH_4B").toInt();
  
  PORT_5_ENABLE = str_to_bool( get_value_json_str(response, "PORT_5_ENABLE"), print);
  DEPTH_5A = get_value_json_str(response, "DEPTH_5A").toInt();
  DEPTH_5B = get_value_json_str(response, "DEPTH_5B").toInt();
  
  PORT_6_ENABLE = str_to_bool( get_value_json_str(response, "PORT_6_ENABLE"), print);
  DEPTH_6A = get_value_json_str(response, "DEPTH_6A").toInt();
  DEPTH_6B = get_value_json_str(response, "DEPTH_6B").toInt();
  
  
 
  

  if(ATMOS_RQ != EEPROM.readBool(ATMOS_RQ_eeprom)) EEPROM.writeBool(ATMOS_RQ_eeprom, ATMOS_RQ);
  if(BAT_H != EEPROM.readFloat(BAT_H_eeprom)) EEPROM.writeFloat(BAT_H_eeprom, BAT_H);
  if(BAT_L != EEPROM.readFloat(BAT_L_eeprom)) EEPROM.writeFloat(BAT_L_eeprom, BAT_L);
  if(BEEP_END != EEPROM.readBool(BEEP_END_eeprom)) EEPROM.writeBool(BEEP_END_eeprom, BEEP_END);
  if(BEEP_INIT != EEPROM.readBool(BEEP_INIT_eeprom)) EEPROM.writeBool(BEEP_INIT_eeprom, BEEP_INIT);
  if(DAY_SLEEPTIME != EEPROM.readInt(DAY_SLEEPTIME_eeprom)) EEPROM.writeInt(DAY_SLEEPTIME_eeprom, DAY_SLEEPTIME);
  if(DAY_SLEEP_HOUR != EEPROM.readInt(DAY_SLEEP_HOUR_eeprom)) EEPROM.writeInt(DAY_SLEEP_HOUR_eeprom, DAY_SLEEP_HOUR);
  if(LATITUDE != EEPROM.readFloat(LATITUDE_eeprom)) EEPROM.writeFloat(LATITUDE_eeprom, LATITUDE);
  if(LONGITUDE != EEPROM.readFloat(LONGITUDE_eeprom)) EEPROM.writeFloat(LONGITUDE_eeprom, LONGITUDE);
  if(NIGHT_SLEEPTIME != EEPROM.readInt(NIGHT_SLEEPTIME_eeprom)) EEPROM.writeInt(NIGHT_SLEEPTIME_eeprom, NIGHT_SLEEPTIME);
  if(NIGHT_SLEEP_HOUR != EEPROM.readInt(NIGHT_SLEEP_HOUR_eeprom)) EEPROM.writeInt(NIGHT_SLEEP_HOUR_eeprom, NIGHT_SLEEP_HOUR);  
  if(PORT_RQ != EEPROM.readBool(PORT_RQ_eeprom)) EEPROM.writeBool(PORT_RQ_eeprom, PORT_RQ);
  if(SD_ENABLE != EEPROM.readBool(SD_ENABLE_eeprom)) EEPROM.writeBool(SD_ENABLE_eeprom, SD_ENABLE);
  
  if(PORT_1_ENABLE != EEPROM.readBool(PORT_1_ENABLE_eeprom)) EEPROM.writeBool(PORT_1_ENABLE_eeprom, PORT_1_ENABLE);  
  if(DEPTH_1A != EEPROM.readInt(DEPTH_1A_eeprom)) EEPROM.writeInt(DEPTH_1A_eeprom, DEPTH_1A);
  if(DEPTH_1B != EEPROM.readInt(DEPTH_1B_eeprom)) EEPROM.writeInt(DEPTH_1B_eeprom, DEPTH_1B);

  if(PORT_2_ENABLE != EEPROM.readBool(PORT_2_ENABLE_eeprom)) EEPROM.writeBool(PORT_2_ENABLE_eeprom, PORT_2_ENABLE);  
  if(DEPTH_2A != EEPROM.readInt(DEPTH_2A_eeprom)) EEPROM.writeInt(DEPTH_2A_eeprom, DEPTH_2A);
  if(DEPTH_2B != EEPROM.readInt(DEPTH_2B_eeprom)) EEPROM.writeInt(DEPTH_2B_eeprom, DEPTH_2B);

  if(PORT_3_ENABLE != EEPROM.readBool(PORT_3_ENABLE_eeprom)) EEPROM.writeBool(PORT_3_ENABLE_eeprom, PORT_3_ENABLE);  
  if(DEPTH_3A != EEPROM.readInt(DEPTH_3A_eeprom)) EEPROM.writeInt(DEPTH_3A_eeprom, DEPTH_3A);
  if(DEPTH_3B != EEPROM.readInt(DEPTH_3B_eeprom)) EEPROM.writeInt(DEPTH_3B_eeprom, DEPTH_3B);

  if(PORT_4_ENABLE != EEPROM.readBool(PORT_4_ENABLE_eeprom)) EEPROM.writeBool(PORT_4_ENABLE_eeprom, PORT_4_ENABLE);  
  if(DEPTH_4A != EEPROM.readInt(DEPTH_4A_eeprom)) EEPROM.writeInt(DEPTH_4A_eeprom, DEPTH_4A);
  if(DEPTH_4B != EEPROM.readInt(DEPTH_4B_eeprom)) EEPROM.writeInt(DEPTH_4B_eeprom, DEPTH_4B);

  if(PORT_5_ENABLE != EEPROM.readBool(PORT_5_ENABLE_eeprom)) EEPROM.writeBool(PORT_5_ENABLE_eeprom, PORT_5_ENABLE);  
  if(DEPTH_5A != EEPROM.readInt(DEPTH_5A_eeprom)) EEPROM.writeInt(DEPTH_5A_eeprom, DEPTH_5A);
  if(DEPTH_5B != EEPROM.readInt(DEPTH_5B_eeprom)) EEPROM.writeInt(DEPTH_5B_eeprom, DEPTH_5B);
  
  if(PORT_6_ENABLE != EEPROM.readBool(PORT_6_ENABLE_eeprom)) EEPROM.writeBool(PORT_6_ENABLE_eeprom, PORT_6_ENABLE);  
  if(DEPTH_6A != EEPROM.readInt(DEPTH_6A_eeprom)) EEPROM.writeInt(DEPTH_6A_eeprom, DEPTH_6A);
  if(DEPTH_6B != EEPROM.readInt(DEPTH_6B_eeprom)) EEPROM.writeInt(DEPTH_6B_eeprom, DEPTH_6B);

  EEPROM.commit();


  print.logqq("Saved in EEPROM.");



} else print.logqq("No setting configuration detected.");

print.logqq("Turning update_config_flag off.");
for (int i = 1; !Firebase.setBool(firebasedata, "/Services_controllers_Flags/NeoLink/" + SN + "/update_config_flag" , false) ; i++) i>3 ? deepsleep(NO_INTERNET_DESPITE_PING, print, "Attemps Failed.") : print.logq(" Updating failed. Attemp: ", i);
  

}

void write_eeprom(String string_to_eeprom, int position, vprint print){

  
  for (size_t i = 0; i <  string_to_eeprom.length() ; i++) EEPROM.write(i+position,string_to_eeprom[i]);
  EEPROM.commit(); 

  print.logqq("\tDone. Bytes saved: ", int (string_to_eeprom.length()));
  
  
}

String read_eeprom(int init, int len){

  char eeprom_to_read[100];

  for( int i=0; i< len ; i++ ){
    eeprom_to_read[i] = char(EEPROM.read(i+init));
    /*
    Serial.print(i);
    Serial.print(": ");
    Serial.print(eeprom_to_read[i]);
    Serial.print(": ");
    Serial.println(String(EEPROM.read(i+init)));
    */
  } 
  eeprom_to_read[len]='\0';

  return String(eeprom_to_read);
}

void print_controller_status(vprint print){
  print.logq("Running in modes:");
   
    if(HARDWARE_AVAILABLE) print.logqq("[Running with Hadware]");
    else  print.logqq("[Running without Hadware]");

    if(MODE_PRG) print.logqq("[PRG Mode enabled]");
    
    if(LOCAL_SERVER) print.logqq("[Local Server enabled]");

    if(FIRMWARE_MODE == 'DEV') print.logqq("[Firmware mode DEV]");
    else ("[Firmware mode PROD]");

}

void starting_wifi(vprint print) {
  int wifi_try = 1;


    print.logq("Starting WiFi:");

    WiFi.begin(WIFI_SSID, WIFI_PSSWD);
    
    init_timestamp = millis();
    Serial.print("\t\t");
    while (1) {
      while (WiFi.status() != WL_CONNECTED && millis() - init_timestamp < WIFI_TIME_LIMIT ) {
        Serial.print(".");
        delay(500);
      }
      if (WiFi.status() == WL_CONNECTED) {
        if(LOCAL_SERVER) delay(5000);
        print.logqq("Connected to " + String(WIFI_SSID));
        print.logqq("IP address: "+ WiFi.localIP().toString());

        Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
        Firebase.reconnectWiFi(true);

        
     
        break;

      }

      else {

        init_timestamp = millis();
        WiFi.begin(WIFI_SSID, WIFI_PSSWD);

        print.logqq("ERROR: "+ String(WiFi.status()) +  ". Attempt: " +  String(wifi_try) );

        wifi_try++;

        if (wifi_try > 3) {

          print.logqq("WiFi refuses connection. Sleeping..");
          Stage = 1;
          deepsleep(1,print);
        }

       

      }
    }

  

}

void SendMsgToClient(String command, vprint print){
  
  ws.textAll(command);
  print.logq("Sending Command: " + command)  ;
  
}

void handleCommands(String data){

  if(data == "SoftReset") {
    SendMsgToClient("[RESET]",print);
    delay(1000);
    deepsleep(1,print);}
  else Serial.print("No command.");

}

void handleCommands(String data, int timeToSleep){

if(data == "SoftReset") {
  SendMsgToClient("[RESET]",print);
  delay(1000);
  deepsleep(timeToSleep,print);}
else Serial.print("No command.");

}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
 
    //Serial.println("Websocket client connection received");
    client->text("\nGetting Data:");
 
  } else if(type == WS_EVT_DISCONNECT){
    Serial.println("Client disconnected");
   
    
    } else if(type == WS_EVT_DATA){
       AwsFrameInfo *info = (AwsFrameInfo*)arg;
       if(info->final && info->index == 0 && info->len == len){

          if(info->opcode == WS_TEXT){
        data[len] = 0;
        String str_data = String((char*)data);
        Serial.println( "Comando: " + str_data );
        handleCommands(str_data);
      }


       }
    }

}

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    // handle upload and update
    if (!index)
    {
        Serial.printf("Update: %s\n", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN))
        { //start with max available size
            Update.printError(Serial);
        }
    }

    /* flashing firmware to ESP*/
    if (len)
    {
        Update.write(data, len);
    }

    if (final)
    {
        if (Update.end(true))
        { //true to set the size to the current progress
            Serial.printf("Update Success: %ub written\nRebooting...\n", index+len);
        }
        else
        {
            Update.printError(Serial);
        }
    }
    // alternative approach
    // https://github.com/me-no-dev/ESPAsyncWebServer/issues/542#issuecomment-508489206
}

void start_server(vprint print){

 /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  delay(1000);

  //Serial.println("Server started: http://esp32.local/"); //http://esp32.local/
  print.logq("Server started: http://esp32.local/");  /*return index page which is stored in serverIndex */


  server.on("/", xHTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", serverIndex);
  });

  
      

  
server.on("/update", xHTTP_POST, [](AsyncWebServerRequest *request) {
        
            AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
            response->addHeader("Connection", "close");
            request->send(response);
            ESP.restart();
         }, handleUpload);


    


 
  server.begin();

}

int checking_battery(vprint print) {
  
  print.logq("Checking Battery: ");

  pinMode(BAT_VALUE, INPUT_PULLDOWN);
  pinMode(SOLAR_VALUE, INPUT_PULLDOWN);

  sensors.begin();

  digitalWrite(BAT_SOLAR_EN, HIGH);
  delay(20);

  float solar_voltage_temp;
  float battery_voltage_temp;
  float internal_temperature_temp;

  solar_voltage_temp  = ReadVoltage(SOLAR_VALUE) ;
  battery_voltage_temp = ReadVoltage(BAT_VALUE) ;

  sensors.requestTemperatures(); 
  internal_temperature_temp = sensors.getTempCByIndex(0);

  digitalWrite(BAT_SOLAR_EN, LOW);

  if (battery_voltage <= 0 )battery_voltage = battery_voltage_temp;
  if ( solar_voltage <= 0) solar_voltage = solar_voltage_temp ;
  if ( internal_temperature <= 0) internal_temperature = internal_temperature_temp ;
    
  battery_voltage = (battery_voltage_temp + battery_voltage ) / 2 ;
  solar_voltage = (solar_voltage_temp + solar_voltage) / 2;
  internal_temperature = (internal_temperature_temp + internal_temperature) / 2;

  
  print.logqq("Internal temperature: ", internal_temperature);

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

void deepsleep(int time2sleep, vprint print) {
 
  esp_sleep_enable_timer_wakeup(time2sleep * uS_TO_S_FACTOR);
  print.logq("Going to sleep for " + String(time2sleep), " seconds");
  
  delay(1000);
  esp_deep_sleep_start();

}

void deepsleep(int time2sleep, vprint print, String message) {
 
  print.logq(message);
  esp_sleep_enable_timer_wakeup(time2sleep * uS_TO_S_FACTOR);
  print.logqq("Sleeping for " + String(time2sleep), " seconds");
  
  delay(1000);
  esp_deep_sleep_start();

}

void atmega_force_reset(vprint print){

  print.logqq("Forcing Atmega to reset.");

  pinMode(ATMEGA_FORCED_RESET_PIN, OUTPUT);

  digitalWrite(ATMEGA_FORCED_RESET_PIN, LOW);
  delay(700);
  digitalWrite(ATMEGA_FORCED_RESET_PIN, HIGH);
  delay(700);
  digitalWrite(ATMEGA_FORCED_RESET_PIN, LOW);
  delay(700); //TODO: search for a lower time

  print.logqqq("Reseting Atmega: Done.");

}

void atmega_soft_reset(vprint print){

  print.logq("Atmega soft reset.");
  digitalWrite(ARDUINO_RESTART, HIGH);
  delay(1);
  digitalWrite(ARDUINO_RESTART, LOW );
  delay(10);
  print.logqq("Done.");

}



int check_WiFi(vprint print, int millis_delay){
  int WiFi_HW_Status = 0;
  print.logq("Checking WiFi Modem Status");
  delay(millis_delay);
  WiFi_HW_Status = digitalRead(PIN_WIFI_STATUS);

  print.logqq("WiFi_HW_Status = ", WiFi_HW_Status );

  if(!HARDWARE_AVAILABLE) WiFi_HW_Status = 1;

  if(WiFi_HW_Status) print.logqq("WiFi is already ON.");
  else print.logqq("WiFi is OFF. " , WiFi_HW_Status);


  return WiFi_HW_Status;
}

void turn_modem_on(vprint print) {
  print.logq("Turning Modem ON.");
  digitalWrite(SIM_ON, HIGH);
  delay(1200);
  digitalWrite(SIM_ON, LOW);
  delay(2000);
  print.logqq("Done.");
}

double ReadVoltage(byte pin) {

  double reading = analogRead(pin);
  return reading;

}

bool str_to_bool(String bool_str, vprint print){


  if(bool_str == "true" || bool_str == "True" || bool_str == "TRUE") return true;
  else if (bool_str == "false" || bool_str == "False" || bool_str == "FALSE") return false;
  else {
    print.logqq("ERROR. str_to_bool receives unexpected string.");
    return false;
  }

}

void  get_ports(vprint print) {
  
  char message[500];
  memset(message,0,500);
  int16_t port_div[50];
  int8_t sample_div[50];
  int16_t eos; //end of string
  int16_t m = 0;
  int8_t n = 0; // counter ports
  int8_t try_counter = 0;

  String response;

  char recipe_aux;
  int stop=0;

  PORT_RQ = true;

  if (!PORT_RQ) print.logq("No Port sensing  request.");
  else  {

    print.logq("Sensors on port requested.");

    long unsigned time = millis();
    long unsigned data_rq_elapsed_time;
    
    print.logq("Sending data_rq");
    ArdSerial.print("Data_rq");

    while (millis() - time < 10000)
    { 
      data_rq_elapsed_time = millis();
      while(millis() - data_rq_elapsed_time < 1000) {
        if (ArdSerial.available()) {
          response = ArdSerial.readString();
          print.logq("response obtained: ", response);
          atmega_force_reset(print);
          break;

        }
      }

    }
    
        
    
 /*



    print.logqq("Atmega response: ");


    unsigned long time_sensor = millis();

    while (true) {

      if (ArdSerial.available()) { 
        recipe_aux= ArdSerial.read();
        if(recipe_aux=='$'){
              stop=stop+1;
              if(stop==2)break;     
        }
 
        message[m]=recipe_aux;
        m++;

      } else {
        // TODO: we can avoid this time using a bidirectional communication ACKs an NACKs
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



    
      // 01:11:12.511 -> %P1_-3235.2 20.2_-3105.5 20.2_-2969.8 20.2_k%P2_-3036.4 20.2_-3006.5 20.2_k%P3 %P4 $
    
      // %P1_-3235.2 20.2_-3105.5 20.2_-2969.8 20.2_k%P2_-3036.4 20.2_-3006.5 20.2_k%P3 %P4_123.123 12.213_452.34 3434_x%P41_123.123 123.32_x%P5 %P6 $

      // 01:11:12.511 -> -3235.2 20.2_-3105.5 20.2_-2969.8 20.2_k
      // 01:11:12.511 -> -3036.4 20.2_-3006.5 20.2_k

    


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

    
    //-------------------------------------------------------------------------------
    Serial.println("--------------------------- END PORT -----------------------------");
    moving_average_sensor();

  */  

  }
 
}

void force_hardware_reset(vprint print){
  //reset both atmega and esp32

  print.logq("Forcing hardware reset");

    unsigned long task_start_time;
    int i = 0;

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

    //
    print.logqq("resetting ESP32 using Atmega.");
    delay(1000); //TODO: Explain this time
}

bool send_peripheral_command(char command , vprint print) {

  
  String str_recipe = "NACK"; // default value because we are pessimists
  unsigned long startime = millis();
  bool serial_response = false;
  
  print.logq("Resetting, sending command and waiting for ACK.");
  atmega_soft_reset(print);
  

  for (int i = 0; i < 3 && millis() - startime < 1000 && !serial_response; i++)
  {
    ArdSerial.write(command);
    delay(150);
    serial_response = ArdSerial.available();
  }
  
  if (!serial_response) return false;
  else str_recipe = ArdSerial.readString();

  if(str_recipe == "ACK") {
    print.logqq("ACK from peripheral received.");
    return true;}
          
  else print.logqq("Error");
  
    
   
}

bool firmware_update_manager(int update_mode, String version, vprint print){



  if(update_mode == 0){
  
    print.logqqq("Current version: ", firmware_version);
    if(version ==  firmware_version){
      print.logqqq("Current firmware already target the global version ", version);
      return false;
    }
    else print.logqqq("Firmware will update to global version ", version);
   
  }

  else if(update_mode == 1){

    if(version ==  firmware_version){
      print.logqqq("Current firmware already target the local version ", version);
      return false;
    }
    else print.logqqq("Firmware will update to specific version ", version);

  }

  else {
    print.logqqq("[ERROR] Unexpected update_mode."); 
    return false;
    }

  
  print.logqqq("Downloading firmware_manager.json");

  HTTPClient http;
  http.begin(UPDATE_JSON_URL+ (String)"firmware_manager.json");
  int httpCode = http.GET();

  
  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
      print.logqqq("[HTTP] CODE: " + String( httpCode));

    if (httpCode == HTTP_CODE_OK) {
      print.logqqq("[HTTP] CODE OK");


      String payload = http.getString();
      
      char rcv_buffer[300];

      payload.toCharArray(rcv_buffer, payload.length() + 1);

      cJSON *json = cJSON_Parse(rcv_buffer);
      if (json == NULL)  print.logqqq("downloaded file is not a valid json, aborting...");
      else {
         print.logqqq("[HTTP] JSON OK");

        cJSON *hw_version_json = cJSON_GetObjectItemCaseSensitive(json, HARDWARE_VERSION_FIRMWARE.c_str());

        if (hw_version_json == NULL) {
          print.logqqq("There is no any FW for the HW version " + HARDWARE_VERSION_FIRMWARE + ". Aborting...");
          http.end();
          return 0;
          }

        cJSON *firmware_version_json = cJSON_GetObjectItemCaseSensitive(hw_version_json, version.c_str());

        if (firmware_version_json == NULL) {
          print.logqqq("There is no FW version " + version + ". Aborting...");
          http.end();
          return 0;
          }
       
        cJSON *file = cJSON_GetObjectItemCaseSensitive(firmware_version_json, "file");

        String file_name = file->valuestring;

        http.end();

        print.logqqq("File name: " + file_name);

        //Firebase.get(firebasedata, "/Certificates/S3_FW");
        //String certificate = firebasedata.stringData();
        print.logqqq("Getting Certificate.");
        http.begin(UPDATE_JSON_URL+ (String)"ca.cer");
        httpCode = http.GET();
        if (httpCode > 0) {
          // HTTP header has been send and Server response header has been handled
          print.logqqq("[HTTP] CODE: " + String( httpCode));

          if (httpCode == HTTP_CODE_OK) {
            print.logqqq("[HTTP] CODE OK");


            String payload = http.getString();
            
            char rcv_buffer[1650];

            payload.toCharArray(rcv_buffer, payload.length() + 1);

            String certificate = String(rcv_buffer);
            //print.logqqq("Getting Certificate:\n ", certificate);
            
            print.logqqq("Done.");
            
            update_firmware(file_name, print, certificate);


            }
        } 
        else {
          Serial.println("[HTTP] GET CA FAIL. ERROR: " +  http.errorToString(httpCode));
          return 0;
        }


      }

    }

  }
  else {
    Serial.println("[HTTP] GET... failed, error: " +  http.errorToString(httpCode));
    return 0;

  }

}


bool update_firmware(String file_name, vprint print, String certificate){
  print.logq("Updating:");
  WiFiClientSecure client;

  client.setCACert(certificate.c_str());
  t_httpUpdate_return ret = httpUpdate.update(client, UPDATE_JSON_URL + file_name);

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
