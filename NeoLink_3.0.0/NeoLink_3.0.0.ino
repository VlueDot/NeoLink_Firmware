
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
//--------- EEPROM -----------------------    
#include <EEPROM.h>
//-----------------------------Firebase & Servers-------------------------------------
#include "FirebaseESP32.h"
#include "FirebaseJson.h"
#include "neoFirebaseJson.h"
#include <Arduino_JSON.h>
#include <HTTPClient.h>



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
String HARDWARE_VERSION_FIRMWARE = "03";


#if FIRMWARE_MODE == 'PRO'
  #define FIREBASE_HOST "https://neolink-934b4.firebaseio.com"
  #define FIREBASE_AUTH "IroB3fdbcPb9vxPlJKDJcqmfJgs0KouJGe0sUBKN"
  #define UPDATE_JSON_URL  "https://firmware-neolink.s3-sa-east-1.amazonaws.com/firmware_pro.json"
  const char* WIFI_SSID = "LINUX5"; //modem default
  const char* WIFI_PSSWD = "1a23456789abc";
  
 

#elif FIRMWARE_MODE == 'DEV'
  #define FIREBASE_HOST "https://aidadev-71837-default-rtdb.firebaseio.com/"
  #define FIREBASE_AUTH "RbsWJ3F5EsLGLvpRefgTeyGQhEFHFp5pJfECurTE"
  #define UPDATE_JSON_URL  "https://firmware-neolink.s3-sa-east-1.amazonaws.com/firmware_pro.json"
  const char* WIFI_SSID = "MOVISTAR_9F86";
  const char* WIFI_PSSWD = "9Qt6DFyaXZUG7SPkgZzn";
  

#endif

//Time Variables

#define POWERLESS_TIME 600   //seconds to sleep when battery is low.
#define WIFI_TIME_LIMIT 6000 //seconds to connect wifi

#define NO_PING_TIME 180      // when doesnt exist first internet connection
#define NO_INTERNET_DESPITE_PING 60 //  when doesnt exist internet connection before the first one

#define DEFAULT_SN_SLEEPTIME 60






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
RTC_DATA_ATTR int PORT_1_DEPTH_A = 0;
RTC_DATA_ATTR int PORT_1_DEPTH_B = 0;

RTC_DATA_ATTR bool PORT_2_ENABLE = false;
RTC_DATA_ATTR int PORT_2_DEPTH_A = 0;
RTC_DATA_ATTR int PORT_2_DEPTH_B = 0;

RTC_DATA_ATTR bool PORT_3_ENABLE = false;
RTC_DATA_ATTR int PORT_3_DEPTH_A = 0;
RTC_DATA_ATTR int PORT_3_DEPTH_B = 0;

RTC_DATA_ATTR bool PORT_4_ENABLE = false;
RTC_DATA_ATTR int PORT_4_DEPTH_A = 0;
RTC_DATA_ATTR int PORT_4_DEPTH_B = 0;



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
RTC_DATA_ATTR int8_t battery_available = 1; 
RTC_DATA_ATTR int update_sn_flag = 0 ;






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
void atmega_force_reset(vprint print);
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
void get_SN_n_MAC(vprint print);
bool ping(vprint print);
void print_controller_status(vprint print);
void create_SN_nodes(vprint print);
void create_nodes(vprint print) ;
String* get_firebase_json_str(vprint print);
void deepsleep(int time2sleep, vprint print, String message);
void update_SN(vprint print);
bool str_to_bool(String bool_str, vprint print);
void get_firebase_configuration(vprint print);

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
  EEPROM.begin(150);
  
  //Stage =1;
  if (Stage == 1) {
    Serial.println();
    Serial.println("___________________________________________________");
    print.logq("Greenbird AG - AIDA | NeoLink Firmware Version: ", firmware_version);

    if (Start_or_Restart) beep.vbeep(250);

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
    Serial.println("___________________________________________________");
    print.logq("[STAGE 2]: ");

    //getting SN an MAC, again..
    get_SN_n_MAC(print); 

    //physical forced reset Atmega328pu. Why?
    atmega_force_reset(print);

    // Connect to WiFi network
    if(check_WiFi(print,500)) starting_wifi(print);
    else { Stage = 1 ; turn_modem_on(print); handleCommands("SoftReset"); }

    // Enabling local server if is required
    if(LOCAL_SERVER){  ws.onEvent(onEvent); server.addHandler(&ws); start_server(print);}

    //wait time to refresh your web
    //if(LOCAL_SERVER) delay(5000);

    //checking first if there is internet
    if(!ping(print)) deepsleep(1,print,"Deepsleep because no ping."); //handleCommands("SoftReset",NO_PING_TIME);

    update_SN(print);

    read_controllers_flags(print);

    read_configuration(print);
    
   
    for (int i = 0; i < 20; i++)
    {
      print.logq(i);
      delay(1000);
    }
    
    Stage = 1;
}

}

void loop(void) {
  if(LOCAL_SERVER) ws.cleanupClients();
  if(MODE_PRG) handleCommands("SoftReset");

}


//______________________________________________________________________
//
// Functions
//______________________________________________________________________


String* get_firebase_json_str(vprint print){
  /*get json from firebase as ordered string*/
  
  String buffer;
  
  FirebaseJson  &firebase_json = firebasedata.jsonObject();
  
  firebase_json.toString(buffer,true);
  print.logqqq("json: \n",buffer);
   
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

void get_SN_n_MAC(vprint print){

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
  Serial.println(SN_CORRELATIVE);*/

  SN = DEVICE_HEADER+HARDWARE_VERSION+ENVIRONMENT+DATE_CORRELATIVE+SN_CORRELATIVE;

  //Serial.println(SN);
  
  if((DEVICE_HEADER + HARDWARE_VERSION)==(DEVICE_HEADER_FIRMWARE+HARDWARE_VERSION_FIRMWARE)) {

    print.logqq("Firmware compatible with this device version.");
    print.logqq("SN: ", SN);
    
    }
  else print.logqq("SN not for this firmware. Updating SN in a while. SN: ", SN );

  uint64_t chipid;
  chipid = ESP.getEfuseMac(); //The chip ID is its MAC address(length: 6 bytes).
  chipid_str = String((uint16_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);

  print.logqq("Chip ID: ", chipid_str);


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
  print.logq("Creating SN nodes for " + chipid_str);

  neoFirebaseJson json_SN_nodes;
  json_SN_nodes.FirebaseJson::set("DEVICE_HEADER", chipid_str); 
  json_SN_nodes.FirebaseJson::set("HARDWARE_VERSION","00");
  json_SN_nodes.FirebaseJson::set("ENVIRONMENT", "D");
  json_SN_nodes.FirebaseJson::set("DATE_CORR_AAMM", "2200");
  json_SN_nodes.FirebaseJson::set("SN_CORR_AAMM", "9999");

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

  json_node_controllers_flag.FirebaseJson::set("local_last_firmware_update", false);
  json_node_controllers_flag.FirebaseJson::set("local_specific_firmware_update", false); 
  json_node_controllers_flag.FirebaseJson::set("local_specific_firmware_version", firmware_version);

  json_node_controllers_flag.FirebaseJson::set("update_config_flag", false); 

  
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
  
  json_node_services_config.FirebaseJson::set("PORTS/PORT_1/ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_1/DEPTH_1A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_1/DEPTH_1B", 0); 

  json_node_services_config.FirebaseJson::set("PORTS/PORT_2/ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_2/DEPTH_2A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_2/DEPTH_2B", 0); 

  json_node_services_config.FirebaseJson::set("PORTS/PORT_3/ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_3/DEPTH_3A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_3/DEPTH_3B", 0); 

  json_node_services_config.FirebaseJson::set("PORTS/PORT_4/ENABLE", false); 
  json_node_services_config.FirebaseJson::set("PORTS/PORT_4/DEPTH_4A", 0);
  json_node_services_config.FirebaseJson::set("PORTS/PORT_4/DEPTH_4B", 0); 

  
  Firebase.updateNode(firebasedata, "/Services_config/NeoLink/" + SN + "/", json_node_services_config );

  print.logqq("Creating nodes in Services_controllers.");

  for (int i = 1; !Firebase.updateNode(firebasedata, "/Services_controllers_Flags/NeoLink/" + SN + "/", json_node_controllers_flag ); i++)  i>3 ? deepsleep(NO_INTERNET_DESPITE_PING, print, "Attemps Failed.") : print.logq(" Updating failed. Attemp: ", i);
  
  print.logqq("Done.");
  print.logqq("Creating nodes in Services_config.");

  for (int i = 1; !Firebase.updateNode(firebasedata,"/Services_config/NeoLink/" + SN + "/", json_node_services_config ); i++)  i>3 ? deepsleep(NO_INTERNET_DESPITE_PING, print, "Attemps Failed.") : print.logq(" Updating failed. Attemp: ", i);
  print.logqq("Done.");  


}

void update_SN(vprint print){

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
    String fb_SN_CORR_AAMM= get_value_json_str(response, "SN_CORR_AAMM");

    if(fb_DEVICE_HEADER == "00") print.logqq("Not update because the SN in Firebase has the default value chipid");
   
    else{

    String fb_SN = fb_DEVICE_HEADER + fb_HARDWARE_VERSION + fb_ENVIRONMENT + fb_DATE_CORR_AAMM + fb_SN_CORR_AAMM;

    
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

  bool global_last_firmware_update  = false;
  bool global_specific_firmware_update = false;
  String global_specific_firmware_version;
  const String Global_update_path = "/Services_controllers_Flags/Global/NeoLink";
  bool global_json_obtained = Firebase.get(firebasedata, Global_update_path);
   

  
  print.logqq("Reading global flags.");

  if( global_json_obtained ){
    
    String* response = get_firebase_json_str(print);

    global_last_firmware_update = str_to_bool(get_value_json_str(response, "global_last_firmware_update"),print);
    global_specific_firmware_update = str_to_bool(get_value_json_str(response, "global_specific_firmware_update"), print);
    global_specific_firmware_version = get_value_json_str(response, "global_specific_firmware_version");

    print.logqqq("Done.");
   
  } else print.logqq("Unsuccessful.");

  bool local_last_firmware_update  = false;
  bool local_specific_firmware_update = false;
  String local_specific_firmware_version;
  String Local_update_path = "/Services_controllers_Flags/NeoLink/" + SN;
  bool local_json_obtained = Firebase.get(firebasedata, Local_update_path);
  bool update_config_flag = false;
  

  print.logqq("Reading local controller flags.");

  if( local_json_obtained ){
    
    String* response = get_firebase_json_str(print);
    
    local_last_firmware_update = str_to_bool( get_value_json_str(response, "local_last_firmware_update"), print);
    local_specific_firmware_update = str_to_bool( get_value_json_str(response, "local_specific_firmware_update"), print);
    local_specific_firmware_version = get_value_json_str(response, "local_specific_firmware_version");
    update_config_flag = str_to_bool(get_value_json_str(response, "update_config_flag"), print);

    print.logqqq("Done.");
   
  } else print.logqqq("Unsuccessful.");

  
    if(global_specific_firmware_update && !global_specific_firmware_update && !local_last_firmware_update && !local_specific_firmware_update) {
      print.logqq("Firmware will update to the last version via global update.");
      

    }

    else if(global_specific_firmware_update && !local_last_firmware_update && !local_specific_firmware_update){
      print.logqq("Firmware will update via global specific update to the version " + global_specific_firmware_version + ".");
      

    }

    else if(local_last_firmware_update && !local_specific_firmware_update){
      print.logqq("Firmware will update to the last version via local update");

    }

    else if (local_specific_firmware_update){
      print.logqq("Firmware will update via specific update to the version " + local_specific_firmware_version);

    }

    else print.logqq("No update will perform.");

    
    if(update_config_flag){
      print.logqq("Reading and Saving in eeprom new configuration from Services_config.");
      get_firebase_configuration(print);
      
    } else print.logqq("No configuration updating will performed.");


}

void read_configuration(vprint print){

    print.logq("Reading configurations in eeprom.");








  

}

void get_firebase_configuration(vprint print){

const String configurations_path = "/Services_config/NeoLink";


if( Firebase.get(firebasedata, configurations_path)) {
  print.logq("New configuration received.");
  String* response = get_firebase_json_str(print);

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
  PORT_1_DEPTH_A = get_value_json_str(response, "PORT_1_DEPTH_A").toInt();
  PORT_1_DEPTH_B = get_value_json_str(response, "PORT_1_DEPTH_B").toInt();

  PORT_2_ENABLE = str_to_bool( get_value_json_str(response, "PORT_2_ENABLE"), print);
  PORT_2_DEPTH_A = get_value_json_str(response, "PORT_2_DEPTH_A").toInt();
  PORT_2_DEPTH_B = get_value_json_str(response, "PORT_2_DEPTH_B").toInt();


  PORT_3_ENABLE = str_to_bool( get_value_json_str(response, "PORT_3_ENABLE"), print);
  PORT_3_DEPTH_A = get_value_json_str(response, "PORT_3_DEPTH_A").toInt();
  PORT_3_DEPTH_B = get_value_json_str(response, "PORT_3_DEPTH_B").toInt();


  PORT_4_ENABLE = str_to_bool( get_value_json_str(response, "PORT_4_ENABLE"), print);
  PORT_4_DEPTH_A = get_value_json_str(response, "PORT_4_DEPTH_A").toInt();
  PORT_4_DEPTH_B = get_value_json_str(response, "PORT_4_DEPTH_B").toInt();



  Serial.println(String(LONGITUDE,8));

  print.logqq("Longitud: ", LONGITUDE);


  
    





} else print.logqq("No configuration received.");

print.logqq("Turning update_config_flag off.");
for (int i = 1; !Firebase.setBool(firebasedata, "/Services_controllers_Flags/NeoLink/" + SN + "/update_config_flag" , false) ; i++) i>3 ? deepsleep(NO_INTERNET_DESPITE_PING, print, "Attemps Failed.") : print.logq(" Updating failed. Attemp: ", i);
  




}

void write_eeprom(String string_to_eeprom, int position, vprint print){

 
  for (size_t i = 0; i <  string_to_eeprom.length() ; i++) EEPROM.write(i+position,string_to_eeprom[i]);
  EEPROM.commit(); 

  print.logqq("\tDone. Bytes saved: ", int (string_to_eeprom.length()));
  
  
}

String read_eeprom(int init, int len){

  char eeprom_to_read[80];

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
        if(LOCAL_SERVER) delay(8000);
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

  print.logq("Forcing Atmega to reset.");

  digitalWrite(ATMEGA_FORCED_RESET_PIN, HIGH);
  delay(500);
  digitalWrite(ATMEGA_FORCED_RESET_PIN, LOW);
  delay(2000);

  print.logqq("Reseting Atmega: Done.");

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