
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

#define NO_PING_TIME 60




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

//Default Settings
RTC_DATA_ATTR float BAT_L = 3.2;
RTC_DATA_ATTR float BAT_H = 3.4;

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
void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
void start_server(vprint print);
void starting_wifi(vprint print);
void check_configuration(vprint print);
void Update_Local_Info_Chip(vprint print, int update_local_info_flag);
void write_eeprom(String data, int position);
String read_eeprom(int init, int len);
void get_SN_n_MAC(vprint print);
bool ping(vprint print);
void print_controller_status(vprint print);
void create_SN_nodes(vprint print);

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
    if(check_WiFi(print,1000)) starting_wifi(print);
    else { turn_modem_on(print); handleCommands("SoftReset"); }


    if(LOCAL_SERVER){  ws.onEvent(onEvent); server.addHandler(&ws); start_server(print);}

    if(LOCAL_SERVER) delay(6000);

    //checking first if there is internet
    if(!ping(print)) handleCommands("SoftReset",NO_PING_TIME);
    
    check_configuration(print);
    
   
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

void get_SN_n_MAC(vprint print){

  print.logq("Reading SN and MAC:");

  DEVICE_HEADER = read_eeprom( 0 , 2 );
  HARDWARE_VERSION = read_eeprom( 2 , 2);
  ENVIRONMENT = read_eeprom( 4 , 1 );
  DATE_CORRELATIVE = read_eeprom( 5 , 4);
  SN_CORRELATIVE = read_eeprom( 9 , 4);

  SN = DEVICE_HEADER+HARDWARE_VERSION+ENVIRONMENT+DATE_CORRELATIVE+SN_CORRELATIVE;
  
  if((DEVICE_HEADER + HARDWARE_VERSION)==(DEVICE_HEADER_FIRMWARE+HARDWARE_VERSION_FIRMWARE)) print.logqq("Firmware compatible with this device version. SN: ", SN);
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
    print.logqq("Fail. Resetting NO_PING. ");
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

void check_configuration(vprint print){

    print.logq("Checking for configurations.");

    //getting update_sn_flag and new_config_flags
    //get sn by chipid and compare. if is the same update_sn_flag = 1 and 
    //well ... the sn will update and check if nodes in firebase related exist. 
    
    FirebaseJsonData SN_Chip_firebase_data;
    FirebaseJson SN_Chip_firebase;
    String SN_Chip_str;

    if(Firebase.get(firebasedata, "/SN_Chips/" + chipid_str + "/")) {
      Serial.println(firebasedata.dataType());

      firebasedata.jsonObject().toString(SN_Chip_str,true);
      Serial.println(SN_Chip_str);
    
/*
      String buff_string_SN_Chip;
      SN_Chip_firebase.toString(buff_string_SN_Chip,true);
      print.logqq("Looking for SN_Chip_Firebase: ");
      print.logqq(buff_string_SN_Chip); */
      
      
      }
    else {
      print.logqq("SN not found.");
      create_SN_nodes(print);
    
    }
    
 


    //Update_Local_Info_Chip(print, update_local_info_flag);

    /*Firebase.getInt(firebasedata, "Services_controllers_Flags/Devices/" + SN + "/local_update_firmware"); 
    local_update_firmware = firebasedata.intData();
    Firebase.getInt(firebasedata, "Services_controllers_Flags/Devices/" + SN + "/update_config_flag"); 
    update_config_flag = firebasedata.intData();
    Firebase.getInt(firebasedata, "Services_controllers_Flags/Devices/" + SN + "/update_local_info_flag"); 
    update_local_info_flag = firebasedata.intData();

    print.logq(local_update_firmware);
    print.logq(update_config_flag);
    print.logq(update_local_info_flag);*/

}
  


void Update_Local_Info_Chip(vprint print, int update_local_info_flag){

//Verifica que se encuentre el tipo de equipo en la eeprom no sea XX o vacio. Si lo es descarga la Local Info y guarda en la eeprom. 
//verifica la que la bandera update_local_info_chip se encuentre en 1 para descargar nuevamente la data

print.logq("Checking for new configurations in firebase. " );




}

void write_eeprom(String string_to_eeprom, int position){

 
  for (size_t i = 0; i <  string_to_eeprom.length() ; i++)
  {
    EEPROM.write(i+position,string_to_eeprom[i]);
    Serial.print("Saving ");
     Serial.print(string_to_eeprom[i]);
     Serial.print("in ");
     Serial.print( i + position);
  }

  EEPROM.commit(); 
  
  
}

String read_eeprom(int init, int len){

  char eeprom_to_read[20];

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

        print.logqq("Connected to " + String(WIFI_SSID));
        print.logqq("IP address: "+ WiFi.localIP().toString());

        Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
        Firebase.reconnectWiFi(true);
     
        break;

      }

      else {

        init_timestamp = millis();
        WiFi.begin(WIFI_SSID, WIFI_PSSWD);

        print.logqq("ERROR: "+ String(WiFi.status()) +  "Attempt: " +  String(wifi_try) );

        wifi_try++;

        if (wifi_try > 3) {

          print.logqq("WiFi refuses connection. Sleeping..");
          Stage = 0;
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

