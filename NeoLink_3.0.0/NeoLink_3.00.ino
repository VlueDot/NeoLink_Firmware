
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
#include <string.h>
#include "Constants.h"
//------------- OTA HTTPS ------------------------------------------------
#include "cJSON.h"
#include <WiFiClientSecure.h>
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include <HTTPUpdate.h>
//------------- Serials----------------------------------------------------------
#include <SoftwareSerial.h>
//-----------------------------------OWN---------------------------------
#include <beep.h>
//--------- INTERNAL TEMP ----------------
#include <OneWire.h>
#include <DallasTemperature.h>


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
// Constantes Globales
//______________________________________________________________________


const String version = "3.0.0";
const char* host = "esp32";

//#define BAND    433E6

#define uS_TO_S_FACTOR 1000000
#define FIRMWARE_MODE 'DEV'


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




//______________________________________________________________________
//
//Variable globales
//______________________________________________________________________

RTC_DATA_ATTR int16_t MODE_PRG = 1;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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

};







//______________________________________________________________________
//
// Codigo principal
//______________________________________________________________________



 
    

void setup(void) {

  Serial.begin(115200);
  vprint print;

  print.logq("Modo PRG: ", MODE_PRG);




  // Connect to WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PSSWD);
  
  
  Serial.println("");

  // Wait for connection
  int16_t start_time = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    
    if(millis()-start_time > 10000) deepsleep(1);
    
      }


  ws.onEvent(onEvent);
  server.addHandler(&ws);
  start_server();


  print.logq("Connected to " + String(WIFI_SSID));
  print.logq("IP address: "+ String(WiFi.localIP()));
  

  for (int16_t i = 0; i < 100; i++)
  {
    
    print.logq(i);
    delay(1000);
  }
  
 
 
}



void loop(void) {
  
  ws.cleanupClients();

}

void handleCommands(String data){

if(data == "SoftReset") ESP.restart();
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



void start_server(){

 /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  delay(1000);

  Serial.println("Server started: http://esp32.local/"); //http://esp32.local/
  /*return index page which is stored in serverIndex */


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


void deepsleep(int time2sleep) {
  esp_sleep_enable_timer_wakeup(time2sleep * uS_TO_S_FACTOR);
  Serial.println("Going to sleep for " + String(time2sleep) + " Seconds");
  Serial.println();
  //LoRa.end();
  //LoRa.sleep();
  delay(100);
  esp_deep_sleep_start();

}






