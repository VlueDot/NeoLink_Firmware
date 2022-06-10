
//                                                                      
// NEOLINK - AIDA v3.0.0                                                                                                       //
//
// Compatiblee con Peripheral v2.0.0
// Date 02 Jun 22 by V.R 
//______________________________________________________________________

const String version = "3.0.0";
const char* host = "esp32";
const char* ssid = "MOVISTAR_9F86";
const char* password = "9Qt6DFyaXZUG7SPkgZzn";

//------------- Web Debug ------------------------------------------------

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>//include webserver modified. 
#include <ESPmDNS.h>
#include <Update.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <string.h>

//------------- OTA HTTPS ------------------------------------------------

#include "cJSON.h"
#include <WiFiClientSecure.h>
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include <HTTPUpdate.h>



#define uS_TO_S_FACTOR 1000000



String string_Log="";
String tlog ="";

RTC_DATA_ATTR int16_t MODE_PRG = 1;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");



 
/*
 * Server Index Page
 */

char* log1 = 
  "<script>t=document.createElement('div');"
  "t.appendChild( document.createElement('br').appendChild(document.createTextNode('hola perro2')));"
  "document.body.appendChild(t)</script>";

const char* serverIndex =
"<head>"
  "<title>Greenbird Ag - AIDA Developer Dashboard</title>"
 " <link rel='shortcut icon' href='https://static.wixstatic.com/media/5ab796_bec8037a1f7043baaa84c8e883348b70%7Emv2.jpg/v1/fill/w_32%2Ch_32%2Clg_1%2Cusm_0.66_1.00_0.01/5ab796_bec8037a1f7043baaa84c8e883348b70%7Emv2.jpg' type='image/jpeg'>"
"</head>"
"<style type = 'text/css'>"
" pre{  color: #192f43;"
        "margin:0 ;"
"}"
"</style>"


"<body onload = initWebSocket(event)>"
//"<body>"
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
  "<button  type=\"button\" onclick = \"SoftReset()\">Soft Reset</button>"
  "<button  type=\"button\" onclick = \"SoftResetClear()\">Soft Reset and Clear</button>"
  "<button  type=\"button\" onclick = \"HardReset()\"> Hard Reset </button>"
  "<button  type=\"button\" onclick = \"HardResetClear()\">Hard Reset and Clear</button>"
  "<button  type=\"button\" onclick=\"Exit_PRG_Mode()\">Exit PRG_MODE</button>"
  "<button  type=\"button\" onclick= \"Clear_data()\">Clear Data</button>"
 

  
  "<div><br><b>Read data: </b><br></div>"
  "<div><p id='log'></p></div>"
  
"<script>"

    "var gateway = `ws://esp32.local/ws`;"
    "var websocket;"

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
    "}"

    
    "let count=0;"
    "function onMessage(event) {"
     "console.log(event.data);"
      "count++;"
      "t=document.createElement('pre');"
      "t.setAttribute('id',count);"
      "t.setAttribute('class', 'data');"
      "t.appendChild( document.createTextNode(event.data));"
      "document.body.appendChild(t)"
  "}"

"</script>"
    

"<script>"
  "function SoftReset() {"
    "websocket.send('SoftReset');"
    "websocket.close(); setTimeout(initWebSocket,3200);"  
    "console.log('Sent Soft Reset');"
    "}"

  "function SoftResetClear() {"
    "SoftReset();"
    "Clear_data();"
  
    "}"
    
  "function HardReset() {"
    "websocket.send('HardReset');"
    "websocket.close(); setTimeout(initWebSocket,3200);"  
    "console.log('Sent Hard Reset');"
    "}"

  "function HardResetClear() {"
    "HardReset();"
    "Clear_data();"
    "}"

    "function Exit_PRG_Mode() {"
    "console.log('Exit PRG_MODE');"
    "}"

    "function Clear_data() {"
    "$('.data').remove();"
    "}" 
"</script>"

"<script>"
  "$('form').submit(function(e){"
  "websocket.close(); setTimeout(initWebSocket,3500);"
  "Clear_data();"
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

//FUNCTIONS HEADERS

void SendMessageWeb(String message) {
  ws.textAll(message);
}

class vprint{

public: 


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










 
    

void setup(void) {

  Serial.begin(115200);
  vprint print;

  print.logq("Modo PRG: ", MODE_PRG);




  // Connect to WiFi network
  WiFi.begin(ssid, password);
  
  
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


  print.logq("Connected to " + String(ssid));
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






