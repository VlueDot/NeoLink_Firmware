
// Peripheral v1.0

#include <Sensor.h>
#include "LowPower.h"
#include <avr/sleep.h>
#include <SoftwareSerial.h>
#include "Arduino.h"


#define RX0_ARD_TX_SENSOR 0
const byte esp32_RX___ard_TX = 2;
#define INTERRUPT_PIN_RESET 3
#define LED 4
const byte TX_ARD___RX_GPS = 5;
const byte RX_ARD___TX_GPS = 6;
#define SENSOR1_EN 7
#define SENSOR2_EN 8
#define SENSOR3_EN 9
#define SENSOR4_EN 10
#define SERIAL_A 11
#define SERIAL_B 12
#define SERIAL_C 13

#define GPS_EN 17 //A3
#define EN_RESET_ESP 18 //A4
const byte esp32_TX___ard_RX = 19;



#define MAX_TIME_SENSOR 2450
#define MAX_TIME_GPS


//--------------------------------------------------------
#include <SoftwareSerial.h>

SoftwareSerial ESP32(esp32_TX___ard_RX, esp32_RX___ard_TX);

SoftwareSerial gps(RX_ARD___TX_GPS, TX_ARD___RX_GPS);

//---------------------------------------------------------
//memoria interna
//states

int8_t REGISTER_DEVICE = 0;
int8_t LOW_BATTERY = 0;

//tasks

//-------------------------------------

String recipe;
char   type;
int    len;

String port1;
String port2;
String port3;
String port4;


int8_t request = 0;

unsigned long init_time;

#define time_between 500

Sensor sensor(SENSOR1_EN, SENSOR2_EN, SENSOR3_EN, SENSOR4_EN, SERIAL_A, SERIAL_B, SERIAL_C, MAX_TIME_SENSOR);


int one_read = 1;

void setup() {
  Serial.begin(1200);
  digitalWrite(GPS_EN, LOW );
  pinMode(INTERRUPT_PIN_RESET, INPUT);
  pinMode(GPS_EN, OUTPUT);
  pinMode(18,OUTPUT);
  
  ESP32.begin(57600);
  init_time = millis();

}

int port_number;
String message;
String com_response = "nan";

void loop() {

 if (ESP32.available()) {
    request = ESP32.read();


    if (request == 's') {
      
      delay(100);

      Serial.println("Sending ACK to atmega");
      ESP32.print("ACK");
      Serial.println("Starting reading sensors");
      
      message = reading_sensors();

      Serial.println("message is: " + message );

      Serial.println("Waiting request for sending message to ESP32.");

      while(!ESP32.available()) delay (150);

      String data_request = ESP32.readString();

      if (data_request != "Data_rq") Serial.println("Error. Received: " + data_request);
      else {
        delay(150);
        ESP32.print(message);
        
      }
      




      delay(400);
      Serial.println("data_request: " + data_request);
      Serial.println("Sensor request end. Pass to sleep");
      //delay(500);
      //Going_To_Sleep();


    }
      
        

    else if (request == 'e'){
        delay(100);
        ESP32.write('@');

        Serial.println("Resetting ESP32.");
        delay(400);
        digitalWrite(EN_RESET_ESP, HIGH);
        delay(500);
        digitalWrite(EN_RESET_ESP, LOW);
        delay(500);
        Serial.println("Powering down.");
        delay(500);
        Going_To_Sleep();
         



    }


  }

  if (millis() - init_time >= 500) {
    
    Serial.println("Powering off due inactivity");
    //ESP32.println("wa");
    delay(500);
    Going_To_Sleep();
    //while (1) Serial.println("pf");
  }


}

void (*resetFunc)(void) = 0;

void Going_To_Sleep() {
  sleep_enable();//Enabling sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);//Setting the sleep mode, in our case full sleep
  //Serial.println("e");
  //digitalWrite(LED_BUILTIN, LOW); //turning LED off

  attachInterrupt(1, wakeUp, HIGH);//attaching a interrupt to pin d2
  delay(100); //wait a second to allow the led to be turned off before going to sleep

  sleep_cpu();//activating sleep mode3..
  //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  resetFunc();


  // digitalWrite(LED_BUILTIN, HIGH); //turning LED on
}



void wakeUp() {

  //Serial.begin(1200);
  //Serial.println("just woke up!");//next line of code executed after the interrupt
  //delay(1000);
  detachInterrupt(1); //Removes the interrupt from 0:pin 2 1: pin3;
  //digitalWrite(INTERRUPT_PIN, LOW);

  //delay(1);

}




String  lat_long(String gpsrecipe){

  if (getValue(gpsrecipe,',',2).equals("A")){

               
              String hora= getValue(gpsrecipe,',',1);
              String pre_lat= getValue(gpsrecipe,',',3);
              String pre_lon= getValue(gpsrecipe,',',5);

              //Serial.println("----------------");
              //Serial.println(hora);
              //Serial.println(pre_lat);
              //Serial.println(pre_lon);
              
              
              String pre_lat_minutes= pre_lat.substring(2);
              String pre_lat_grades;
              if (pre_lat_minutes.toDouble()<100) {
                pre_lat_grades= pre_lat.substring(0,2);
                
                }
              else
              {
                pre_lat_minutes= pre_lat.substring(3);
                pre_lat_grades= pre_lat.substring(0,3);
              }
    
              
              String pre_lon_minutes= pre_lon.substring(2);
              String pre_lon_grades;
              if (pre_lon_minutes.toDouble()<100) {
                pre_lon_grades= pre_lon.substring(0,2);
                
                }
              else
              {
                pre_lon_minutes= pre_lon.substring(3);
                pre_lon_grades= pre_lon.substring(0,3);
              }


              double latitud= pre_lat_grades.toDouble() + pre_lat_minutes.toDouble()/60.00;
              if (getValue(gpsrecipe,',',4).equals("S")) latitud=latitud*-1;
              double longitud= pre_lon_grades.toDouble() + pre_lon_minutes.toDouble()/60.00;
              if (getValue(gpsrecipe,',',6).equals("W")) longitud=longitud*-1;

              //Serial.println(String(latitud,6));
              //Serial.println(String(longitud,6));
              
              
           
          

            
            return "GP$"+String(latitud,6) + "$" + String(longitud,6)+"$0";
            
            

          }

          else
          {
            return "NaN";
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




String reading_sensors(){

  for (size_t i = 0; i < 10; i++)
  {
    Serial.print(".");
    delay(1000);
    
  }
  Serial.println();
  return "data obtenida del sensado";
  

}


String start_sensing(){

  delay(10);
      //digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Sensing..");
      //----------------- PUERTO 1 ------------------------------------------
      recipe = "";
      port_number = 1;
      recipe = sensor.read_sensor(port_number) + "x";
      //Serial.println(recipe);
      type = sensor.sensor_type;
      len = sensor.size_array;

      if (recipe != "NaNx") {
        
       
        
        String temporal1 = getValue(getValue(recipe,13, 0), 9 , 1);
        //port1 = "%P1_" + getValue(temporal1,' ',0) + " " + getValue(temporal1,' ',1) + " " + getValue(temporal1,' ',2) ;
        port1 = "%P1_" + temporal1 ;
        //Serial.print(port1);
        

        delay(time_between);


        recipe = "";
        recipe = sensor.read_sensor(port_number) + "x" ;


        if (recipe != "NaNx") {

          temporal1 = getValue(getValue(recipe,13, 0), 9 , 1);
          //port1 = port1 + "_" + getValue(temporal1,' ',0) + " " + getValue(temporal1,' ',1) + " " + getValue(temporal1,' ',2) ;
          port1 = port1 + "_" + temporal1;


        }

        port1 = port1 + "_" + type;

      } else  {
        port1 = "%P1_";

      }

      if (recipe != "NaNx") Serial.println(port1);




      //----------------- PUERTO 2 ------------------------------------------

      recipe = "";
      port_number = 2;
      recipe = sensor.read_sensor(port_number)+"x";

      type = sensor.sensor_type;
      len = sensor.size_array;

      if (recipe != "NaNx") {

        String temporal2 = getValue(getValue(recipe,13, 0), 9 , 1);
        //port2 = "%P2_" + getValue(temporal2,' ',0) + " " + getValue(temporal2,' ',1) + " " + getValue(temporal2,' ',2) ;
        port2 = "%P2_" + temporal2 ;




        delay(time_between);


        recipe = "";
        recipe = sensor.read_sensor(port_number) + "x";


        if (recipe != "NaNx") {


          temporal2 = getValue(getValue(recipe,13, 0), 9 , 1);
          //port2 = port2 + "_"  + getValue(temporal2,' ',0) + " " + getValue(temporal2,' ',1) + " " + getValue(temporal2,' ',2) ;
          port2 = port2 + "_" + temporal2;


        }

        port2 = port2 + "_" + type;

      } else  {
        port2 = "%P2_";

      }

      if (recipe != "NaNx") Serial.println(port2);



      //----------------- PUERTO 3 ------------------------------------------



      recipe = "";
      port_number = 3;
      recipe = sensor.read_sensor(port_number)+"x";

      type = sensor.sensor_type;
      len = sensor.size_array;

      if (recipe != "NaNx") {

        String temporal3 = getValue(getValue(recipe,13, 0), 9 , 1);
        //port3 = "%P3_" + getValue(temporal3,' ',0) + " " + getValue(temporal3,' ',1) + " " + getValue(temporal3,' ',2) ;
        port3 = "%P3_" + temporal3;
        delay(time_between);


        recipe = "";
        recipe = sensor.read_sensor(port_number) + "x";


        if (recipe != "NaNx") {


          temporal3 = getValue(getValue(recipe,13, 0), 9 , 1);
          //port3 = port3 + "_"  + getValue(temporal3,' ',0) + " " + getValue(temporal3,' ',1) + " " + getValue(temporal3,' ',2) ;
          port3 = port3 + "_" + temporal3;

        }

        port3 = port3 + "_" + type;

      } else  {
        port3 = "%P3_";

      }

      if (recipe != "NaNx") Serial.println(port3);






      //----------------- PUERTO 4 ------------------------------------------



      recipe = "";
      port_number = 4;
      recipe = sensor.read_sensor(port_number)+"x";

      type = sensor.sensor_type;
      len = sensor.size_array;

      if (recipe != "NaNx") {

        String temporal4 = getValue(getValue(recipe,13, 0), 9 , 1);
        //port4 = "%P4_" + getValue(temporal4,' ',0) + " " + getValue(temporal4,' ',1) + " " + getValue(temporal4,' ',2) ;
        port4 = "%P4_" + temporal4 ;
        
        delay(time_between);


        recipe = "";
        recipe = sensor.read_sensor(port_number) + "x";


        if (recipe != "NaNx") {


          temporal4 = getValue(getValue(recipe,13, 0), 9 , 1);
          //port4 = port4 + "_"  + getValue(temporal4,' ',0) + " " + getValue(temporal4,' ',1) + " " + getValue(temporal4,' ',2) ;
          port4 = port4 + "_" + temporal4;

        }

        port4 = port4 + "_" + type;

      } else  {
        port4 = "%P4_";

      }

      if (recipe != "NaNx") Serial.println(port4);

 

      //---------------------------------------------------------------------------------------
      String message = port1 + port2 + port3 + port4 + "$";
      //Serial.println(message);

    return message;

}