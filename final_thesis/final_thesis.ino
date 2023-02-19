#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "Adafruit_Si7021.h"

#include <WiFi.h>
#include <HTTPClient.h>


const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

//Your Domain name with URL path or IP address with path
String serverName = "http://192.168.1.106:1880/update-sensor";



uint32_t delayMS;
//--dht var till here

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */


//   delay(1000);
// }



bool enableHeater = false;
uint8_t loopCnt = 0;
Adafruit_Si7021 sensor = Adafruit_Si7021();

long interval = 1800;  
long previousMillis = 0;
//*****************Arduino anemometer sketch******************************
const byte interruptPin = 18; //anemomter input to digital pin
volatile unsigned long sTime = 0; //stores start time for wind speed calculation
unsigned long dataTimer = 0; //used to track how often to communicate data
volatile float pulseTime = 0; //stores time between one anemomter relay closing and the next
volatile float culPulseTime = 0; //stores cumulative pulsetimes for averaging
volatile bool start = true; //tracks when a new anemometer measurement starts
volatile unsigned int avgWindCount = 0; //stores anemometer relay counts for doing average wind speed
float aSetting = 60.0; //wind speed setting to signal alarm
void anemometerISR();
//This is the interrupt service routine (ISR) for the anemometer input pin
//it is called whenever a falling edge is detected
void anemometerISR() {
  unsigned long cTime = millis(); //get current time
  if(!start) { //This is not the first pulse and we are not at 0 MPH so calculate time between pulses
   // test = cTime - sTime;
    pulseTime = (float)(cTime - sTime)/1000;
    culPulseTime += pulseTime; //add up pulse time measurements for averaging
    avgWindCount++; //anemomter went around so record for calculating average wind speed
  }
  sTime = cTime; //store current time for next pulse time calculation
  start = false; //we have our starting point for a wind speed measurement
}
//using time between anemometer pulses calculate frequency of anemometer
float getAnemometerFreq(float pTime) { return (1/pTime); }
//Use anemometer frequency to calculate wind speed in MPH, note 2.5 comes from anemometer data sheet
float getWindMPH(float freq) { return (freq*2.5); }
//uses wind MPH value to calculate KPH
float getWindKPH(float wMPH) { return (wMPH*1.61); }
//Calculates average wind speed over given time period
float getAvgWindSpeed(float cPulse,int per) {
  if(per) return getWindMPH(getAnemometerFreq((float)(cPulse/per)));
  else return 0; //average wind speed is zero and we can't divide by zero
  }
void adc() {
  Serial.begin(9600);
  Serial.println("Hello!");

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

     int16_t adc0, adc1, adc2, adc3;
       float volts0, volts1, volts2, volts3;

      adc0 = ads.readADC_SingleEnded(0);
      adc1 = ads.readADC_SingleEnded(1);
      adc2 = ads.readADC_SingleEnded(2);
      adc3 = ads.readADC_SingleEnded(3);

      volts0 = ads.computeVolts(adc0);
      volts1 = ads.computeVolts(adc1);
      volts2 = ads.computeVolts(adc2);
      volts3 = ads.computeVolts(adc3);
      
      Serial.println("-----------------------------------------------------------");
      Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
      Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.println("V");
      Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
      Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");
}
void anenometer(){
    detachInterrupt(interruptPin); //shut off wind speed measurement interrupt until done communication
      float aWSpeed = getAvgWindSpeed(culPulseTime,avgWindCount); //calculate average wind speed
      if(aWSpeed >= aSetting) digitalWrite(13, HIGH);   // high speed wind detected so turn the LED on
      else digitalWrite(13, LOW);   //no alarm so ensure LED is off
      culPulseTime = 0; //reset cumulative pulse counter
      avgWindCount = 0; //reset average wind count

      float aFreq = 0; //set to zero initially
      if(pulseTime > 0.0) aFreq = getAnemometerFreq(pulseTime); //calculate frequency in Hz of anemometer, only if pulsetime is non-zero
      float wSpeedMPH = getWindMPH(aFreq); //calculate wind speed in MPH, note that the 2.5 comes from anemometer data sheet
      //Serial.begin(9600); //start serial monitor to communicate wind data
      Serial.println();
      Serial.println("...................................");
      Serial.print("Anemometer speed in Hz ");
      Serial.println(aFreq);
      Serial.print("Current wind speed is ");
      Serial.println(wSpeedMPH);
      Serial.print("Current average wind speed is ");
      Serial.println(aWSpeed);
      Serial.end(); //serial uses interrupts so we want to turn it off before we turn the wind measurement interrupts back on
    
      start = true; //reset start variable in case we missed wind data while communicating current data out
      attachInterrupt(digitalPinToInterrupt(interruptPin), anemometerISR, RISING); //turn interrupt back on
      dataTimer = millis(); //reset loop timer
}
void temp(){
  Serial.print("Humidity:    ");
  Serial.print(sensor.readHumidity(), 2);
  Serial.print("\tTemperature: ");
  Serial.println(sensor.readTemperature(), 2);
  delay(1000);

  // Toggle heater enabled state every 30 seconds
  // An ~1.8 degC temperature increase can be noted when heater is enabled
 // if (++loopCnt == 30) {
    enableHeater = !enableHeater;
    sensor.heater(enableHeater);
    Serial.print("Heater Enabled State: ");
    if (sensor.isHeaterEnabled())
      Serial.println("ENABLED");
    else
      Serial.println("DISABLED");
       
   // loopCnt = 0;
  //}

}

void wifi_send_data() {
  //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;

      String serverPath = serverName + "?temperature=24.37";
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());
      
      // If you need Node-RED/server authentication, insert user and password below
      //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
      
      // Send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
}

void setup() {

Serial.begin(9600);

WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  //Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");


//-------analog setup here
 

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
   ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
   //ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  //ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

//--------------
  pinMode(13, OUTPUT); //setup LED pin to signal high wind alarm condition
  pinMode(interruptPin, INPUT_PULLUP); //set interrupt pin to input pullup
  attachInterrupt(interruptPin, anemometerISR, RISING); //setup interrupt on anemometer input pin, interrupt will occur whenever falling edge is detected
  dataTimer = millis(); //reset loop timer

  Serial.println("Si7021 test!");
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true)
      ;
  }
  Serial.print("Found model ");
  switch(sensor.getModel()) {
    case SI_Engineering_Samples:
      Serial.print("SI engineering samples"); break;
    case SI_7013:
      Serial.print("Si7013"); break;
    case SI_7020:
      Serial.print("Si7020"); break;
    case SI_7021:
      Serial.print("Si7021"); break;
    case SI_UNKNOWN:
    default:
      Serial.print("Unknown");
  }
  Serial.print(" Rev(");
  Serial.print(sensor.getRevision());
  Serial.print(")");
  Serial.print(" Serial #"); Serial.print(sensor.sernum_a, HEX); Serial.println(sensor.sernum_b, HEX);



}

void loop() {
  
 unsigned long rTime = millis();
  
if((rTime - sTime) > 2500) pulseTime = 0; //if the wind speed has dropped below 1MPH than set it to zero
    
if((rTime - dataTimer) > 6000){ //See if it is time to transmit
    anenometer();
  } else if((rTime - dataTimer) > 3000) { 
    temp();
  } else if((rTime - dataTimer) > 9000) {
    adc();
  } else if((rTime - dataTimer) > 11000) {
    wifi_send_data();
  }
  
delay(1000);
}


