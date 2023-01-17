#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_ADS1X15.h>

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
//dht11 lib till here --

#define DHTPIN 2     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
//--dht var till here

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

// void setup(void)
// {
//   Serial.begin(9600);
//   Serial.println("Hello!");

//   Serial.println("Getting single-ended readings from AIN0..3");
//   Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

//   // The ADC input range (or gain) can be changed via the following
//   // functions, but be careful never to exceed VDD +0.3V max, or to
//   // exceed the upper and lower limits if you adjust the input range!
//   // Setting these values incorrectly may destroy your ADC!
//   //                                                                ADS1015  ADS1115
//   //                                                                -------  -------
//    ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
//   // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
//    //ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
//   // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
//   // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
//   //ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

//   if (!ads.begin()) {
//     Serial.println("Failed to initialize ADS.");
//     while (1);
//   }
// }

// void loop(void)
// {
//   int16_t adc0, adc1, adc2, adc3;
//   float volts0, volts1, volts2, volts3;

//   adc0 = ads.readADC_SingleEnded(0);
//   adc1 = ads.readADC_SingleEnded(1);
//   adc2 = ads.readADC_SingleEnded(2);
//   adc3 = ads.readADC_SingleEnded(3);

//   volts0 = ads.computeVolts(adc0);
//   volts1 = ads.computeVolts(adc1);
//   volts2 = ads.computeVolts(adc2);
//   volts3 = ads.computeVolts(adc3);
// Serial.println("test");
//   Serial.println("-----------------------------------------------------------");
//   Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
//   Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.println("V");
//   Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
//   Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");







//   delay(1000);
// }



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

void setup() {

Serial.begin(9600);
// Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));





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
}

void loop() {
  Serial.begin(9600);
  Serial.println("Hello!");

  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

 unsigned long rTime = millis();
  
  if((rTime - sTime) > 2500) pulseTime = 0; //if the wind speed has dropped below 1MPH than set it to zero
     
  if((rTime - dataTimer) > 6000){ //See if it is time to transmit
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
    } else if((rTime - dataTimer) > 3000) { //See if it is time to transmit
      // Get temperature event and print its value.
        sensors_event_t event;
        dht.temperature().getEvent(&event);
        if (isnan(event.temperature)) {
          Serial.println(F("Error reading temperature!"));
        }
        else {
          Serial.print(F("Temperature: "));
          Serial.print(event.temperature);
          Serial.println(F("°C"));
        }
        // Get humidity event and print its value.
        dht.humidity().getEvent(&event);
        if (isnan(event.relative_humidity)) {
          Serial.println(F("Error reading humidity!"));
        }
        else {
          Serial.print(F("Humidity: "));
          Serial.print(event.relative_humidity);
          Serial.println(F("%"));
        }

    } else {
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
      // delay(500);
    }
  delay(500);
}


