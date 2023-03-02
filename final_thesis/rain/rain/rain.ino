



// #include <Wire.h>



// const int rainGauge = 12;   //Pin for rain gauge which has a resolution of .15mm
// const int ledPin = 4;
 float rainAmount = .3;     //variable to hold .3mm when bucket tips
 float totalAmount;          //variable to hold total amount of rain that has fallen
// long rssi;  //radio signal strength indicator
// String mymin = " minutes";

// char auth[] = "XXXXXXXXXXXXX";
// char ssid[] = "NotTelling";  // Your WiFi credentials.
// char pass[] = "NorHere";     // Set password to "" for open networks.




// void setup()
// {
//   attachInterrupt(rainGauge, rain, RISING); //interrupt to catch pulses sent from rain gauge
//   Serial.begin(9600); 
  
  
//     pinMode(ledPin, OUTPUT);
//     pinMode(rainGauge, INPUT_PULLUP);
   
    
// }


//   void rain(){
//       int buttonState = digitalRead(rainGauge);
//       if(buttonState==0){  //it is raining
//       digitalWrite (ledPin,HIGH);   //LED for visual feedback that pulses from rain gauge are being received
//       totalAmount += rainAmount; //add up the total amount of rain that has fallen and put it in variable totalAmount
//       Serial.print("this is the total rain that has fallen ");  //Print out to serial monitor
//       Serial.print(totalAmount); 
//       Serial.println(" mm");    
//       digitalWrite(ledPin,LOW);
//     }
//   }

// // BLYNK_WRITE(V12){  // this function returnes WiFi signal strength
// //   rssi = WiFi.RSSI();
// //   int pinData = param.asInt(); 
// //   if(pinData==1)
// //   {
// //     lcd.clear();
// //     lcd.print(0 ,0,"Wifi Strength");   
// //     lcd.print(0 ,1,rssi);
// //   }
// //  }

// //  bool isFirstConnect = true; // Keep this flag not to re-sync on every reconnection

// //   // This function will run every time Blynk connection is established
// //   BLYNK_CONNECTED() 
// //   {
// //     if (isFirstConnect) 
// //     {
// //       // Request Blynk server to re-send latest values for all pins
// // //      Blynk.syncAll();
// //       // You can also update an individual Virtual pin like this:
// //       Blynk.syncVirtual(V0);
// //       Blynk.syncVirtual(V1);
// //       Blynk.syncVirtual(V2);
// //       Blynk.syncVirtual(V3);
// //       Blynk.syncVirtual(V4);
// //       Blynk.syncVirtual(V11);
// //       Blynk.syncVirtual(V13);
// //       isFirstConnect = false;
// //     }
// //   }
  

// void loop()
// {
  

// }
struct Button {
	const uint8_t PIN;
	float numberKeyPresses;
	bool pressed;
};
//miles change pin from 12 to 18, new pin nato is ground and pin 18, wait
Button button1 = {16, 0, false};

void IRAM_ATTR Rain_isr() {
	totalAmount += rainAmount;
	button1.pressed = true;
}

void setup() {
	Serial.begin(115200);
	pinMode(button1.PIN, INPUT_PULLUP);
	attachInterrupt(button1.PIN, Rain_isr, FALLING);
}

void loop() {
	if (button1.pressed) {
		Serial.printf(" %.2f mm of rain \n", totalAmount);
    
		button1.pressed = false;
	}

  delay(200);
}
