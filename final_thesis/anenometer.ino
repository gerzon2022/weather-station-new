
float windVal = 0;
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
      windVal = getWindKPH(wSpeedMPH);
      //Serial.begin(9600); //start serial monitor to communicate wind data
      Serial.println();
      Serial.println("...................................");
      Serial.print("Anemometer speed in Hz ");
      Serial.println(aFreq);
      Serial.print("Current wind speed is ");
      Serial.println(wSpeedMPH);
      Serial.print("Current average wind speed is ");
      Serial.println(aWSpeed);
      //Serial.end(); //serial uses interrupts so we want to turn it off before we turn the wind measurement interrupts back on
    
      start = true; //reset start variable in case we missed wind data while communicating current data out
      attachInterrupt(digitalPinToInterrupt(interruptPin), anemometerISR, RISING); //turn interrupt back on
      dataTimer = millis(); //reset loop timer
}