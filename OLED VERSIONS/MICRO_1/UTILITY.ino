//=======================================================================
//    UTILITIES
//=======================================================================
void beepBuzzer(unsigned long hz, unsigned long ms) 
{ 
  unsigned long us = (750000 / hz);  
  unsigned long rep = (ms * 500L) / us; 

  for (int i = 0; i < rep; i++) 
  {  
    digitalWrite(BUZZER_PIN, HIGH);  
    delayMicroseconds(us);  
    digitalWrite(BUZZER_PIN, LOW);  
    delayMicroseconds(us);  
  }
}
//=======================================================================
void setPwmFrequency(int pin, int divisor) 
{
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) 
  {
    switch(divisor) {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 64: 
      mode = 0x03; 
      break;
    case 256: 
      mode = 0x04; 
      break;
    case 1024: 
      mode = 0x05; 
      break;
    default: 
      return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } 
    else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } 
  else if(pin == 3 || pin == 11) 
  {
    switch(divisor) 
    {
    case 1: 
      mode = 0x01; 
      break;
    case 8: 
      mode = 0x02; 
      break;
    case 32: 
      mode = 0x03; 
      break;
    case 64: 
      mode = 0x04; 
      break;
    case 128: 
      mode = 0x05; 
      break;
    case 256: 
      mode = 0x06; 
      break;
    case 1024: 
      mode = 0x7; 
      break;
    default: 
      return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
//=======================================================================
int getTemperature()
{  
  analogWrite(PWMpin, 0);		//switch off heater
  delay(DELAY_MEASURE);			//wait for some time (to get low pass filter in steady state)

  total = total - readings[readIndex];
  readings[readIndex] = analogRead(TEMPin);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) 
  {
    readIndex = 0;
  }
  adcValue = total / numReadings;
//  adcValue = 128;
  analogWrite(PWMpin, pwm);	//switch heater back to last value
  return round(((float) adcValue)*ADC_TO_TEMP_GAIN+ADC_TO_TEMP_OFFSET); //apply linear conversion to actual temperature
}
//=======================================================================
void Text(char *text, int size, int x, int y, byte color)
{
  display.clearDisplay();
  display.setTextSize(size);
  display.setTextColor(color);
  display.setCursor(x,y);
  display.println(text);
  display.display();
}  
//=======================================================================
void splashScreen()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(F("ARDUINO MICRO PRO"));
  display.setCursor(0,10);
  display.println(F("SOLDERING STATION"));
  display.setCursor(0,20);
  display.println(F("MANOS MAR. 2016"));
  display.setCursor(0,30);
  display.print(F("96W/25-400 oC"));  
  display.display();
  delay(5000);
  display.clearDisplay();
}  
//=======================================================================
void checkTimer()
{
  if(mins >= TIMER_10MIN)
  {
    
  }  
  if(mins >= TIMER_20MIN)
  {
    pwm = 0;
    digitalWrite(HEAT_LED, LOW);
    heater = LOW;   
      
    sw_millis.reset();  
    sw_millis.stop();    
    seconds = 0;
    minutes = 0;
    sw_countdownmillis.reset();
    sw_countdownmillis.stop();
    secs = 0;
    mins = 0;
  }  
}  
