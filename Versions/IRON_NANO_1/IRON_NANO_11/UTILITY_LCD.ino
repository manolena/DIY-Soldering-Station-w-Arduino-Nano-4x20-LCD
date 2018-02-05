//=======================================================
void customChars()
{
  uint8_t iron1[8] = {     // Custom Character 2
    B00001,
    B00010,
    B00100,
    B01110,
    B01110,
    B01110,
    B01110,
    B01110
  };

  uint8_t iron2[8] = {     // Custom Character 3
    B01110,
    B11111,
    B11111,
    B00100,
    B00100,
    B00100,
    B00100,
    B00100
  };

  lcd.createChar(6,iron1);
  lcd.createChar(7,iron2);
}
//=======================================================================
byte *getChar(int n, byte newChar[]) 
{
  int i;
  byte code[5] = 
  {
    B10000,
    B11000,
    B11100,
    B11110,
    B11111
  }; 
  for (i = 0; i < 8; i++)
    newChar[i] = code[n - 1];
  return newChar;
}
//=======================================================================
void splashScreen()
{
  lcd.setCursor(0,0);
  lcd.print(F("    ARDUINO NANO    "));
  lcd.setCursor(0,1);
  lcd.print(F(" SOLDERING STATION  "));
  lcd.setCursor(0,2);
  lcd.print(F("  MANOS MAR. v1.11  "));
  lcd.setCursor(0,3);
  lcd.print(F("    96W/25-400"));
  lcd.write(223);
  lcd.print(F("C    "));
//  analogWrite(PWMpin, 255);
  delay(5000);
  lcd.clear();
}  
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
void shutDownWarning()
{
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print(F(" THE DEVICE WILL BE "));
  lcd.setCursor(0,2);
  lcd.print(F("SHUT DOWN in 10 MINS"));
  delay(5000);
  lcd.clear();
}
//=======================================================================
void shutDown()
{
  lcd.clear();
  lcd.setBacklight(HIGH);
  lcd.setCursor(0,1);
  lcd.print(F(" THE DIVICE WILL BE "));
  lcd.setCursor(0,2);
  lcd.print(F("      SHUT DOWN!    "));
  delay(5000);
  lcd.clear();
  lcd.setBacklight(LOW);
}
//=======================================================================
void showIron(int row, int line)
{
  lcd.setCursor(row,line);
  lcd.write(byte(6));
  lcd.setCursor(row,line+1);
  lcd.write(byte(7));
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
int getTemperatureCelsius()
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
//  analogWrite(PWMpin, pwm);	//switch heater back to last value
  return round(((float) adcValue)*ADC_TO_TEMP_GAIN + ADC_TO_TEMP_OFFSET); //apply linear conversion to actual temperature
}
//=======================================================================
void print_histogram(float val, float maxVal) 
{
  int i;
  int bloks;
  float histogram;

  lcd.noBlink(); 
  histogram = (SIZE_BAR * val) / maxVal;
  histogram = histogram + 0.5; 
  bloks = (int)histogram / 5;

  for (i = 0; i < bloks; i++)
  {
    lcd.write(5); 
  }  
  if ((int)(histogram) % 5 > 0)
  {
    lcd.write((int)(histogram) % 5);    
  }
  lcd.print(" ");  
}
//=======================================================================
