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
void Text(char *text, int x, int y)
{
  lcd.setCursor(x,y);
  lcd.print(text);
}  
//=======================================================================
void Number(int number, int x, int y)
{
  lcd.setCursor(x,y);
  lcd.printLong(number);
}  
//=======================================================================

void splashScreen()
{
  lcd.clear();
  lcd.setFont(FONT_SIZE_SMALL);
  Text("  ARDUINO MICRO PRO  ", 0, 0);
  Text("  SOLDERING STATION  ", 0, 2);
  Text("   MANOS MAR. 2016   ", 0, 4);
  Text("    96W/25-400 oC    ", 0, 6);  
  delay(2000);
  lcd.clear();

  for(int i=0; i<=400; i++)
  {
    lcd.setFont(FONT_SIZE_XLARGE);
    Number(i, 0, 0);
    lcd.print(" oC");
    delay(250);
  }  
  lcd.clear();
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
//=======================================================================
void drawRoundRect(int16_t x, int16_t y, int16_t w,int16_t h, int16_t r) 
{
  // smarter version
  drawFastHLine(x+r  , y    , w-2*r); // Top
  drawFastHLine(x+r  , y+h-1, w-2*r); // Bottom
  drawFastVLine(x    , y+r  , h-2*r); // Left
  drawFastVLine(x+w-1, y+r  , h-2*r); // Right
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1);
  drawCircleHelper(x+w-r-1, y+r    , r, 2);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4);
  drawCircleHelper(x+r    , y+h-r-1, r, 8);
}
//=======================================================================
void drawFastVLine(int16_t x, int16_t y, int16_t h) 
{
  drawLine(x, y, x, y+h-1);
}
//=======================================================================
void drawFastHLine(int16_t x, int16_t y, int16_t w) 
{
  drawLine(x, y, x+w-1, y);
}
//=======================================================================
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) 
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } 
  else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(y0, x0);
    } 
    else {
      drawPixel(x0, y0);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}
//=======================================================================
void drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername) 
{
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) 
    {
      drawPixel(x0 + x, y0 + y);
      drawPixel(x0 + y, y0 + x);
    } 
    if (cornername & 0x2) 
    {
      drawPixel(x0 + x, y0 - y);
      drawPixel(x0 + y, y0 - x);
    }
    if (cornername & 0x8) 
    {
      drawPixel(x0 - y, y0 + x);
      drawPixel(x0 - x, y0 + y);
    }
    if (cornername & 0x1) 
    {
      drawPixel(x0 - y, y0 - x);
      drawPixel(x0 - x, y0 - y);
    }
  }
}
//=======================================================================
void drawPixel(int16_t x, int16_t y) 
{
  swap(x, y);
  x = 128 - x - 1; 
  buffer[x + (y / 8) * DISPLAY_WIDTH] |=  (1 << (y & 7)); 
  lcd.draw(dot, x, y);
}
//=======================================================================
//void draw(const PROGMEM byte* buffer, byte width, byte height);


//=======================================================================
//=======================================================================

