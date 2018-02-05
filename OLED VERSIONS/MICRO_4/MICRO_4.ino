/*

 Menu.pde
 
 Simple Menu Selection
 
 >>> Before compiling: Please remove comment from the constructor of the 
 >>> connected graphics display (see below).
 
 Universal 8bit Graphics Library, https://github.com/olikraus/u8glib/
 
 Copyright (c) 2012, olikraus@gmail.com
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without modification, 
 are permitted provided that the following conditions are met:
 
 * Redistributions of source code must retain the above copyright notice, this list 
 of conditions and the following disclaimer.
 
 * Redistributions in binary form must reproduce the above copyright notice, this 
 list of conditions and the following disclaimer in the documentation and/or other 
 materials provided with the distribution.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
 
 
 */


#include <U8glib.h>
#include <Wire.h>
#include <EEPROMEx.h>
#include <Encoder1.h>
#include <PinChangeInt.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <StopWatch.h>



#define KEY_NONE 0
#define KEY_PREV 1
#define KEY_NEXT 2
#define KEY_SELECT 3
#define KEY_BACK 4

#define VERSION "v1.6"		
#define INTRO

#define LCD_I2C_ADDRESS      0x27
#define ROWS 4
#define COLUMNS 20

#define ADC_TO_TEMP_GAIN 	1.8//0.99//2.50//0.53 //0.415
#define ADC_TO_TEMP_OFFSET      25.0
#define STANDBY_TEMP		175
#define MAX_TEMP		400
#define MIN_TEMP	        25 // Minimum setpoint temperature
#define MAX_PWM_LOW		50//180
#define MAX_PWM_HI		255//210//240
#define PWM_DIV                 1024	

#define Encoder1ChnA          2
#define Encoder1ChnB          3
#define EncoderDetent         4

#define BUZZER_PIN            5
#define HEAT_LED              6
#define STANDBYin             7
#define TEMPin 	              A0
#define PWMpin 	              8

#define WILL_TEMP_EEPROM_ADDRESS 0x10
#define ENCODER_EEPROM_ADDRESS   0x20

#define DELAY_MAIN_LOOP 	1//150
#define DELAY_MEASURE 		2

#define SIZE_BAR (9 * 5)
#define TIMER_10MIN            10//0
#define TIMER_20MIN            20//0
//=======================================================================
uint8_t uiKeyPrev = 7;
uint8_t uiKeyNext = 3;
uint8_t uiKeySelect = 2;
uint8_t uiKeyBack = 8;

uint8_t uiKeyCodeFirst = KEY_NONE;
uint8_t uiKeyCodeSecond = KEY_NONE;
uint8_t uiKeyCode = KEY_NONE;

#define MENU_ITEMS 5
const char *menu_strings[MENU_ITEMS] = { 
  "MEMORY", "SET MAX TEMP", "SET MIN TEMP", "SET STBY TEMP", "UNITS" };

uint8_t menu_current = 0;
uint8_t menu_redraw_required = 0;
uint8_t last_key_code = KEY_NONE;


int pwm = 0; //pwm Out Val 0.. 255
unsigned int actual_temperature, will_temp = STANDBY_TEMP;
int MAX_PWM;
boolean standby_act = false;
int tempDIV;
int will_temp_tmp;
float encoderValue = 0;
volatile float encoderPos = 0;
volatile float encoderPosTemp = 0;
boolean memWrite = false;
boolean memNoWrite = true;
boolean state = false;
boolean heater = false;
boolean unplug = 0;
int t1,t2;
int p1;
static boolean rotating = false;
const int numReadings = 30;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
float adcValue;

//PID parameters
double Setpoint, Input, Output;
double aggKp=8.00, aggKi=0.10, aggKd=4.00;
double consKp=4.00, consKi=0.05, consKd=2.00;

int seconds, minutes, hours = 0;
int secs, mins = 0;
//=======================================================================
Encoder1 myEncoder = Encoder1(Encoder1ChnA,Encoder1ChnB,EncoderDetent);
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
StopWatch sw_millis;    
StopWatch sw_countdownmillis;  
StopWatch sw_secs(StopWatch::SECONDS);
StopWatch sw_countdownsecs(StopWatch::SECONDS);
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);	// I2C / TWI 
//======================================================================= 
void setup() 
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Sketch'es location:");
  Serial.println("C:\\Users\\Administrator\\Documents\\DXP\\AVR SOLDERING IRON\\FIRMWARE\\MICRO\\MICRO_3");

  byte newChar[8];
  int i; 

  pinMode(Encoder1ChnA, INPUT); 
  digitalWrite(Encoder1ChnA, HIGH); //turn pullup resistor on
  pinMode(Encoder1ChnB, INPUT); 
  digitalWrite(Encoder1ChnB, HIGH); //turn pullup resistor on
  pinMode(EncoderDetent, INPUT); 
  digitalWrite(EncoderDetent, HIGH); //turn pullup resistor on 

  PCintPort::attachInterrupt(Encoder1ChnA, &updateEncoder_ISR, CHANGE); 
  PCintPort::attachInterrupt(Encoder1ChnB, &updateEncoder_ISR, CHANGE);
  PCintPort::attachInterrupt(EncoderDetent, &EncoderClick_ISR, FALLING);

  Timer1.initialize(150000); // set a timer of length 150000 microseconds (or 0.15 sec)
  Timer1.attachInterrupt(timer1_ISR); // attach the service routine here  

  pinMode(9, OUTPUT);
  digitalWrite(9,HIGH);  
  pinMode(10, OUTPUT);
  digitalWrite(10,LOW);  

  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);  

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN,HIGH);

  pinMode(STANDBYin, INPUT_PULLUP);

  pinMode(TEMPin, INPUT);
  digitalWrite(TEMPin, LOW);

  pinMode(HEAT_LED, OUTPUT);
  digitalWrite(HEAT_LED, LOW);     

  myEncoder.setRate(1.0f);
  myEncoder.setMinMax(MIN_TEMP,MAX_TEMP);

  beepBuzzer(6250,80);

  u8g.firstPage();
  do 
  { 
    splashScreen();
  } 
  while( u8g.nextPage()); 

  delay(5000);
  sw_secs.start();
  sw_millis.start(); 
  sw_countdownmillis.start(); 

  will_temp = EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS);
  myEncoder.setPosition(will_temp); 
  if (will_temp > MAX_TEMP) 
  {
    will_temp = STANDBY_TEMP;
  }
  encoderPos = EEPROM.readInt(ENCODER_EEPROM_ADDRESS);
  if (encoderPos == MAX_TEMP) 
  {    
    myEncoder.setPosition(will_temp);
  }

  Input = getTemperature();
  Setpoint = will_temp;
  myPID.SetMode(AUTOMATIC);
  pwm = 0; 
  //  lcd.setCursor(10,3); 
  //  lcd.print("MEM=");   
  //  lcd.print(will_temp);
  // lcd.write(223); 
  // lcd.print("C ");  


  //uiSetup();                                // setup key detection and debounce algorithm
  //menu_redraw_required = 0;     // force initial redraw
}
//=======================================================================
void loop() 
{ 
  u8g.firstPage();  
  do 
  {  
    unsigned long now = millis();
    static boolean oneTime = false; 

    Input = getTemperature();

    if(standby_act == false)
    {    
      Setpoint = encoderPos;
    }
    else
    {
      Setpoint = STANDBY_TEMP;
    }    
    double gap = abs(Setpoint-Input); 
    if(gap < 10)
    {  
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute();   

    will_temp = map(encoderPos, MIN_TEMP, MAX_TEMP, MIN_TEMP, MAX_TEMP);
    int tempWill = EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS);  


    if (digitalRead(STANDBYin) == HIGH)
    {
      standby_act = false;
      //lcd.setCursor(19,0);
      if(oneTime)
      { 

        showCountdownTime(13,1); 
        will_temp = STANDBY_TEMP;
        u8g.setFont(u8g_font_6x13);
        u8g.drawStr(0, 22, "STAND-BY");

        oneTime = false;
        seconds = 0;
        minutes = 0;
        sw_millis.reset();
        sw_millis.start(); 
      }  
      sw_countdownmillis.reset();
      sw_countdownmillis.start();
      secs = 0;
      mins = 0; 
      //      lcd.setCursor(13,1);
      //      lcd.print("     ");    
      //      lcd.setCursor(19,0);    
      //      showIron(19,0);         
    }  
    else
    {
      standby_act = true;
      sw_millis.stop();
      sw_millis.reset();
      seconds = 0;
      minutes = 0;

      if(!oneTime)
      {
        sw_countdownmillis.reset();
        sw_countdownmillis.start();
        secs = 0;
        mins = 0;

        //showIron(19,0);    


        oneTime = true;      
      }
      showCountdownTime(13,1); 
      will_temp = STANDBY_TEMP;
    }

    if((memNoWrite == true) && (memWrite = false))
    {       
      memNoWrite = true; 
      memWrite = false; 
    }  
    else if((memNoWrite == false) && (memWrite = true))
    {
      u8g.setPrintPos(64, 50);
      u8g.print(tempWill);
    }
  } 
  while(u8g.nextPage()); 

  memNoWrite = true; 
  memWrite = false; 

  will_temp_tmp = will_temp;

  if ((standby_act && (will_temp >= STANDBY_TEMP))) 
  {
    will_temp = STANDBY_TEMP;    
  }  

  actual_temperature = getTemperature();

  checkUnplugged();
  checkTimer(); 
  sw_millis.start(); 

  pwm = Output;
  MAX_PWM = actual_temperature <= STANDBY_TEMP ? MAX_PWM_LOW : MAX_PWM_HI;
  pwm = pwm > MAX_PWM ? pwm = MAX_PWM : pwm < 0 ? pwm = 0 : pwm;
  analogWrite(PWMpin, pwm);

  if(pwm != 0)
  {
    heater = HIGH;
  }
  else 
  {    
    heater = LOW;
  }
  writeHEATING(will_temp, Input, pwm);
  while(u8g.nextPage()); 
  


  //   showTemps(); 
  //uiStep();                                     // check for key press
  /*
  if (  menu_redraw_required != 0 ) {
   u8g.firstPage();
   do  {
   drawMenu();
   } 
   while( u8g.nextPage() );
   menu_redraw_required = 0;
   }
   updateMenu();                            // update menu bar
   */

  
}

//=======================================================================
void uiSetup(void) 
{
  // configure input keys 

  pinMode(uiKeyPrev, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeyNext, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeySelect, INPUT_PULLUP);           // set pin to input with pullup
  pinMode(uiKeyBack, INPUT_PULLUP);           // set pin to input with pullup
}
//======================================================================= 
void uiStep(void) {
  uiKeyCodeSecond = uiKeyCodeFirst;
  if ( digitalRead(uiKeyPrev) == LOW )
    uiKeyCodeFirst = KEY_PREV;
  else if ( digitalRead(uiKeyNext) == LOW )
    uiKeyCodeFirst = KEY_NEXT;
  else if ( digitalRead(uiKeySelect) == LOW )
    uiKeyCodeFirst = KEY_SELECT;
  else if ( digitalRead(uiKeyBack) == LOW )
    uiKeyCodeFirst = KEY_BACK;
  else 
    uiKeyCodeFirst = KEY_NONE;

  if ( uiKeyCodeSecond == uiKeyCodeFirst )
    uiKeyCode = uiKeyCodeFirst;
  else
    uiKeyCode = KEY_NONE;
}
//=======================================================================
void drawMenu(void) {
  uint8_t i, h;
  u8g_uint_t w, d;

  u8g.drawFrame(0, 0, 128, 64);
  u8g.setFont(u8g_font_6x13);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();

  h = u8g.getFontAscent()-u8g.getFontDescent();
  w = u8g.getWidth();
  for( i = 0; i < MENU_ITEMS; i++ ) {
    d = (w-u8g.getStrWidth(menu_strings[i]))/2;
    u8g.setDefaultForegroundColor();
    if ( i == menu_current ) {
      u8g.drawBox(0, i*h+1, w, h);
      u8g.setDefaultBackgroundColor();
    }
    u8g.drawStr(d, i*h, menu_strings[i]);
  }
}
//=======================================================================
void updateMenu(void) {
  if ( uiKeyCode != KEY_NONE && last_key_code == uiKeyCode ) {
    return;
  }
  last_key_code = uiKeyCode;

  switch ( uiKeyCode ) {
  case KEY_NEXT:
    menu_current++;
    if ( menu_current >= MENU_ITEMS )
      menu_current = 0;
    menu_redraw_required = 1;
    break;
  case KEY_PREV:
    if ( menu_current == 0 )
      menu_current = MENU_ITEMS;
    menu_current--;
    menu_redraw_required = 1;
    break;
  }
}
//=======================================================================
//    ENCODER ISR
//=======================================================================
void updateEncoder_ISR() 
{
  myEncoder.lowLevelTick(1,1);
  encoderPos = myEncoder.getPosition();
  if(encoderPos <= MIN_TEMP)
  {
    myEncoder.setPosition(MIN_TEMP);
    encoderPos = MIN_TEMP;
  }  
  if(encoderPos >= MAX_TEMP) 
  {
    myEncoder.setPosition(MAX_TEMP);//1150
    encoderPos = MAX_TEMP;
  }
  beepBuzzer(6250,1);
}
//=======================================================================
//    ENCODER'S DETENT ISR
//=======================================================================
void EncoderClick_ISR() 
{  
  myEncoder.lowLevelClick();
  {
    EEPROM.writeInt(WILL_TEMP_EEPROM_ADDRESS, will_temp);
    EEPROM.writeInt(ENCODER_EEPROM_ADDRESS, encoderPos);
    myEncoder.setPosition(encoderPos);
    memWrite = true;
    memNoWrite = false; 
    beepBuzzer(6250,80);
  }
}
//=======================================================================
//    TIMER 1 ISR
//=======================================================================
void timer1_ISR()
{
  Timer1.detachInterrupt();
  state =!state;

  switch(heater)
  {
  case HIGH:
    digitalWrite(HEAT_LED, state);
    break;
  case LOW:
    digitalWrite(HEAT_LED, LOW);
    break;  
  }
  if (digitalRead(STANDBYin) == LOW) 
  {
    standby_act = true;
    sw_countdownsecs.start(); 
  }  
  else 
  {
    standby_act = false;
    sw_countdownsecs.stop();
    sw_countdownsecs.reset();  
  }  
  Timer1.attachInterrupt( timer1_ISR );
}
//=======================================================================


















