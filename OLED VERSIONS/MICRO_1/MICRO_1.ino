/*********************************************************************
This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

This example is for a 128x64 size display using I2C to communicate
3 pins are required to interface (2 I2C and one reset)

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

/*********************************************************************
I change the adafruit SSD1306 to SH1106

SH1106 driver don't provide several functions such as scroll commands.

*********************************************************************/

#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROMEx.h>
#include <Encoder1.h>
#include <PinChangeInt.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <StopWatch.h>
//=======================================================
#define VERSION "v1.6"		
#define INTRO

#define OLED_RESET 15

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
//=======================================================
//=======================================================
int pwm = 0; //pwm Out Val 0.. 255
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

double aggKp=1.00, aggKi=0.002, aggKd=8.00;
double consKp=0.50, consKi=0.001, consKd=4.00;

/*
double aggKp=8.00, aggKi=0.10, aggKd=4.00;
double consKp=4.00, consKi=0.05, consKd=2.00;
*/

int seconds, minutes, hours = 0;
int secs, mins = 0;

int clickFunction = 0;
unsigned int actual_temperature, will_temp = STANDBY_TEMP;
volatile float fast = 1.0f;
volatile float slow = 1.0f;
char *text[50];
//=======================================================
Adafruit_SH1106 display(OLED_RESET);
Encoder1 myEncoder = Encoder1(Encoder1ChnA,Encoder1ChnB,EncoderDetent);
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
StopWatch sw_millis;    
StopWatch sw_countdownmillis;  
StopWatch sw_secs(StopWatch::SECONDS);
StopWatch sw_countdownsecs(StopWatch::SECONDS);
//=======================================================
void setup()   
{  
  Wire.begin();   
//  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  Serial.begin(9600);
  
  Serial.println("Sketch'es location:");
  Serial.println("D:\\MANOS\\DXP\\AVR SOLDERING IRON\\FIRMWARE\\MICRO\\MICRO_1\\MICRO_1.ino");
  
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

  myEncoder.setMinMax(MIN_TEMP,MAX_TEMP);

  beepBuzzer(6250,80);
//  splashScreen();
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
/*  
  display.clearDisplay();  
  display.setTextSize(2);
  display.setTextColor(WHITE);
  
  for(int i = 128; i<=255; i++)
  {    
    display.setCursor(0,0);
    display.write(i);
    display.display();
    beepBuzzer(6250,2);
    delay(1000); 
    display.clearDisplay();   
  }
  
  splashScreen();*/
}

//=======================================================================
void loop() 
{
  
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

