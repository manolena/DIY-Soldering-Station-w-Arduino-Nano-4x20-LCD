#include <LiquidCrystal_I2C.h>
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
#define VERSION "v1.11"		
#define INTRO

#define LCD_I2C_ADDRESS      0x27//0x3F
#define ROWS 4
#define COLUMNS 20

#define ADC_TO_TEMP_GAIN 	4.0//0.99//2.50//0.53 //0.415
#define ADC_TO_TEMP_OFFSET      25.0
#define STANDBY_TEMP		175
#define STANDBY_TEMP_F		347
#define MAX_TEMP		400
#define MAX_TEMP_F		752
#define MIN_TEMP	        150//25 // Minimum setpoint temperature
#define MIN_TEMP_F	        77
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

#define DELAY_MAIN_LOOP 	100
#define DELAY_MEASURE 		30

#define SIZE_BAR (9 * 5)
#define TIMER_10MIN            10//0
#define TIMER_20MIN            20//0
//=======================================================
int pwm = 0; //pwm Out Val 0.. 255
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

//double aggKp=4.0, aggKi=0.2, aggKd=1.0;
//double consKp=1, consKi=0.05, consKd=0.25;
double aggKp=200.00, aggKi=300.00, aggKd=10.00;
double consKp=4.00, consKi=0.2, consKd=1.00;

int seconds, minutes, hours = 0;
int secs, mins = 0;

int clickFunction = 0;
boolean celsiusOrFahrenheit = true;
unsigned int actual_temperature, will_temp = STANDBY_TEMP;
boolean oneTimeON = false;
boolean oneTimeOFF = true;
boolean flag = false;
//=======================================================
Encoder1 myEncoder = Encoder1(Encoder1ChnA,Encoder1ChnB,EncoderDetent);
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
StopWatch sw_millis;    
StopWatch sw_countdownmillis;  
StopWatch sw_secs(StopWatch::SECONDS);
StopWatch sw_countdownsecs(StopWatch::SECONDS);

//LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 5, 6, 7, 2, 3, 4, 1, 0, NEGATIVE);  // Set the LCD I2C address
//=======================================================================
void setup() 
{
  Wire.begin();
  lcd.begin(COLUMNS, ROWS); 
  lcd.setBacklight(HIGH);
  Serial.begin(9600);
  Serial.println("Sketch'es location:");
  Serial.println("C:\\Users\\Administrator\\Documents\\DXP\\AVR SOLDERING IRON\\FIRMWARE\\NANO\\IRON_NANO_11");

  byte newChar[8];
  int i; 

  for (i = 0; i < 8; i++)
  {
    lcd.createChar(i, getChar(i, newChar));
  }  
  customChars();
  pinMode(Encoder1ChnA, INPUT); 
  digitalWrite(Encoder1ChnA, HIGH); //turn pullup resistor on
  pinMode(Encoder1ChnB, INPUT); 
  digitalWrite(Encoder1ChnB, HIGH); //turn pullup resistor on
  pinMode(EncoderDetent, INPUT); 
  digitalWrite(EncoderDetent, HIGH); //turn pullup resistor on 

  pinMode(PWMpin, OUTPUT);
  digitalWrite(PWMpin, LOW);
  digitalWrite(PWMpin, LOW);

  PCintPort::attachInterrupt(Encoder1ChnA, &updateEncoder_ISR, CHANGE); 
  PCintPort::attachInterrupt(Encoder1ChnB, &updateEncoder_ISR, CHANGE);
  PCintPort::attachInterrupt(EncoderDetent, &EncoderClick_ISR, FALLING);

  Timer1.initialize(350000); // set a timer of length 150000 microseconds (or 0.15 sec)
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
  splashScreen();
  analogWrite(PWMpin, 255);
  sw_secs.start();
  sw_millis.start(); 
  sw_countdownmillis.start(); 

  will_temp = EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS);
  myEncoder.setPosition(will_temp); 

  if (will_temp >= MAX_TEMP) 
  {
    will_temp = STANDBY_TEMP;
  }
  encoderPos = EEPROM.readInt(ENCODER_EEPROM_ADDRESS);
  if (encoderPos >= MAX_TEMP) 
  {    
    myEncoder.setPosition(will_temp);
  }
  Input = getTemperatureCelsius();
  Setpoint = will_temp;
  myPID.SetMode(AUTOMATIC);
  pwm = 0; 
  lcd.setCursor(10,3); 
  lcd.print("MEM=");   
  lcd.print(will_temp);  
  lcd.write(223); 
  lcd.print("C ");  
}
//=======================================================================
void loop()
{  
  unsigned long now = millis();  
  Input = getTemperatureCelsius();
  delay(DELAY_MAIN_LOOP);
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
    lcd.setBacklight(HIGH);
    standby_act = false;
    lcd.setCursor(19,0);
    if((!oneTimeON) && (oneTimeOFF))
    { 
      showCountdownTime(13,1);
      will_temp = STANDBY_TEMP;
      lcd.print(" ");
      lcd.setCursor(19,1);
      lcd.print(" ");
      oneTimeON = true;
      oneTimeOFF = false;
      lcd.setCursor(6,1);
      lcd.print("      ");
      lcd.setCursor(19,0);
      lcd.print(" ");
      seconds = 0;
      minutes = 0;
      sw_millis.reset();
      sw_millis.start();
      showIron(19,0);
    }     

//    sw_countdownmillis.reset();
//    sw_countdownmillis.start();
//    secs = 0;
//    mins = 0; 
    lcd.setCursor(13,1);
    lcd.print("     ");    
    showIron(19,0);         
  }  
  else if (digitalRead(STANDBYin) == LOW)
  {
    standby_act = true;
    sw_millis.stop();
    sw_millis.reset();
    seconds = 0;
    minutes = 0;
    if((oneTimeON) && (!oneTimeOFF))
    {
      sw_countdownmillis.reset();
      sw_countdownmillis.start();
//      secs = 0;
//      mins = 0; 
      lcd.setCursor(13,1);
      lcd.print("     ");   
      showIron(19,0);
      oneTimeON = false;  
      oneTimeOFF = true;  
    }
    showCountdownTime(13,1); 
    will_temp = STANDBY_TEMP;
    lcd.print(" ");
    lcd.setCursor(19,1);
    lcd.print(" ");
    lcd.setCursor(6,1);
    lcd.print("      ");
    lcd.setCursor(19,0);
    lcd.print(" ");    
  }

  if((memNoWrite == true) && (memWrite == false))
  {       
    memNoWrite = true; 
    memWrite = false; 
  }  
  else if((memNoWrite == false) && (memWrite == true))
  {
    lcd.setCursor(14,3);
    lcd.print(tempWill);
  } 
  memNoWrite = true; 
  memWrite = false; 

  will_temp_tmp = will_temp;  
  checkUnplugged();
  checkTimer(); 
  analogWrite(PWMpin, Output);

  if(Output != 0)
  {
    heater = HIGH;
  }
  else 
  {    
    heater = LOW;
  } 
  if(standby_act && (will_temp >= STANDBY_TEMP)) writeHEATING(STANDBY_TEMP, Input, Output);
  else if(!standby_act) writeHEATING(will_temp, Input, Output);
  lcd.setCursor(10,3); 
  lcd.print("MEM=");   
  lcd.print(EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS));  
  lcd.write(223); 
  lcd.print("C ");  
  showTemps();  
}
//=======================================================================
void checkTimer()
{  
  if((mins >= TIMER_10MIN) && (mins < TIMER_20MIN) && (!flag))
  {
    shutDownWarning();
//    will_temp = EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS);
    lcd.setCursor(10,3); 
    lcd.print("MEM=");   
    lcd.print(EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS));
    showTemps(); 
    if(standby_act && (will_temp >= STANDBY_TEMP)) writeHEATING(STANDBY_TEMP, Input, pwm);
    else if(!standby_act) writeHEATING(will_temp, Input, pwm);    
    flag = true;
  }  
  if(mins >= TIMER_20MIN)
  {
    shutDown();
    pwm = 0;
    analogWrite(PWMpin, pwm);
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
    while(digitalRead(STANDBYin) == LOW);
    lcd.setCursor(10,3); 
    lcd.print("MEM=");   
    lcd.print(EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS));
    showTemps();       
  }  
}  
//=======================================================================
void checkUnplugged()
{
  actual_temperature = getTemperatureCelsius();
  if((actual_temperature >= MAX_TEMP+50) && (actual_temperature < MAX_TEMP+100))
  {    
    pwm = 0;
    analogWrite(PWMpin, pwm);
    digitalWrite(HEAT_LED, LOW);
    actual_temperature = MAX_TEMP; 
    lcd.clear();

    do
    {      
      lcd.setCursor(5,0);
      lcd.print(F("UNPLUGGED!"));
      lcd.setCursor(0,1);
      lcd.print(F("   PLEASE CONNECT   "));
      lcd.setCursor(0,2);
      lcd.print(F("        PLUG!       "));
      lcd.setBacklight(HIGH);
      actual_temperature = getTemperatureCelsius();        
    }
    while(actual_temperature >= MAX_TEMP+50);

    lcd.setBacklight(HIGH);
    lcd.clear();

    lcd.setCursor(14,3); 
    lcd.print(will_temp);

    lcd.setCursor(4,2);
    if (t1 < 100)
      lcd.print(" ");
    if (t1 <10)
      lcd.print(" ");
    lcd.print(t1); 

    lcd.setCursor(4,3);
    if (t2 < 100)
      lcd.print(" ");
    if (t2 <10)
      lcd.print(" ");
    lcd.print(t2); 

    lcd.setCursor(14,2);
    if (p1 < 100)
      lcd.print(" ");
    if (p1 <10)
      lcd.print(" ");
    lcd.print(p1); 
  }
} 
//=======================================================================
void showTemps()
{
  lcd.setCursor(0,0);
  lcd.print(F("A>"));
  lcd.setCursor(0,1);
  lcd.print(F("S>"));
  lcd.setCursor(0,2);
  lcd.print(F("ACT="));
  lcd.setCursor(0,3);
  lcd.print(F("SET="));
  lcd.setCursor(10,2);
  lcd.print(F("PWM="));
  lcd.setCursor(10,3); 
  lcd.print("MEM="); 
  lcd.setCursor(7,2);
  lcd.write(223); 
  lcd.print("C ");
  lcd.setCursor(7,3);
  lcd.write(223); 
  lcd.print("C ");
  lcd.setCursor(17,3);
  lcd.write(223); 
  lcd.print("C ");  
  lcd.setCursor(17,2);
  lcd.print("% ");
  if(standby_act && (will_temp >= STANDBY_TEMP)) writeHEATING(STANDBY_TEMP, Input, pwm);
    else if(!standby_act) writeHEATING(will_temp, Input, pwm);  
  lcd.setCursor(2,0);
  print_histogram(t1,MAX_TEMP);  
  lcd.setCursor(2,1);
  print_histogram(t2,MAX_TEMP);
  showTime(13,0);
}   
//=======================================================================
void writeHEATING(int tempWILL, int tempVAL, int pwmVAL)
{
  static int d_tempWILL = 1;//2		
  static int tempWILL_OLD = 1;//10
  static int tempVAL_OLD = 1;//10
  static int pwmVAL_OLD	= 1;//10

  pwmVAL = map(pwmVAL, 0, 255, 0, 99);

  if (tempVAL_OLD != tempVAL)
  {
    lcd.setCursor(4,2);
    if ((tempVAL_OLD/100) != (tempVAL/100))
    {
      lcd.print(tempVAL_OLD/100);
    }
    else
      lcd.print(" ");

    if (((tempVAL_OLD/10)%10) != ((tempVAL/10)%10))
      lcd.print((tempVAL_OLD/10)%10);
    else
      lcd.print(" ");

    if ((tempVAL_OLD%10) != (tempVAL%10))
      lcd.print(tempVAL_OLD%10 );

    lcd.setCursor(4,2);
    if (tempVAL < 100)
      lcd.print(" ");
    if (tempVAL <10)
      lcd.print(" ");

    lcd.print(tempVAL); 
    t1 = tempVAL;
    tempVAL_OLD = tempVAL; 
  }
  if ((tempWILL_OLD+d_tempWILL < tempWILL) || (tempWILL_OLD-d_tempWILL > tempWILL))
  {
    lcd.setCursor(4,3);

    if ((tempWILL_OLD/100) != (tempWILL/100))
    {
      lcd.print(tempWILL_OLD/100);
    }
    else
      lcd.print(" ");

    if (((tempWILL_OLD/10)%10) != ((tempWILL/10)%10))
      lcd.print((tempWILL_OLD/10)%10 );
    else
      lcd.print(" ");

    if ((tempWILL_OLD%10) != (tempWILL%10))
      lcd.print(tempWILL_OLD%10 );

    lcd.setCursor(4,3);
    if (tempWILL < 100)
      lcd.print(" ");
    if (tempWILL <10)
      lcd.print(" ");

    lcd.print(tempWILL); 
    t2 = tempWILL;
    tempWILL_OLD = tempWILL;
  }

  if (pwmVAL_OLD != pwmVAL)
  {
    lcd.setCursor(14,2);
    if ((pwmVAL_OLD/100) != (pwmVAL/100))
    {
      lcd.print(pwmVAL_OLD/100);
    }
    else
      lcd.print(" ");

    if (((pwmVAL_OLD/10)%10) != ((pwmVAL/10)%10))
      lcd.print((pwmVAL_OLD/10)%10 );
    else
      lcd.print(" ");

    if ((pwmVAL_OLD%10) != (pwmVAL%10))
      lcd.print(pwmVAL_OLD%10 );

    lcd.setCursor(14,2);
    if (pwmVAL < 100)
      lcd.print(" ");
    if (pwmVAL <10)
      lcd.print(" ");

    lcd.print(pwmVAL);
    p1 = pwmVAL; 
    pwmVAL_OLD = pwmVAL;
  }    
}
//=======================================================================
void showCountdownTime(int row, int line)
{
  lcd.setCursor(row, line);
  if(sw_countdownmillis.elapsed() > 999)
  {
    secs++;
    sw_countdownmillis.reset();
    sw_countdownmillis.start();
  }  
  if(secs > 59)
  {
    secs = 0;
    mins++;
  }

  if(mins < 10)
  {
    lcd.print("0");
  } 
  lcd.print(mins,DEC);
  lcd.print(":");
  if(secs < 10)
  {
    lcd.print("0");
  } 
  lcd.print(secs,DEC); 

}  
//=======================================================================
void showTime(int line, int row)
{
  lcd.setCursor(line, row);
  if(sw_millis.elapsed() > 999)
  {
    seconds++;
    sw_millis.reset();
    sw_millis.start();
  }
  if(seconds > 59)
  {
    seconds = 0;
    minutes++;
  }

  if(minutes < 10)
  {
    lcd.print("0");
  } 
  lcd.print(minutes,DEC);
  lcd.print(":");
  if(seconds < 10)
  {
    lcd.print("0");
  } 
  lcd.print(seconds,DEC);  
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
  EEPROM.writeInt(WILL_TEMP_EEPROM_ADDRESS, will_temp);
  EEPROM.writeInt(ENCODER_EEPROM_ADDRESS, encoderPos);
  myEncoder.setPosition(encoderPos);
  memWrite = true;
  memNoWrite = false; 
  beepBuzzer(6250,80);
  while(digitalRead(EncoderDetent)==LOW);
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

















