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
#define VERSION "v1.5"		
#define INTRO

#define LCD_I2C_ADDRESS      0x27
#define ROWS 4
#define COLUMNS 20

#define ADC_TO_TEMP_GAIN 	1.8//0.99//2.50//0.53 //0.415
#define ADC_TO_TEMP_OFFSET      25.0
#define STANDBY_TEMP		175
#define MAX_TEMP		450
#define MIN_TEMP	        25 // Minimum setpoint temperature
#define MAX_PWM_LOW		180//180
#define MAX_PWM_HI		240//210
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

#define SIZE_BAR (10 * 5)
//=======================================================
int pwm = 0; //pwm Out Val 0.. 255
unsigned int actual_temperature, will_temp = STANDBY_TEMP;
int MAX_PWM;
boolean standby_act = false;
int tempDIV;
float encoderValue = 0;
volatile float encoderPos = 0;
volatile float encoderPosTemp = 0;
boolean memWrite = false;
boolean memNoWrite = true;
boolean state = false;
boolean heater = false;
boolean unplug = 0;
int t1,t2;
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
//=======================================================
Encoder1 myEncoder = Encoder1(Encoder1ChnA,Encoder1ChnB,EncoderDetent);
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
StopWatch sw_millis;    
StopWatch sw_secs(StopWatch::SECONDS);
//LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 5, 6, 7, 2, 3, 4, 1, 0, NEGATIVE);  // Set the LCD I2C address
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
void setup() 
{
  Wire.begin();
  lcd.begin(COLUMNS, ROWS); 
  lcd.setBacklight(HIGH);
  Serial.begin(9600);
  Serial.println("Sketch'es location:");
  Serial.println("C:\\Users\\Administrator\\Documents\\DXP\\AVR SOLDERING IRON\\FIRMWARE\\NANO\\IRON_NANO_1");

  byte newChar[8];
  int i; 

  for (i = 0; i < 8; i++)
    lcd.createChar(i, getChar(i, newChar));
  customChars();
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
  Timer1.attachInterrupt( timer1_ISR ); // attach the service routine here  

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
  sw_secs.start();
  sw_millis.start();

  will_temp = EEPROM.readInt(WILL_TEMP_EEPROM_ADDRESS);
  myEncoder.setPosition(will_temp); 
  if (will_temp == 4294967295) 
  {
    will_temp = STANDBY_TEMP;
  }
  encoderPos = EEPROM.readInt(ENCODER_EEPROM_ADDRESS);
  if (encoderPos == 4294967295) 
  {    
    myEncoder.setPosition(will_temp);
  }

  Input = getTemperature();
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
  static boolean oneTime = false; 

  if(standby_act == false)
  {
    Input = getTemperature();
    Setpoint = encoderPos;
    encoderPosTemp = encoderPos;
  }
  else
  {
    Input = getTemperature();
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
    lcd.setCursor(19,0);
    if(oneTime)
    { 
      will_temp = STANDBY_TEMP;
      lcd.print(" ");
      lcd.setCursor(19,1);
      lcd.print(" ");
      oneTime = false;
      lcd.setCursor(5,1);
      lcd.print("       ");      
    } 
    lcd.setCursor(19,0);    
    showIron(19,0);
  }  
  else
  {
    lcd.setCursor(19,0);
    if(!oneTime)
    {
      showIron(19,0);
      oneTime = true;
    }
    will_temp = STANDBY_TEMP;
    lcd.print(" ");
    lcd.setCursor(19,1);
    lcd.print(" ");
    lcd.setCursor(5,1);
    lcd.print("       ");     
  }

  if((memNoWrite == true) && (memWrite = false))
  {       
    memNoWrite = true; 
    memWrite = false; 
  }  
  else if((memNoWrite == false) && (memWrite = true))
  {
    lcd.setCursor(10,3);
    lcd.print("MEM=");   
    lcd.print(tempWill);
    lcd.write(byte(223)); 
    lcd.print("C ");
  } 
  memNoWrite = true; 
  memWrite = false; 

  int will_temp_tmp = will_temp;

  checkUnplugged();

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

  if ((standby_act && (will_temp >= STANDBY_TEMP))) 
  {
    will_temp = STANDBY_TEMP;  
  }  
  writeHEATING(will_temp, Input, pwm);
  showTime(13,0);
}  
//=======================================================================
void checkUnplugged()
{
  actual_temperature = getTemperature();
  if((actual_temperature >= MAX_TEMP) && (actual_temperature < MAX_TEMP+100))
  {    
    pwm = 0;
    digitalWrite(HEAT_LED, LOW);
    lcd.clear();
    do
    {      
      lcd.setCursor(5,1);
      lcd.print("UNPLUGGED!");
      actual_temperature = getTemperature();
    }
    while(actual_temperature >= MAX_TEMP);
    lcd.clear();
  }
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
  if( minutes > 59)
  {
    minutes = 0;
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
  myEncoder.lowLevelTick();
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
  if (digitalRead(STANDBYin) == LOW) standby_act = true;
  else standby_act = false;
  Timer1.attachInterrupt( timer1_ISR );
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
void splashScreen()
{
  lcd.setCursor(0,0);
  lcd.print(F("    ARDUINO NANO    "));
  lcd.setCursor(0,1);
  lcd.print(F(" SOLDERING STATION  "));
  lcd.setCursor(0,2);
  lcd.print(F("   MANOS MAR v1.0   "));
  lcd.setCursor(0,3);
  lcd.print(F("    96W/25-400"));
  lcd.write(byte(0));
  lcd.print(F("C    "));
  delay(5000);
  lcd.clear();
  //  showIron(0,0);
  //  delay(5000);
}  
//=======================================================================
void showIron(int line, int row)
{
  lcd.setCursor(line,row);
  lcd.write(byte(6));
  lcd.setCursor(line,row+1);
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
  analogWrite(PWMpin, pwm);	//switch heater back to last value
  return round(((float) adcValue)*ADC_TO_TEMP_GAIN+ADC_TO_TEMP_OFFSET); //apply linear conversion to actual temperature
}
//=======================================================================
void writeHEATING(int tempWILL, int tempVAL, int pwmVAL)
{
  static int d_tempWILL = 1;//2		
  static int tempWILL_OLD = 1;//10
  static int tempVAL_OLD = 1;//10
  static int pwmVAL_OLD	= 1;//10

  pwmVAL = map(pwmVAL, 0, 255, 0, 99);

  lcd.setCursor(0,0);
  lcd.print(F("A"));
  lcd.setCursor(0,1);
  lcd.print(F("S"));

  lcd.setCursor(0,2);
  lcd.print(F("ACT="));
  lcd.setCursor(0,3);
  lcd.print(F("SET="));
  lcd.setCursor(10,2);
  lcd.print(F("PWM="));
  lcd.setCursor(10,3);

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
    if (standby_act)
    {

    }
    lcd.print(tempVAL); 
    t1 = tempVAL;
    lcd.write(byte(223)); 
    lcd.print("C");
    tempVAL_OLD = tempVAL; 

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
      lcd.write(byte(223)); 
      lcd.print("C");
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
      lcd.print("%");
      pwmVAL_OLD = pwmVAL;
    }
    lcd.setCursor(1,0);
    print_histogram(t1,MAX_TEMP); 
    lcd.print(" ");
    lcd.setCursor(1,1);
    print_histogram(t2,MAX_TEMP); 
    lcd.print(" ");   
  }
}  
//=======================================================================
void showTempVAL()
{

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
}
//=======================================================================
/*
void print_histogram(float val, float maxVal) 
 {
 float prevValue;
 float lastFullChars;
 int temp;
 
 // -- calculate full (filled) character count
 float fullChars = (float)val * COLUMNS / maxVal;
 // -- calculate partial character bar count
 byte mod = (val * COLUMNS * 5 / maxVal) ;
 temp = mod % 5;
 mod = temp;
 
 // -- if value does not change, do not draw anything
 int normalizedValue = (int)fullChars * 5 + mod;
 if(prevValue != normalizedValue) 
 {
 
 // -- write filled characters
 for(byte i = 0; i < fullChars; i++) 
 {
 lcd.write(byte(5));
 }
 
 // -- write the partial character
 if(mod > 0) 
 {
 lcd.write(mod); // -- index the right partial character
 ++fullChars;
 }
 
 // -- clear characters left over the previous draw
 for(int i = fullChars; i < lastFullChars; i++) 
 {
 lcd.print("  ");
 }
 
 // -- save cache
 lastFullChars = fullChars;
 prevValue = normalizedValue;
 }
 }
 */
//=======================================================================
//=======================================================================
//=======================================================================
//=======================================================================
//=======================================================================







