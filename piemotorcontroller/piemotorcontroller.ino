/*
 * Berkeley Pioneers in Engineering
 * PiE Motor Controller Firmware
 * 
 * This is the firmware for the PiE motor controller. 
 */

#include <Wire.h>
#include <EEPROM.h>

//whether to print debug messages to serial
#define DEBUG 1

//I2C bus address default
//also used to store curent I2C address after startup
byte I2C_ADDRESS = 0x0C;

//How often the encder is sampled for differentiation and debug 
//msgs are printed in milliseconds
const int LIM_INT_PD = 10;

//used to store previous sample time
long pmillis = 0;

//previous encoder value for differentiation
long pencoder = 0;

//VERSION
const byte VERSION = 0x01;

//H-Bridge Pin Definitions
const int IN1 =  3; //forward
const int IN2 =  5; //reverse (brakes if both IN1 and IN2 set)
const int D1  =  6; //disable (normally low)
const int D2  =  7; //disable (normally high)
const int FS  = 10; //fault status (currently not used)
const int FB  = A0; //feedback
const int EN  = A1; 
//Pin to be pulsed : depends on the direction register
int PWMPIN = D1;

//Encoder Pin Definitions
const int ENCA = 2;
const int ENCB = 3;

//LED Pin Definitions
const int LED_RED   = 8;
const int LED_GREEN = 9;

//buffer size
const int BUFFER_SIZE = 256;
//Buffer
byte reg[BUFFER_SIZE];
//current buffer address pointer
int addr = 0;
//buffer addresses
const int REG_DIR = 0x01;
const int REG_PWM = 0x02;
const int REG_FB  = 0x10; //2-byte uint

const int REG_ENC_CNT  = 0x20; //4-byte int
const int REG_ENC_LOOP = 0x24; //4-byte int
const int REG_ENC_EN   = 0x28;
const int REG_ENC_VEL  = 0x28; //float

const int REG_PID_SET = 0x30; //float
const int REG_PID_P   = 0x34; //float
const int REG_PID_I   = 0x38; //float
const int REG_PID_D   = 0x3C; //float
const int REG_PID_EN  = 0x40;

const int REG_ADDR = 0x50;
const int REG_VER  = 0x51;
const int REG_DATE = 0x52;
const int REG_TIME = 0x5B;

//type expansion macros
#define expandToLong(A)  ( *( (long* )( &(A) ) ) )
#define expandToWord(A)  ( *( (word* )( &(A) ) ) )
#define expandToFloat(A) ( *( (float*)( &(A) ) ) )

//function prototypes
void setup();
void loop();
void receiveEvent();
void requestEvent();

//called on startup
void setup()
{
  //Get I2C Address
  byte eeread = EEPROM.read(0);  //read from EEPROM
  if(eeread == 255){  //will read as 255 if hasn't been set
    EEPROM.write(0, I2C_ADDRESS);  //set to default if not set
  }
  else{
    I2C_ADDRESS = eeread;  //set address to value from eeprom
  }
  
  //Setup I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  //Setup Digital IO Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(D1 , OUTPUT);
  pinMode(D2 , OUTPUT);
  
  pinMode(EN , OUTPUT);
  digitalWrite(EN, HIGH);
  digitalWrite(D1, LOW);
  digitalWrite(D2, HIGH);
  
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  
  //setup registers
  reg[REG_VER] = VERSION;
  memcpy((void*)__DATE__, (&reg[REG_DATE]), 11);
  memcpy((void*)__TIME__, (&reg[REG_TIME]), 8);
  //(most registers should remain at their default state of 0)
  
  enableEncoder();
  
  //Setup Serial Port 
  #ifdef DEBUG
    Serial.begin(9600);
    delay(30);
    Serial.println("Started");
  #endif
}

//called repeatedly after startup
void loop(){
  if(!reg[REG_PID_EN]){ //if not PID enabled, set motors normally
    //set motor direction and speed
    setMotorDir(reg[REG_DIR]);
    setMotorPWM(reg[REG_PWM]);
  }
  else{
    //this function will se the motors using the pid
    runPID();
  }
  
  //read current feedback value
  expandToLong(reg[REG_FB]) = analogRead(FB);
  
  //check if i2c address has changed
  if( reg[REG_ADDR] != I2C_ADDRESS ){
    I2C_ADDRESS = reg[REG_ADDR];  //update current address
    EEPROM.write(0, I2C_ADDRESS); //write change to EEPROM
  }
  
  //start of limited sample interval section (uses LIM_INT_PD constant)
  long cmillis = millis();
  if ( (cmillis - pmillis) >= LIM_INT_PD){
    pmillis = cmillis;
    //differentiate encoder
    long cencoder = reg[REG_ENC_CNT];
    int dencoder = cencoder - pencoder;
    expandToFloat(reg[REG_ENC_VEL]) = (float)(dencoder) / LIM_INT_PD * 1000.0;
    pencoder += LIM_INT_PD;
    
    //write debug data
    #ifdef DEBUG
      Serial.print(int(reg[REG_DIR]));
      Serial.print(" ");
      Serial.print(int(reg[REG_PWM]));
      Serial.print(" ");
      Serial.println(addr);
    #endif
  }
}

//called when I2C data is received
void receiveEvent(int count){
  //set address
  addr = Wire.read();
  //read data
  while(Wire.available()){
    if(addr >= BUFFER_SIZE){
      error("addr out of range");
    }
    //write to registers
    reg[addr++] = Wire.read();
  }
}

//called when I2C data is requested
void requestEvent()
{
  Wire.write(reg[addr++]);
}

//set motor direction
void setMotorDir(byte dir){
  //deactivate both disable pins
  digitalWrite(D1, LOW);
  digitalWrite(D2, HIGH);
  //set the direction pins and LED indicators
  if ((dir == 1) || (dir == 4)){
    //set direction forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    //set LED Green
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }
  else if ((dir == 0) || (dir == 3)){
    //set direction backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    //set LED RED
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
  }
  else if (dir == 2){
    //set braking
    digitalWrite(IN1, HIGH);
    digitalWrite(IN1, HIGH);
    //set LEDs OFF
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
  }
  else{
    error("Unrecognized direction");
  }
  //Set the pin we're pulsing
  if ((dir == 3) || (dir == 4)){
    PWMPIN = IN1;
  }
  else{
    PWMPIN = D2;
  }
}

//Set motor PWM value (between 0-255)
void setMotorPWM(byte value){
  //set pwm using PWMPIN
  analogWrite(PWMPIN, value);
}

//sets both LEDs on to indicate error state and delays for 500ms
//also writes the message to serial if debugging enabled
void error(char* message){
  //set both LEDs on
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  delay(500);
  //write debug data
  #ifdef DEBUG
    Serial.print("ERROR:  ");
    Serial.println(message);
  #endif
}

////////// ENCODER ///////////

//Encoder Interrupts
boolean enca = 0;
boolean encb = 0;
//PIN2: ENCA
void encoderChangeA(){
  enca = digitalRead(ENCA);
  if(enca == encb){//rising
    expandToLong(reg[REG_ENC_CNT])++;
  }
  else{//falling
    expandToLong(reg[REG_ENC_CNT])--;
  }
  encoderCheckBounds();
}

//PIN3: ENCB
void encoderChangeB(){
  if(enca != encb){//rising
    expandToLong(reg[REG_ENC_CNT])++;
  }
  else{//falling
    expandToLong(reg[REG_ENC_CNT])--;
  }
  encoderCheckBounds();
}

void encoderCheckBounds(){
  long &loopbackval = expandToLong(reg[REG_ENC_LOOP]);
  if ( loopbackval > 0 ){
    long &encoderval = expandToLong(reg[REG_ENC_CNT]);
    if ( encoderval >= loopbackval ){
      encoderval = 0;
    }
    else if ( encoderval < 0 ){
      encoderval = loopbackval;
    }
  }
}

void enableEncoder(){
  attachInterrupt(0, encoderChangeA, CHANGE);
  attachInterrupt(1, encoderChangeB, CHANGE);
}
void disableEncoder(){
  detachInterrupt(0);
  detachInterrupt(1);
}

/////////// PID ///////////

//accumulated integral term
float integral = 0;
//timing variable : time of last iteration
long pidPmillis = 0;

float runPID(){
  //timing
  long cmillis = millis();
  //get the error and the value we'll use for the d-term
  float error = -(float)expandToLong(reg[REG_PID_SET]);
  float dterm;
  switch (reg[REG_PID_EN]){
    case 1://current
      error += (float)expandToLong(reg[REG_FB]);
      dterm = 0.0;//TODO:  add it here
      break;
    case 2://encoder speed
      error += (float)expandToLong(reg[REG_ENC_VEL]);
      dterm = 0.0;//TODO:  add it here
      break;
    case 3://encoder position
      error += (float)expandToLong(reg[REG_ENC_CNT]);
      dterm = expandToFloat(reg[REG_ENC_VEL]);
      break;
  }
  //accumulate integral
  integral += error * (cmillis - pidPmillis) / 1000.0;
  //main PID calculation
  float result = expandToFloat(reg[REG_PID_P]) * error +
                 expandToFloat(reg[REG_PID_I]) * integral +
                 expandToFloat(reg[REG_PID_D]) * dterm;
  //timing
  pidPmillis = cmillis;
  //clip result
  return max(min(result, 1.0), -1.0);
}

