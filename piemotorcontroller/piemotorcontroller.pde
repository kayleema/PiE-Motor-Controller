/*
 * Berkeley Pioneers in Engineering
 * PiE Motor Controller Firmware (Simple Version)
 * 
 * This is the simplest possible version of firmware
 * for the PiE motor controller.  This version of the 
 * firmware will only support setting the motor PWM 
 * frequency and the direction.
 */

#include <Wire.h>

//whether to print debug messages to serial
#define DEBUG 1

//I2C bus address (hardcoded)
byte I2C_ADDRESS = 0x0C;

//VERSION
byte VERSION = 0x01;

//H-Bridge Pin Definitions
const int IN1 =  3; //forward
const int IN2 =  5; //reverse (brakes if both IN1 and IN2 set)
const int D1  =  6; //disable (normally low)
const int D2  =  7; //disable (normally high)
const int FS  = 10; //fault status (currently not used)
const int FB  = A0; //feedback
const int EN  = A1; 

//Encoder Pin Definitions 
//TODO: check these
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
const int REG_FB  = 0x10;
const int REG_ENC_CNT = 0x11;
const int REG_ENC_RES = 0x11;

const int REG_VER = 0x51;
const int REG_DATE = 0x52;
const int REG_TIME = 0x5B;

//called on startup
void setup()
{
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
  
  //Encoder IRQ
  attachInterrupt(0, encoderChangeA, CHANGE);
  attachInterrupt(1, encoderChangeB, CHANGE);
  
  //Setup Serial Port 
  #ifdef DEBUG
    Serial.begin(9600);
    delay(30);
  #endif
}

//called continuously after startup
void loop(){
  setMotorDir(reg[REG_DIR]);
  setMotorPWM(reg[REG_PWM]);
  (*((word*)(reg + REG_FB))) = analogRead(FB);
  if(reg[REG_ENC_RES]!=0){
    reg[REG_ENC_RES] = 0;
    reg[REG_ENC_CNT] = 0;
  }
  //write debug data
  #ifdef DEBUG
    Serial.print(int(reg[REG_DIR]));
    Serial.print(" ");
    Serial.print(int(reg[REG_PWM]));
    Serial.print(" ");
    Serial.println(addr);
  #endif
}

//called when I2C data is received
void receiveEvent(int count){
  //set address
  addr = Wire.receive();
  //read data
  while(Wire.available()){
    if(addr >= BUFFER_SIZE){
      error("addr out of range");
    }
    //write to registers
    reg[addr++] = Wire.receive();
  }
}

//called when I2C data is reqested
void requestEvent()
{
  Wire.send(reg[addr++]);
}

//set motor direction
//Params:
//   byte dir:  1=fwd, 0=rev, 2=break
void setMotorDir(byte dir){
  //set direction
  if (dir == 1){
    //set direction forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    //set LED Green
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }
  else if (dir == 0){
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
  else if (dir == 3){
    error("break/rev not implemented");
  }
  else if (dir == 4){
    error("break/fwd not implemented");
  }
  else{
    error("Unrecognized direction");
  }
}

//Set motor PWM value (between 0-255)
void setMotorPWM(byte value){
  //set pwm
  analogWrite(D1, 255 - value);
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

//Encoder Interrupts
boolean enca = 0;
boolean encb = 0;
//PIN2: ENCA
void encoderChangeA(){
  enca = digitalRead(ENCA);
  if(enca == encb){//rising
    (*((long*)(&reg[REG_ENC_CNT])))++;
  }
  else{//falling
    (*((long*)(&reg[REG_ENC_CNT])))--;
  }
}

//PIN3: ENCB
void encoderChangeB(){
  if(enca != encb){//rising
    (*((long*)(&reg[REG_ENC_CNT])))++;
  }
  else{//falling
    (*((long*)(&reg[REG_ENC_CNT])))--;
  }
}
