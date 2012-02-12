/*
 * Berkeley Pioneers in Engineering
 * PiE Motor Controller Firmware (Simple Version)
 * 
 * This is the simplest possible version of firmware
 * for the PiE motor controller.  This version of the 
 * firmware will only support setting the motor PWM 
 * frequency and the direction.
 */

#include "WProgram.h"
#include <Wire.h>

//whether to print debug messages to serial
#define DEBUG 0
#define STRESS 1

//I2C bus address (hardcoded)
byte I2C_ADDRESS = 0x0A;

//H-Bridge Pin Definitions
const int IN1 =  4; //forward
const int IN2 =  5; //reverse (brakes if both IN1 and IN2 set)
const int D1  =  6; //disable (normally low)
const int D2  =  7; //disable (normally high)
const int FS  = 10; //fault status (currently not used)
const int FB  = A0; //feedback (currently not used)

const int EN  = A1; 

//Encoders
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

//buffer Registers
#define directionReg    (*((byte*)(reg+0x01)))
#define pwmReg          (*((byte*)(reg+0x02)))
#define feedbackReg     (*((int* )(reg+0x10)))
#define encoderCountReg (*((long*)(reg+0x20)))
#define currentLowpass  (*((long*)(reg+0x80)))
#define stressReg       (*((byte*)(reg+0xA0)))
#define nyanReg         (*((byte*)(reg+0xA1)))

#include "nyan.h"

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
  
  attachInterrupt(0, encoderA, CHANGE);
  attachInterrupt(1, encoderB, CHANGE);
  
  //Setup Serial Port 
  #ifdef DEBUG
    Serial.begin(9600);
    delay(30);
  #endif
}

//called continuously after startup
void loop(){
  setMotorDir(directionReg);
  setMotorPWM(pwmReg);
  
  feedbackReg=analogRead(FB);
  
  //write debug data
  #ifdef DEBUG
    Serial.print(int(directionReg));
    Serial.print(" ");
    Serial.print(int(pwmReg));
    Serial.print(" ");
    Serial.println(addr);
  #endif
  
  #ifdef STRESS
  if(stressReg){
    if((millis() % (stressReg*10)) < (stressReg*10) / 2){
      directionReg= 1;
    }
    else{
      directionReg = 0;
    }
  }
  if(nyanReg){
    playme(D1);
  }
  else{
  }
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
byte pdir = 0;
void setMotorDir(byte dir){
  if(dir != pdir && pwmReg == 255){
    //set braking
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    delay(250);
  }
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
    digitalWrite(IN2, HIGH);
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
  pdir = dir;
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
  delay(250);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  delay(250);
  //write debug data
  #ifdef DEBUG
    Serial.print("ERROR:  ");
    Serial.println(message);
  #endif
}

void encoderA(){
  if(digitalRead(ENCA)){
    if(digitalRead(ENCB))
      encoderCountReg--;
    else
      encoderCountReg++;
  }
  else{
    if(digitalRead(ENCB))
      encoderCountReg++;
    else
      encoderCountReg--;
  }
}

void encoderB(){
  if(digitalRead(ENCA)){
    if(digitalRead(ENCB))
      encoderCountReg++;
    else
      encoderCountReg--;
  }
  else{
    if(digitalRead(ENCB))
      encoderCountReg--;
    else
      encoderCountReg++;
  }
}

