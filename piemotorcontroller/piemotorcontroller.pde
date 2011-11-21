/*
 * Berkeley Pioneers in Engineering
 * PiE Motor Controller Firmware (Simple Version)
 * 
 * This is the simplest possible version of firmware
 * for the PiE motor controller.  This version of the 
 * firmware will only support setting the motor PWM 
 * frequency.  Later, will will develop a more 
 * sophisticated version.
 */

#include <Wire.h>

//whether to write debig messages to serial
#define DEBUG 1

//I2C bus address (hardcoded)
byte I2C_ADDRESS = 10;

//TODO: check these definitions

//H-Bridge Pin Definitions
int IN1 = 3; //forward
int IN2 = 5; //reverse (brakes if both IN1 and IN2 set)
int D1  = 6; //disable (normally low)
int D2  = 7; //disable (normally high)

int FS  = 10; //fault status

//LED Pin Definitions
int LED_RED = 8;
int LED_GREEN = 9;

void setup()
{
  //Setup I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  
  //Setup IO Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(D1 , OUTPUT);
  pinMode(D2 , OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  
  //Setup Serial Port 
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
}

void loop(){
  delay(100);
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
}

void receiveEvent(int count){
  char c;
  while(Wire.available() > 0){
    c = Wire.receive();
  }
  #ifdef DEBUG
    Serial.print("received: ");
    Serial.println(c);
  #endif
  setMotor(c);
}

void setMotor(char value){
  if(value >= 0){
    //set direction forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN1, LOW);
    //set LED Green
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }
  else{
    //set direction backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN1, HIGH);
    //set LED RED
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    //make value positive to get ready for PWM
    value = -value;
  }
  
  //scale to PWM Freq
  byte pwm = (byte)(value) * 2;
  #ifdef DEBUG
    Serial.print("pwm: ");
    Serial.println(pwm);
  #endif
  
  //set pwm
  analogWrite(D1, pwm);
}
