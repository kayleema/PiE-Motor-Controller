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
byte I2C_ADDRESS = 10;

//H-Bridge Pin Definitions
int IN1 =  3; //forward
int IN2 =  5; //reverse (brakes if both IN1 and IN2 set)
int D1  =  6; //disable (normally low)
int D2  =  7; //disable (normally high)
int FS  = 10; //fault status (currently not used)
int FB  =  0; //feedback (currently not used)

//LED Pin Definitions
int LED_RED   = 8;
int LED_GREEN = 9;

void setup()
{
  //Setup I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  
  //Setup Digital IO Pins
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
  //nothing to do here--everything triggered by interrupts
  delay(100);
}

void receiveEvent(int count){
  //read the instruction
  byte proc = Wire.receive();
  //Handle setMotor Instruction
  if(proc == 0x01){
    //Motor instruction must take two inputs
    if(count != 3){
      //set both LEDs for error indication
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      #ifdef DEBUG
        Serial.print("ERROR: setMotor takes two input bytes\n   ->");
        Serial.print(count);
        Serial.println(" given")
      #endif
      return;
    }
    else{
      //read inputs
      byte dir = Wire.receive();
      byte value = Wire.receive();
      #ifdef DEBUG
        if(dir){
          Serial.print("setMotor: F");
        }
        else{
          Serial.print("setMotor: R");
        }
        Serial.println(value);
      #endif
      //set motor speed/direction
      setMotor(dir, value);
    }
  }
}

//sets the speed/direction of the motor
//byte dir is either 0=rev or 1=fwd
//byte value is the pwm value (0-255)
void setMotor(byte dir, byte value){
  //set direction
  if(dir == 1){
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
  }
  
  //set pwm
  analogWrite(D1, value);
}
