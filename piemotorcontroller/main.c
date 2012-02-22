/*
 * Berkeley Pioneers in Engineering
 * PiE Motor Controller Firmware (Simple Version)
 * 
 * This is the simplest possible version of firmware
 * for the PiE motor controller.  This version of the 
 * firmware will only support setting the motor PWM 
 * frequency and the direction.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

//I2C bus address (hardcoded)
uint8_t I2C_ADDRESS = 0x0B;

//H-Bridge Pin Definitions
const uint8_t IN1 =  PD4; //forward
const uint8_t IN2 =  PD5; //reverse (brakes if both IN1 and IN2 set)
const uint8_t D1  =  PD6; //disable (normally low)
const uint8_t D2  =  PD7; //disable (normally high)
const uint8_t FS  = PB2; //fault status (currently not used)
const uint8_t FB  = PC0; //feedback (also ADC0)

const uint8_t EN  = PC1;

//Encoders
const uint8_t ENCA = PD2;
const uint8_t ENCB = PD3;

//LED Pin Definitions
const uint8_t LED_RED   = PB0;
const uint8_t LED_GREEN = PB1;

//buffer size
const uint8_t BUFFER_SIZE = 256;
//Buffer
uint8_t reg[BUFFER_SIZE];
//current buffer address pointer
uint8_t addr = 0;

//////////// LED DEFINITIONS /////////////
void setupLEDs(){
	DDRB |= _BV(1) | _BV(0); //set pins as output for LEDs
}
void setGreenLED()  {PORTB |=  _BV(PORTB1);}
void setRedLED()    {PORTB |=  _BV(PORTB0);}
void clrGreenLED()  {PORTB &= ~_BV(PORTB1);}
void clrRedLED()    {PORTB &= ~_BV(PORTB0);}

/////////// Register Definitions ///////////
#define directionReg    (*((uint8_t*)(reg+0x01)))
#define pwmReg          (*((uint8_t*)(reg+0x02)))
#define feedbackReg     (*((int* )(reg+0x10)))
#define encoderCountReg (*((long*)(reg+0x20)))
#define currentLowpass  (*((long*)(reg+0x80)))
#define stressReg       (*((byte*)(reg+0xA0)))
#define nyanReg         (*((byte*)(reg+0xA1)))


void motorSetup()
{
  // Set I/O Pins

  // Set IN1, IN2, D1, D2 as outputs
  DDRD |= (1 << IN1) | (1 << IN2) | (1 << D1) | (1 << D2);
  // Set EN as output
  DDRC |= (1 << EN);

  // Set FS pin as input, pull up enabled
  DDRB &= ~(1 << FS);
  PORTB |= (1 << FS);
}

//called on startup
void setup()
{
  //Setup I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  //Setup Digital IO Pin directions
  
  setupLEDs();
  
  //attachInterrupt(0, encoderA, CHANGE);
  //attachInterrupt(1, encoderB, CHANGE);
}

//called continuously after startup
void loop(){
  setMotorDir(directionReg);
  setMotorPWM(pwmReg);
  
  //feedbackReg=analogRead(FB);
  
  while(stressReg){
  	setMotorDir(1);
  	setMotorPWM(pwmReg);
  	_delay_ms(stressReg*10);
  	setMotorDir(0);
  	setMotorPWM(pwmReg);
  	_delay_ms(stressReg*10);
  }
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

