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
#include <util/delay.h>
#include "Wire.h"

//I2C bus address (hardcoded)
uint8_t I2C_ADDRESS = 0x0B;

/** Pin Definitions **/
//H-Bridge
const uint8_t IN1 =  PD4; //forward
const uint8_t IN2 =  PD5; //reverse (brakes if both IN1 and IN2 set)
const uint8_t D1  =  PD6; //disable (normally low)
const uint8_t D2  =  PD7; //disable (normally high)
const uint8_t FS  =  PB2; //fault status (currently not used)
const uint8_t FB  =  PC0; //feedback (also ADC0)
const uint8_t EN  = PC1;
//Encoders
const uint8_t ENCA = PD2;
const uint8_t ENCB = PD3;
//LEDs
const uint8_t LED_RED   = PB0;
const uint8_t LED_GREEN = PB1;

//buffer size
const uint8_t BUFFER_SIZE = 255;
//Buffer
uint8_t reg[BUFFER_SIZE];
//current buffer address pointer
uint8_t addr = 0;

//////////// LED DEFINITIONS /////////////
void setupLEDs(){
	DDRB |= _BV(1) | _BV(0); //set pins as output for LEDs
}

/////////// Register Definitions ///////////
#define directionReg    (*((uint8_t *)(reg+0x01)))
#define pwmReg          (*((uint8_t *)(reg+0x02)))
#define feedbackReg     (*((uint16_t*)(reg+0x10)))
#define encoderCountReg (*((int32_t *)(reg+0x20)))
#define stressReg       (*((uint8_t *)(reg+0xA0)))
#define nyanReg         (*((uint8_t *)(reg+0xA1)))

void setup();
void loop();
void receiveEvent(int count);
void requestEvent();
void setMotorDir(uint8_t dir);
void setMotorPWM(uint8_t value);
uint16_t readFeedback();

void motorSetup()
{
  // Set I/O Pins

  // Set IN1, IN2, D1, D2 as outputs
  DDRD |= (1 << IN1) | (1 << IN2) | (1 << D1) | (1 << D2);
  // Set EN as output
  DDRC |= (1 << EN);
  
  //Set EN
  PORTC |= (1 << EN);
  //Set D2
  PORTD |= (1 << D2);

  // Set FS pin as input, pull up enabled
  DDRB &= ~(1 << FS);
  PORTB |= (1 << FS);

  // Set up motor pwm (Timer 0)
  // Set Timer 0 to phase-correct PWM, output on OC0A
  TCCR0A |= (1 << WGM00) | (1 << COM0A1);
  // Set clock source to F_CPU/8
  TCCR0B |= (1 << CS01);
}

void setupEncoder(){
  EICRA = 0x0A;
  EIMSK = 0x03;
  //fix this!!!!!!!!
  sei();
}

int main(void)
{
	encoderCountReg = 5;
	sei();
	setup();
	for(;;){
 		loop();
	}
}

//called on startup
void setup()
{
  //Setup I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  //Setup Digital IO Pin directions
  motorSetup();
  
  setupLEDs();
  
  setupEncoder();
}

//called continuously after startup
void loop(){
  setMotorDir(directionReg);
  setMotorPWM(pwmReg);
  
  //TODO: Uncomment Me!
  //feedbackReg=readFeedback();
  
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
    //write to register
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
void setMotorDir(uint8_t dir){
  //set direction
  if (dir == 1){
    //set direction forward
    PORTD |=  (1<<IN1);
    PORTD &= ~(1<<IN2);
    
    //set LED Green
    PORTB &= ~(1<<LED_RED);
    PORTB |=  (1<<LED_GREEN);
  }
  else if (dir == 0){
    //set direction backward
    PORTD &= ~(1<<IN1);
    PORTD |=  (1<<IN2);
    
    //set LED RED
    PORTB |=  (1<<LED_RED);
    PORTB &= ~(1<<LED_GREEN);
  }
  else if (dir == 2){
    //set braking
    PORTD |=  (1<<IN1);
    PORTD |=  (1<<IN2);
    
    //set LEDs OFF
    PORTB &= ~(1<<LED_RED);
    PORTB &= ~(1<<LED_GREEN);
  }
}

//Set motor PWM value (between 0-255)
void setMotorPWM(uint8_t value){
  //set pwm
  OCR0A = 255-value;
}

//Encoder A Interrupt
ISR(INT0_vect)
{
  if( PORTD & (1<<ENCA) ){
    if( PORTD & (1<<ENCB) )
      encoderCountReg--;
    else
      encoderCountReg++;
  }
  else{
    if( PORTD & (1<<ENCB) )
      encoderCountReg++;
    else
      encoderCountReg--;
  }
}

ISR(INT1_vect)
{
  if( PORTD & (1<<ENCA) ){
    if( PORTD & (1<<ENCB) )
      encoderCountReg++;
    else
      encoderCountReg--;
  }
  else{
    if( PORTD & (1<<ENCB) )
      encoderCountReg--;
    else
      encoderCountReg++;
  }
}

uint16_t readFeedback(){
    ADMUX = 0xC0;
    ADCSRA = 0xC0;
    while( ~(ADCSRA & (1<<ADIF)) ){
        //wait for ADC Result
    }
    return (uint16_t)(ADCH<<8) & ADCL;
}