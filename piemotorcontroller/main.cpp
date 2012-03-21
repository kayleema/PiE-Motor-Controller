/*
 * Berkeley Pioneers in Engineering
 * PiE Motor Controller Firmware
 * 
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Wire.h"

#define ENABLE_LEDS

//I2C bus address (hardcoded)
uint8_t I2C_ADDRESS = 0x0D;

/** Pin Definitions **/
//H-Bridge
const uint8_t IN1 =  PD4; //forward
const uint8_t IN2 =  PD5; //reverse (brakes if both IN1 and IN2 set)
const uint8_t D1  =  PD6; //disable (normally low)
const uint8_t D2  =  PD7; //disable (normally high)
const uint8_t FS  =  PB2; //fault status (currently not used)
const uint8_t FB  =  PC0; //feedback (also ADC0)
const uint8_t EN  =  PC1;
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
#define timeoutReg      (*((uint16_t*)(reg+0x80)))
#define stressReg       (*((uint8_t *)(reg+0xA0)))
#define nyanReg         (*((uint8_t *)(reg+0xA1)))

/////////// Motor Filter Registers ////////////

int16_t slewRateLimitedMotorV;
int16_t currentLimitedMotorV;
uint8_t finalMagnitude;
uint8_t finalDirection;

uint16_t timeSinceLastCommand = 0;  //Used for timeout in timer 2

///////////////////////////////////////////////

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
  
  //Set Feedback pin as input
  DDRC &= ~(1 << FB);

  // Set up motor pwm (Timer 0)
  // Set Timer 0 to phase-correct PWM, output on OC0A
  TCCR0A |= (1 << WGM00) | (1 << COM0A1);
  // Set clock source to F_CPU/64
  TCCR0B |= (1 << CS01)|(1 << CS00);
  //~3922kHz
}

void setupEncoder(){
  EICRA = 0x05;
  EIMSK = 0x03;
  //DDRD &= ~( (1 << ENCA) & (1 << ENCB) );
  sei();
}

//utility functions
int16_t max(int16_t a,int16_t b){
    if(a > b){
        return a;
    }
    else{
        return b;
    }
}
int16_t min(int16_t a,int16_t b){
    if(a > b){
        return b;
    }
    else{
        return a;
    }
}
int16_t abs(int16_t a){
    if(a < 0){
        return -a;
    }
    else{
        return a;
    }
}

int main(void)
{
	encoderCountReg = 5;
	feedbackReg = 5;
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
  
  //Setup timer 2
  // Set clock source to F_CPU/256
  TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20);
  //Enable Overflow interrupt
  TIMSK2 |= (1<<TOIE2);
  
  ///Default timeout is 100 iterations of timer2
  timeoutReg = 122;
}

//called continuously after startup
void loop(){
  //setMotorDir(directionReg);
  //setMotorPWM(pwmReg);
  setMotorDir(finalDirection);
  setMotorPWM(finalMagnitude);
  
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
  timeSinceLastCommand = 0;
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
    #ifdef ENABLE_LEDS
    PORTB &= ~(1<<LED_RED);
    PORTB |=  (1<<LED_GREEN);
    #endif
  }
  else if (dir == 0){
    //set direction backward
    PORTD &= ~(1<<IN1);
    PORTD |=  (1<<IN2);
    
    //set LED RED
    #ifdef ENABLE_LEDS
    PORTB |=  (1<<LED_RED);
    PORTB &= ~(1<<LED_GREEN);
    #endif
  }
  else if (dir == 2){
    //set braking
    PORTD |=  (1<<IN1);
    PORTD |=  (1<<IN2);
    
    //set LEDs OFF
    #ifdef ENABLE_LEDS
    PORTB &= ~(1<<LED_RED);
    PORTB &= ~(1<<LED_GREEN);
    #endif
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
  if( PIND & (1<<ENCA) ){
    if( PIND & (1<<ENCB) ){
      encoderCountReg--;
    }
    else{
      encoderCountReg++;
    }
  }
  else{
    if( PIND & (1<<ENCB) ){
      encoderCountReg++;
    }
    else{
      encoderCountReg--;
    }
  }
}

ISR(INT1_vect)
{
  if( PIND & (1<<ENCA) ){
    if( PIND & (1<<ENCB) )
      encoderCountReg++;
    else
      encoderCountReg--;
  }
  else{
    if( PIND & (1<<ENCB) )
      encoderCountReg--;
    else
      encoderCountReg++;
  }
}

uint16_t readFeedback(){
    uint8_t lowbyte, highbyte;
    
    PRR &= ~(1<<PRADC);    
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //enable adc1(ADCEN)
    ADCSRB = 0x00;//free running mode
    ADMUX = 1<<REFS0;//select pin 0 and set reference to AVcc
    ADCSRA |= 1<<ADSC;//start conversion
    while( ADCSRA & (1<<ADSC) ){
        //wait for ADC Result
    }
    lowbyte = ADCL;
    highbyte = ADCH & 0x03;
    return lowbyte | ((uint16_t)highbyte)<<8;
}

//Timer 2 Interrupt
uint16_t overspikes = 0;
uint16_t max_overspikes = 50;
uint16_t period = 250;

uint16_t feedback_limit = 200;

uint16_t iterations_this_period = 0;

uint8_t rampingDown = 0;
uint8_t rampingUp = 0;

uint8_t rampupLimit = 0;
uint8_t rampDownLimit = 255;

ISR(TIMER2_OVF_vect)
{
  if( directionReg==2 ){
    finalMagnitude = pwmReg;
    finalDirection = 2;
    return;
  }
  
  iterations_this_period++;
  
  ////////// Timeout
  
  timeSinceLastCommand++;
  if(timeSinceLastCommand > timeoutReg && timeoutReg != 0){
    pwmReg = 0;
  }
  
  ////////// Slew rate limit
  
  if(directionReg == 0){
    slewRateLimitedMotorV = -pwmReg;
  }
  else{
    slewRateLimitedMotorV = pwmReg;
  }
  
  
  ////////// Current Limit 
  
  //check if period is over
  if(iterations_this_period > period){
    iterations_this_period = 0;
    if(rampingDown){
       rampingDown = 0;
       rampupLimit = 128;
       rampingUp = 1;
    }
    else if(rampingUp){
       rampingUp = 0;
    }
    overspikes = 0;
  }
  
  //Check if we have drawn too much current yet
  if(overspikes > max_overspikes){
    //start rampdown
    rampingDown = 1;
    rampDownLimit = abs(currentLimitedMotorV);
    rampingUp = 0;
    overspikes = 0;
    iterations_this_period = 0;
  }
  
  //read current feedback analog pin
  feedbackReg=readFeedback();
  //check if it exceeds our limit
  if(feedbackReg > feedback_limit){
    overspikes++;
  }
  
  
  if( rampingDown ){
    if( rampDownLimit > 128 ){
      rampDownLimit--;
    }
    int16_t directionalRampDownLimit;
    if( directionReg == 1 ){
        directionalRampDownLimit = rampDownLimit;
        currentLimitedMotorV = min(directionalRampDownLimit, slewRateLimitedMotorV);
    }
    else{
        directionalRampDownLimit = -rampDownLimit;
        currentLimitedMotorV = max(directionalRampDownLimit, slewRateLimitedMotorV);
    }
  }
  else if( rampingUp ){
    if( rampupLimit < 255 ){
        rampupLimit++;
    }
    int16_t directionalRampupLimit;
    if( directionReg == 1 ){
        directionalRampupLimit = rampupLimit;
        currentLimitedMotorV = min(directionalRampupLimit, slewRateLimitedMotorV);
    }
    else{
        directionalRampupLimit = -rampupLimit;
        currentLimitedMotorV = max(directionalRampupLimit, slewRateLimitedMotorV);
    }
  }
  else{
    currentLimitedMotorV = slewRateLimitedMotorV;
  }
  reg[0xA1] = rampingUp;
  reg[0xA2] = rampingDown;
  reg[0xA3] = iterations_this_period;
  reg[0xA4] = rampDownLimit;
  
  finalMagnitude = abs(currentLimitedMotorV);
  finalDirection = currentLimitedMotorV > 0;
}

