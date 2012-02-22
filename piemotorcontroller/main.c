#include <avr/io.h>
#include <util/delay.h>

//LED IO Manipulation
void setupLEDs(){
	DDRB |= _BV(1) | _BV(0); //set pins as output for LEDs
}
void setGreenLED()  {PORTB |=  _BV(PORTB1);}
void setRedLED()    {PORTB |=  _BV(PORTB0);}
void clrGreenLED()  {PORTB &= ~_BV(PORTB1);}
void clrRedLED()    {PORTB &= ~_BV(PORTB0);}


//Main Code Loop

int main(void)
{
	setupLEDs();
	for(;;){
 		_delay_ms(500.0);
  		setGreenLED();
  		clrRedLED();
  		_delay_ms(500.0);
  		clrGreenLED();
  		setRedLED();
	}
}
