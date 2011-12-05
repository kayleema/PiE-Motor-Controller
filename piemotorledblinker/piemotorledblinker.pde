/*
 * Berkeley Pioneers in Engineering
 * PiE Motor Controller LED Blinker
 *
 * Blinks the LEDs on the motor controller to make 
 * sure the microcontroller is functioning.
 */
 
//LED Pin Definitions
int LED_RED = 8;
int LED_GREEN = 9;

void setup(){
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
}

void loop(){
  delay(200);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  delay(200);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
}
