/*
 * Berkeley Pioneers in Engineering
 * PiE SPI Tester
 * 
 * Sends out SPI data according to instructions from 
 * the serial port--opens an interactive terminal on 
 * the serial port and displays instructions for use.
 *
 * Also displays any data received from the attached 
 * device.  
 *
 * Press 'd' to set SS high (disable) and 'e' to set
 * it low (enable). (SS *is* inverted)
 *
 * On a unix system with pyserial installed, the following might be useful: 
 * For Arduino Uno:  
 *    python -m serial.tools.miniterm -p "/dev/tty.usbmodem411" -e
 * For Arduino Duemilanove:  
 *    python -m serial.tools.miniterm -p "/dev/tty.usbserial-A900cehS" -e
 */

#include <SPI.h>

//See http://arduino.cc/en/Reference/SPI for more pin assignments
int SS_PIN = 10;

void setup(void){
  Serial.begin(9600);
  
  Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  Serial.println("===========================================================");
  Serial.println("Welcome to PiE SPI Tester");
  Serial.println("Enter the data you would like to send as a decimal number, ");
  Serial.println("then press return. Press 'd' to set SS high (disable) and  ");
  Serial.println("'e' to set it low (enable). (SS *is* inverted)");
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(LSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
}

char inbuffer[10];
int pos;
byte outval;
byte response;
void loop(void){
  //prompt for data
  Serial.print("\n>>> ");
  
  //capture line
  pos = 0;
  while(1){
    while(Serial.available() <= 0){
      //wait
    }
    int c = Serial.read();
    if(c == '\n'){
      inbuffer[pos++] = '\0';
      break;
    }
    else if(c == 'd'){
      digitalWrite(SS_PIN, HIGH);
      Serial.println("\nSet SS HIGH");
      return;
    }
    else if(c == 'e'){
      digitalWrite(SS_PIN, LOW);
      Serial.println("\nSet SS LOW");
      return;
    }
    else{
      inbuffer[pos++] = char(c);
    }
  }
  
  //parse line
  if(pos == 0){
    Serial.println("Error: Encountered blank line");
    return;
  }
  else{
    outval = atoi(inbuffer);
  }
  
  //send data
  response = SPI.transfer(outval);
  
  //print result
  Serial.print("    Sent: ");
  Serial.println(int(outval));
  Serial.print("Response: ");
  Serial.println(int(response));
}

