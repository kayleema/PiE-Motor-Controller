#include <SPI.h>

void setup(void){
  Serial.begin(9600);
  
  Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  Serial.println("===========================================================");
  Serial.println("Welcome to PiE SPI Tester");
  Serial.println("Enter the data you would like to send as a decimal number,");
  Serial.println("then press return.");
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(LSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
}

char inbuffer[10];
int pos;
byte outval;
byte response;
void loop(void){
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
  Serial.print("    Sent: ");
  Serial.println(int(outval));
  Serial.print("Response: ");
  Serial.println(int(response));
}

