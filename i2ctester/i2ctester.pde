/*
 * Berkeley Pioneers in Engineering
 * PiE Motor Controller I2C Tester
 * 
 * Sends out I2C data coming from the serial port.
 * Opens an interactive terminal on the serial port 
 * and displays instructions for use.
 *
 * The address of this device is hardcoded 
 * using the variable "MASTER_ADDRESS" below.
 * Upon startup, the user is prompted for the address
 * of the device to talk to
 *
 * On a unix system with pyserial installed, run: 
 * For Arduino Uno:  
 *    python -m serial.tools.miniterm -p "/dev/tty.usbmodem411" -e
 * For Arduino Duemilanove:  
 *    python -m serial.tools.miniterm -p "/dev/tty.usbserial-A900cehS" -e
 */

#include <Wire.h>
 
//address with which to talk to
byte I2C_ADDRESS;
//address of this device (hardcoded)
byte MASTER_ADDRESS = 13;

//buffer addresses
const int REG_DIR = 0x01;
const int REG_PWM = 0x02;
 
void setup()
{
  //Setup I2C
  Wire.begin(MASTER_ADDRESS);
  //Setup Serial
  Serial.begin(9600);
  //get I2C address
  Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
  Serial.println("\n\n\n===========================================================");
  Serial.println("Welcome to PiE I2C Tester");
  Serial.println("Please Enter the device address with which you would");
  Serial.println("like to communicate.");
  Serial.print("\nI2C Address:  ");
  byte tmp[10];
  int curr = 0;
  while(1){
    while(Serial.available() < 1){
      //wait
    }
    byte in = Serial.read();
    if(in == '\n'){
      tmp[curr] = '\0';
      break;
    }
    else{
      tmp[curr++] = in;
    }
  }
  I2C_ADDRESS = atoi((const char*)tmp);
  Serial.println(int(I2C_ADDRESS));
  Serial.println("\nThank You.  To send data, enter a series of numbers in decimal,");
  Serial.println("each one followed by a newline.  When ready to transmit the sequence, ");
  Serial.println("simply hit return again to send a blank line and the data will be sent.\n");
}

byte transaction[10];
int transactionLength = 0;
byte in[10];
int inLength = 0;
void loop(){
  //get transaction
  transactionLength = 0;
  boolean clearnow = 0;
  while(1){
    //get byte
    Serial.print(">>> ");
    inLength = 0;
    while (1){
      while (Serial.available() <= 0) {
        //wait for data
      }
      byte mybyte = Serial.read();
      if((mybyte == '\n') || (mybyte == '\r')){
        delay(100);
        Serial.flush();
        in[inLength+1] = '\0';
        break;
      }
      else if(mybyte == 'r'){
        Serial.println();
        Serial.print("Requesting...");
        Wire.requestFrom((int)I2C_ADDRESS, 1);
        Serial.println("Received");
        Serial.println(int(Wire.receive()));
        Serial.print(">>> ");
      }
      else if(mybyte == 'c'){
        clearnow = 1;
        inLength = 100;
        Serial.println();
        Serial.println("Cleared");
        break;
      }
      else{
        in[inLength++] = mybyte;
      }
    }
    //blank line indicates end of transaction
    if(inLength == 0){
      break;
    }
    //inLength == 0 indicated that we want to clear the transaction
    else if(clearnow){
      clearnow = 0;
      transactionLength = 0;
      Serial.println("Transaction buffer reset");
    }
    else{
      byte num = atoi((const char *)in);
      Serial.print("Added to transaction:  ");
      Serial.println(int(num));
      transaction[transactionLength++] = num;
    }
  }
  //debug message
  Serial.print("Sent ");
  Serial.print(transactionLength);
  Serial.print(" bytes:\n   [");
  for(int i = 0; i < transactionLength; i++){
    Serial.print(int(transaction[i]));
    if(i < (transactionLength-1))
      Serial.print(", ");
  }
  Serial.println("]");
  //send transaction
  Wire.beginTransmission(I2C_ADDRESS);
  for(int i = 0; i < transactionLength; i++){
    Wire.send(byte(transaction[i]));
  }
  Wire.endTransmission();
}
