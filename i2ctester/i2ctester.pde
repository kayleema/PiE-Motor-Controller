/*
 * Berkeley Pioneers in Engineering
 * PiE Motor Controller I2C Tester
 * 
 * Sends out I2C data according to instructions from 
 * the serial port--opens an interactive terminal on 
 * the serial port and displays instructions for use.
 * Note that pressing 'r' will request a single byte
 * from the I2C device and pressing 'c' will clear 
 * the transmit buffer.
 *
 * The address of this device is hardcoded  using the
 * variable "MASTER_ADDRESS" below. Upon startup, the
 * user is prompted for the address of the device to 
 * talk to.
 *
 * On a unix system with pyserial installed, the following might be useful: 
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
  Serial.println("Please Enter the device address with which to communicate.");
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
  Serial.println("\nThank You.  To send data, enter a series of numbers in decimal, each ");
  Serial.println("one followed by hitting return.  When ready to transmit the sequence,  ");
  Serial.println("simply hit return again to send a blank line and the data will be sent.");
  Serial.println("Pressing 'r' at any time will request a single byte from the I2C device");
  Serial.println("and pressing 'c' will clear the transmit buffer.\n");
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
        in[inLength] = '\0';
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
      else if(mybyte == 127){
        Serial.print("\b \b");
        if(inLength > 0){
          inLength--;
        }
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
      long num = atoi((const char *)in);
      if(num > 255 || num < 0){
        Serial.println("Too big.  Try Again.");
      }
      else{
        Serial.print("Added to transaction:  ");
        Serial.println(int(num));
        transaction[transactionLength++] = byte(num);
      }
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
