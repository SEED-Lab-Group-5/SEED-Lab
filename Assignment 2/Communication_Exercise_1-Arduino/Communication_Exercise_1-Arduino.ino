//////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     David Long
// CLASS:    EENG-350
// TITLE:    Communication Exercise 1 - Arduino
//
// FUNCTION: Receives an integer value from a Raspberry Pi over I2C and adds 5 to it. The
//           sum is sent back to the Pi.
//
// HARDWARE: Pin Connection Guide
//           Raspberry Pi | Arduino
//           GPI02 (SDA) -> A4
//           GPIO3 (SCL) -> A5
//           GND         -> GND
//
// EXECUTE:  Follow the connection guide above to link the Raspberry Pi and Arduino together
//           for I2C. Next upload this program to the Arduino. Finally, run the
//           "Communication Exercise 1 - I2C Communication - Pi.py" Python program on the 
//           Raspberry Pi and follow the on-screen prompts in the Python Shell.
//////////////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);  
  Serial.println("Ready!");
}

void loop() {
  delay(100);
}

// callback for received data
void receiveData(int byteCount){

  Serial.print("data received: ");
  
  // While there are still bytes to read, read them and store the most recent to number
  while(Wire.available()) {
    number = Wire.read();
    number = number + 5;    
    Serial.println(number);
    Serial.println(' ');
  }
  
  Serial.println(' ');
}

// callback for sending data
void sendData(){
  Wire.write(number);
}
