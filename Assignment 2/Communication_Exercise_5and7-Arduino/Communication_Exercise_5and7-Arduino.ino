////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     David Long
// CLASS:    EENG-350
// TITLE:    Communication Exercise 5and7 - Arduino
//
// FUNCTION: Receives a block of data from the Raspberry Pi in the form of an array. This 
//           data is then reversed and sent back to the Pi.
//
// HARDWARE: Pin Connection Guide
//           Raspberry Pi | Arduino
//           GPI02 (SDA) -> A4
//           GPIO3 (SCL) -> A5
//           GND         -> GND
//
// EXECUTE:  Follow the connection guide above to link the Raspberry Pi and Arduino together
//           for I2C. Next upload this program to the Arduino. Finally, run the
//           "Communication Exercise 5 - I2C String Transfer - Pi.py" or
//           "Communication Exercise 7 - Exception Handling - Pi.py" Python program on the 
//           Raspberry Pi and follow the on-screen prompts in the Python Shell.
////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

byte receivedData[32] = {0};
byte reversedData[32] = {0};
int reversedArraySize = 0;

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
void receiveData(int byteCount) {
    int i = 0; 
    
    // While there are still bytes to read, read them and add to the data array. Track # of bytes
    while(Wire.available()) {
        receivedData[i] = Wire.read();
        i++;       
    }
    // If more than 1 byte was received, the Pi is attempting to perform a write
    // Nothing needs to be done if 1 or fewer bytes were received
    if (i > 1) {
        // Reverse the order of the receivedData list and remove the last element (register offset)
        int newIndex = 0;
        reversedArraySize = i - 1;
        for(int index = reversedArraySize ; index > 0 ; index--) {
            reversedData[newIndex] = receivedData[index];
            newIndex++;        
        }    
    }
}

// callback for sending data
void sendData() {
    Wire.write(reversedData, reversedArraySize);
}
