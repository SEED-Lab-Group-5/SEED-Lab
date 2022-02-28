//////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     David Long
// CLASS:    EENG-350
// TITLE:    Communication Exercise 2and3 - Arduino
//
// FUNCTION: Receives a number and a register offset from a Raspberry Pi. The number is 
//           stored to a different variable depending on the offset. The Arduino then waits 
//           for a read request from the Pi containing the offset to read from. The number 
//           corresponding to that offset is then sent to the Pi.
//
// HARDWARE: Pin Connection Guide
//           Raspberry Pi | Arduino
//           GPI02 (SDA) -> A4
//           GPIO3 (SCL) -> A5
//           GND         -> GND
//
// Execute:  Follow the connection guide above to link the Raspberry Pi and Arduino together
//           for I2C. Next upload this program to the Arduino. Finally, run the
//           "Communication Exercise 2 - Pi.py" or 
//           "Communication Exercise 3 - I2C and LCD Display - Pi.py" Python program on the 
//           Raspberry Pi and follow the on-screen prompts in the Python Shell.
//////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

int data[32] = {0};

// Registers 0 and 1 contain a 0 by default
int reg0Val = 0;
int reg1Val = 0;
int regOffset = 0;

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
        data[i] = Wire.read();
        i++;
    }

    // If more than 1 byte was received, the Pi is attempting to perform a write
    // i.e. register values must be modified
    if (i > 1) {
        regOffset = data[0];
        int receivedVal = data[1];
    
        // If the received offset is 0, store the received value to register 0
        if(regOffset == 0) {
            reg0Val = receivedVal;
            Serial.print("New reg0Val: ");
            Serial.println(reg0Val);
            Serial.print("Old reg1Val: ");
            Serial.println(reg1Val);
        }
        // If the received offset is 1, store the received value to register 1
        else if(regOffset == 1) {
            reg1Val = receivedVal;
            Serial.print("Old reg0Val: ");
            Serial.println(reg0Val);
            Serial.print("New reg1Val: ");
            Serial.println(reg1Val);
        }      
    }
    // If only one byte was read, the Pi is attempting to perform a read 
    // i.e. don't modify register values
    else if (i == 1) {
        regOffset = data[0];
    }
}

// callback for sending data
void sendData() {
    // If the received offset is 0, send the value in register 0 to the Pi
    if(regOffset == 0) {
        Wire.write(reg0Val);
        Serial.println("Sent reg0Val to Pi");
    }
    // If the received offset is 1, send the value in register 1 to the Pi
    else if(regOffset == 1) {
        Wire.write(reg1Val);
        Serial.println("Sent reg1Val to Pi");
    }
    Serial.println(' ');
}
