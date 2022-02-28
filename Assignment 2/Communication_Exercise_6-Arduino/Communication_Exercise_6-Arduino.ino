////////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     David Long
// CLASS:    EENG-350
// TITLE:    Communication Exercise 6 - Arduino
//
// FUNCTION: The Arduino reads the voltage across an attached potentiometer and sends the bit
//           representation to the Raspberry Pi.
// 
// HARDWARE: Pin Connection Guide For I2C
//           Raspberry Pi | Arduino
//           GPI02 (SDA) -> A4
//           GPIO3 (SCL) -> A5
//           GND         -> GND
//
//           Potentiometer Setup (Connected to Arduino)
//           A0  -> Center Lead of Pot
//           GND -> Left Outer Lead of Pot
//           5V  -> Right Outer Lead of Pot
//
//           Connect LCD display to Raspberry Pi header pins
//           
// Execute:  Follow the connection guide above to link the Raspberry Pi and Arduino together
//           for I2C. Next upload this program to the Arduino. Finally, run the
//           "Communication Exercise 6 - Analog Voltage - Pi.py" Python program on the Raspberry 
//           Pi and follow the on-screen prompts in the Python Shell.
////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

void setup() {
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

// callback for received data (only used to receive register offset which is not needed in this code)
void receiveData(int byteCount) {
    Wire.read(); 
}

// callback for sending data
void sendData() {
    byte voltageBytes[4] = {0};
    
    // Read the input on analog pin 0 to get integer representation of voltage (value between 0-1023)
    int currentVoltage = analogRead(A0);
    Serial.println(currentVoltage);
    
    // How many times largest 1 byte decimal value fits in currentVoltage
    int numFullBytes = currentVoltage / 256;
    int partialByte = currentVoltage % 256;

    // Fill the first n elements of voltageBytes array with 255 where n is the number of times 256 fits
    // into the currentVoltage integer
    for (int i = 0 ; i < numFullBytes ; i++) {
        voltageBytes[i] = 255;
    }
    // Fill last byte with remainder
    voltageBytes[3] = partialByte;

    Serial.print("Byte1 = ");
    Serial.println(voltageBytes[0]);
    Serial.print("Byte2 = ");
    Serial.println(voltageBytes[1]);
    Serial.print("Byte3 = ");
    Serial.println(voltageBytes[2]);
    Serial.print("Byte4 = ");
    Serial.println(voltageBytes[3]);
    
    // Send 4 bytes making up current voltage to the Pi
    Wire.write(voltageBytes, 4); 
}
