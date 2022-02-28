/////////////////////////////////////////////////////////////////////////////////////////////////
// NAME:     David Long
// CLASS:    EENG-350
// TITLE:    Communication Exercise 8 - Arduino
//
// FUNCTION: Receives an integer value from a Raspberry Pi over serial and adds 5 to it. The
//           sum is sent back to the Pi.
//
// HARDWARE: Connect the Arduino to the Raspberry Pi using a USB-A to USB-B cable
//
// EXECUTE:  Follow the connection guide above to link the Raspberry Pi and Arduino together
//           for serial. Ensure the Arduino serial monitor is not open. Next upload this 
//           program to the Arduino. Finally, run the "Communication Exercise 8 - Serial - Pi.py" 
//           Python program on the Raspberry Pi and follow the on-screen prompts in the Python 
//           Shell.
/////////////////////////////////////////////////////////////////////////////////////////////////
String strValue;
bool dataRead;

void setup() {
    Serial.begin(115200);
}

void loop() {
    if (dataRead) {
        int intValue = strValue.toInt();    // Convert the received string to an integer
        Serial.print("You sent me: ");
        Serial.print(strValue);             
        intValue += 5;                      // Add 5 to the number
        strValue = String(intValue);        // Convert value back to string
        Serial.print("\nThe received value + 5 = ");      
        Serial.println(strValue);           // Send modified value over serial
        dataRead = false;                   // Clear DataRead flag
    }
}

// The Pi is attempting to write to the Arduino over serial
void serialEvent(){    
    
    if (Serial.available() > 0) {
        strValue = Serial.readStringUntil('\n');
        dataRead = true;    // Flag to indicate if a serial read operation has occured
    }
    // Wait until buffer is empty
    Serial.flush();
}
