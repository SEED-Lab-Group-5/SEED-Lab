# SEED Lab Mini-Project
### Purpose
The purpose of this project is to create a system which takes in the location of a marker in a camera's view, and converts that location to an angle that a wheel 
should move to. The computer vision subsystem will be ran off of a Raspberry Pi and will tranmit its data via I2C to an Arduino. The Arduino will control a servo motor using 
the SHIELD motor controller in order to convert the received angle to instructions for the servo. The servo's motion will be implemented using a PID controller designed in MATLAB.

### File Organization
The code for each individual subsystem leading up to the final product will be stored in a unique folder for each subsystem. Finalized versions of this code will be stored in 
a folder titled "Final Product". Code in other folders represents the individual work of each subsystem leading up to the final integration.
