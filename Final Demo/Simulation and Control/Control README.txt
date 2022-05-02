Alex Curtis
Simulation and Control README

For demo 2, I created an entirely separate source and header file for a class called Control. This allows the main Arduino code to be much more straightforward and significantly easier to understand. To make the robot move to a desired position(targetDistance) and rotation(targetAngle) within a program, all one needs to do is create a Control object outside of loop() and setup(), and call the member function drive(targetAngle, targetDistance) inside loop() and my code handles the rest. 
I used the same class files for the final demo, which is indicative of how well my code works. The only changes we made were adjusting MINIMUM_SETTLING_TIME from 3000 ms to 300 ms, which affects how much time the integral part of the controller has to make any adjustments. A higher value means more time for correcting error, which increases the accuracy of the robot at the expense of time.

(You can find my class files at /FinalDemoArduino/lib/Control/ )