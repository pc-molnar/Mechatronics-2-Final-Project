# Mechatronics-2-Final-Project

# Overview
This project was created in the Fall of 2023 by Patrick Molnar and Austin Cramer in Dr. Clark's Mechatronics 2 class at the FAMU-FSU College of Engineering. The project showcases a robotic arm and dexterous hand for object manipulation.

# Features
Arm Control: The arm was controlled by user input via a joystick using the Elegoo Mega microcontroller. It featured a Nema 17 stepper motor and two MG996R servo motors to achieve left and right and up and down movements, respectfully. 

Hand Control: The hand utilized the Teensy 4.0 microcontroller and also featured a joystick, except the functionality was different. The y-axis of the joystick controlled the 28BYJ-48 stepper motor used to move the "thumb" forward and backwards, and the x-axis of the joystick initiated the PID control sequence that opened and closed the two "index fingers." The "index fingers" utilized two 380:1 geared DC motors with encoders. The geared DC motors were selected by analyzing a speed-torque curve that was created to fit the needs of the project. 

# Sensors
Force Sensitive Resistor (FSR): Two FSRs were utilized on the tips of the "index fingers" and provided feedback on how firm the hand was grasping the object for the PID control. 

# Hardware
L293D Motor Driver: This drove the two 380:1 geared DC motors.

LM358PE3 Op-Amp: This facilitated the signal conditioning for the FSRs.
