/*Servo*/
#include <Servo.h>    // Servo library
Servo servoR;   // Right Servo instance
Servo servoL;   // Left Servo instance
#define servoR_pin 7    // Pin for Right Servo
#define servoL_pin 6    // Pin for Left Servo
int initial_positionR = 86;   // Initial position for Right Servo
int initial_positionL = 95;   // Initial position for Left Servo

/*Joystick*/
#define x_key A0    // Pin for X axis of Joystick
#define y_key A1    // Pin for Y axis of Joystick
int x_pos;    // Variable for X axis value
int y_pos;    // Variable for Y axis value

/*Stepper*/
#include <Stepper.h>    // Stepper library
#define STEPS 200   // Number of steps per revoultion for Arm Stepper
#define IN1 11    // Arm Stepper pin 1
#define IN2 10    // Arm Stepper pin 2
#define IN3 9   // Arm Stepper pin 3
#define IN4 8   // Arm Stepper pin 4
Stepper armStepper(STEPS, IN1, IN2, IN3, IN4);    // Arm Stepper instance

void setup() {
  /*Serial communication starts*/
  Serial.begin (9600);

  /*Servo setup*/
  servoR.attach (servoR_pin);   // Attach Right Servo to pin
  servoL.attach (servoL_pin);   // Attach Left Servo to pin
  servoR.write (initial_positionR);   // Set initial position for Right Servo
  servoL.write (initial_positionL);   // Set initial position for Left Servo

  /*Joystick setup*/
  pinMode (x_key, INPUT);   // Set Joystick X pin to an input
  pinMode (y_key, INPUT);   // Set Joystick Y pin to an input

  /*Stepper setup*/
  pinMode(IN1, OUTPUT);   // Set stepper pin 1 to output
  pinMode(IN2, OUTPUT);   // Set stepper pin 2 to output
  pinMode(IN3, OUTPUT);   // Set stepper pin 3 to output
  pinMode(IN4, OUTPUT);   // Set stepper pin 4 to output
}


void loop() {
  /*Read in Joystick Values*/
  x_pos = analogRead (x_key);   // Read X axis value from Joystick
  y_pos = analogRead (y_key);   // Read Y axis value from Joystick

  /*Arm Control - Upward movement*/
  if (y_pos < 300){                                    
    if (initial_positionR < 36 && initial_positionL > 145){}    // Do nothing here
    else{ 
      initial_positionR--;    // Decrease the angle of the Right Servo
      initial_positionL++;    // Increase the angle of the Left Servo
      servoR.write (initial_positionR);   // Move the Right Servo to the desired angle
      servoL.write (initial_positionL);   // Move the Left Servo to the desired angle
      delay(50);    // Delay for smooth movement
    }
  }

  /*Arm Control - Downward movement*/
  else if (y_pos > 700){
    if (initial_positionR > 85 && initial_positionL < 96){}   // Do nothing here
    else{
      initial_positionR++;    // Increase the angle of the Right Servo
      initial_positionL--;    // Decrease the angle of the Left Servo
      servoR.write (initial_positionR);   // Move the Right Servo to the desired angle
      servoL.write (initial_positionL);   // Move the Right Servo to the desired angle
      delay(50);    // Delay for smooth movement of the Servo
    }
  }
  /*Arm Control - No horizontal movement*/
  if (x_pos > 500 && x_pos < 523){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  /*Arm Control - Rightwar movement*/
  else {
    while(x_pos >= 723){
      int armSpeed = map(x_pos, 723, 1023, 5, 30);
      armStepper.setSpeed(armSpeed);
      armStepper.step(1);
      x_pos = analogRead (x_key);
      delay(20);
    }
    while(x_pos <= 300){
      int armSpeed = map(x_pos, 300, 0, 5, 30);
      armStepper.setSpeed(armSpeed);
      armStepper.step(-1);
      x_pos = analogRead (x_key);
      delay(20);
    }
  }   
}
