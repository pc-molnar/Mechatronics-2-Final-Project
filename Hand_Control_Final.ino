/*DC motors*/
#define rightEnable 5   // Right motor pin enabling motor
#define rightDirectionA 3   // Right motor pin controlling A direction
#define rightDirectionB 4   // Right motor pin controlling B direction
#define leftEnable 8    // Left motor pin enabling motor
#define leftDirectionA 9    // Left motor pin controlling A direction
#define leftDirectionB 10   // Left motor pin controlling B direction

/*Encoders*/
#include <Encoder.h>    // Encoder library
#define rightEncoderA 6   // Pin A of the right encoder
#define rightEncoderB 7    // Pin B of the right encoder
#define leftEncoderA 11    // Pin A of the left encoder
#define leftEncoderB 12    // Pin B of the left encoder
volatile long rightEncoderPos = 0;    // Right encoder's current position
volatile long desiredRightPos = 0;    // Right encoder's desired position
volatile long leftEncoderPos = 0;   // Left encoder's current position
volatile long desiredLeftPos = 0;   // Left encoder's desired position
int startRightPos;    // Right encoder's initial position
int startLeftPos;   // Left encoder's initial position
Encoder rightEncoder(rightEncoderA, rightEncoderB);   // Object for right encoder
Encoder leftEncoder(leftEncoderA, leftEncoderB);    // Object for left encoder

/*Joystick*/
#define x_key 22    // Pin for x-axis joystick input
#define y_key 21    // Pin for y-axis joystick input
int x_pos;    // Joystick position for x-axis
int y_pos;    // Joystick position for y-axis

/*Hand stepper*/
#include <Stepper.h>    // Stepper library
#define STEPS 32    // Number of steps per revolution
#define IN1 20    // Stepper motor IN1 pin
#define IN2 19    // Stepper motor IN2 pin
#define IN3 18    // Stepper motor IN3 pin
#define IN4 17    // Stepper motor IN4 pin
Stepper handStepper(STEPS, IN4, IN2, IN3, IN1);   // Object for hand stepper motor

/*Gear specs*/
const float gearRatio = 379.17;   // Geared DC motor gear ratio
const int countsPerRevolution = 12;   // Number of counts per revolution for encoders
const int countsPerRevAfterGear = countsPerRevolution * gearRatio;    // Number of counts for a full revolution of geared DC motor

/*PID*/
const double Kp = 1;    // Proportional constant
const double Ki = 0.01;   // Integral constant
const double Kd = 0.0;    // Derivative constant

/*FSRs*/
#define rightPress 15   // Right FSR pin
#define leftPress 14    // Left FSR pin
int rightSensVal;   // Right FSR value
int leftSensVal;    // Left FSR value

void setup() {
  /*Initialize serial communication*/
  Serial.begin(9600);

  /*Initialize DC motor pins*/
  pinMode(rightEnable, OUTPUT);   // Set right DC motor enable pin to output
  pinMode(rightDirectionA, OUTPUT);   // Set right DC motor direction A pin to output
  pinMode(rightDirectionB, OUTPUT);   // Set right DC motor direction B pin to output
  pinMode(leftEnable, OUTPUT);    // Set left DC motor enable pin to output
  pinMode(leftDirectionA, OUTPUT);    // Set left DC motor direction A pin to output
  pinMode(leftDirectionB, OUTPUT);    // Set left DC motor direction B pin to output

  /*Initialize joystick pins*/
  pinMode (x_key, INPUT);   // Set x-axis joystick pin to input
  pinMode (y_key, INPUT);   // Set y-axis joystick pin to input

  /*Attach interrupt to encoder*/
  attachInterrupt(digitalPinToInterrupt(rightEncoderA), updateRightEncoderA, CHANGE);   // Attach interrupt for right encoder pin A
  attachInterrupt(digitalPinToInterrupt(rightEncoderB), updateRightEncoderB, CHANGE);   // Attach interrupt for right encoder pin B
  attachInterrupt(digitalPinToInterrupt(leftEncoderA), updateLeftEncoderA, CHANGE);   // Attach interrupt for left encoder pin A
  attachInterrupt(digitalPinToInterrupt(leftEncoderB), updateLeftEncoderB, CHANGE);   // Attach interrupt for left encoder pin B

  /*Record starting position of DC motors*/
  startRightPos = desiredRightPos;    // Set right encoder initial position
  startLeftPos = desiredLeftPos;    // Set left encoder initial position

  /*Set initial desired position for the DC motors*/
  desiredRightPos = countsPerRevAfterGear/3;
  desiredLeftPos = -countsPerRevAfterGear/3;
}

void loop() {
  /*Joystick reading*/
  x_pos = analogRead (x_key);   // Read x-axis joystick position
  y_pos = analogRead (y_key);   // Read y-axis joystick position

  /*Make deadzone in the y-axis of joystick for hand stepper control*/
  if (y_pos > 600 && y_pos < 623) {   // Deadzone
    digitalWrite(IN1, LOW);   // Force hand stepper IN1 pin low
    digitalWrite(IN2, LOW);   // Force hand stepper IN2 pin low
    digitalWrite(IN3, LOW);   // Force hand stepper IN3 pin low
    digitalWrite(IN4, LOW);   // Force hand stepper IN4 pin low
  }
  
  /*Hand stepper control*/
  else {
    while (y_pos >= 723) {    // Initate hand stepper upward movement
      int thumbSpeed = map(y_pos, 723, 1023, 5, 500);   // Map joystick position to hand stepper speed
      handStepper.setSpeed(thumbSpeed);   // Set hand stepper speed 
      handStepper.step(1);    // Move hand stepper up one step
      y_pos = analogRead (y_key);   // Read y-axis joystick position
      delay(20);    // Delay for stability
    }
    while (y_pos <= 500) {    // Initiate hand stepper downward movement
      int thumbSpeed = map(y_pos, 500, 200, 5, 500);  // Map joystick position to hand stepper speed
      handStepper.setSpeed(thumbSpeed);   // Set hand stepper speed 
      handStepper.step(-1);   // Move hand stepper down one step
      y_pos = analogRead (y_key);   // Read y-axis joystick position
      delay(20);    // Delay for stability
    }
  }

  /*DC motor control*/
  if (x_pos >= 723) {   // Initiate closing motion of index fingers
    while (x_pos > 500 ) {    // Keep PID running for closed position
      rightSensVal = analogRead(rightPress);    // Read in the right FSR value
      leftSensVal = analogRead(leftPress);    // Read in the left FSR value
      if (rightSensVal > 300) {   // If right FSR value reaches the threshold
        desiredRightPos = rightEncoderPos;    // Store new right encoder desired position
      }
      if (leftSensVal > 300) {    // If left FSR value reaches the threshold
        desiredLeftPos = leftEncoderPos;    // Store new left encoder desired position
      }
      
      /*Initiate PID control with the desired encoder positions*/
      rightPID();   // Initiate PID control for right DC motor
      leftPID();     // Initiate PID control for left DC motor
      x_pos = analogRead (x_key);   // Read in x-axis joystick position
      delay(20);    // Delay for stability
    }
  }
  if (x_pos <= 300) {   // Initiate opening motion of index fingers
    while (x_pos < 700) {   // Keep PID running for open position
      desiredRightPos = startRightPos;    // Set right encoder desired position to initial position
      desiredLeftPos = startLeftPos;    // Set left encoder desired position to initial position

      /*Initiate PID control with the desired encoder positions*/
      rightPID();   // Initiate PID control for right DC motor
      leftPID();    // Initiate PID control for left DC motor
      x_pos = analogRead (x_key);   // Read in x-axis joystick position
      delay(20);    // Delay for stability
    }
  }
}

/*Interrupts to read encoder count*/
void updateRightEncoderA() {    // Interrupt for right encoder channel A
  if (digitalRead(rightEncoderA) == digitalRead(rightEncoderB)) {   // If channel A is equal to channel B
    rightEncoderPos++;    // Increase right encoder position
  } 
  else {  // If channel A is not equal to channel B
    rightEncoderPos--;    // Decrease right encoder position
  }
}
void updateRightEncoderB() {    // Interrupt for right encoder channel B
  if (digitalRead(rightEncoderA) == digitalRead(rightEncoderB)) {   // If channel A is equal to channel B
    rightEncoderPos--;    // Decrease right encoder position
  } 
  else {    // If channel A is not equal to channel B
    rightEncoderPos++;    // Increase right encoder position
  }
}
void updateLeftEncoderA() {   // Interrupt for left encoder channel A
  if (digitalRead(leftEncoderA) == digitalRead(leftEncoderB)) {   // If channel A is equal to channel B
    leftEncoderPos++;   // Increase left encoder position
  } 
  else {    // If channel A is not equal to channel B
    leftEncoderPos--;   // Decrease left encoder position
  }
}
void updateLeftEncoderB() {   // Interrupt for left encoder channel B
  if (digitalRead(leftEncoderA) == digitalRead(leftEncoderB)) {   // If channel A is equal to channel B
    leftEncoderPos--;   // Decrease left encoder position
  } 
  else {    // If channel A is not equal to channel B
    leftEncoderPos++;   // Increase left encoder position
  }
}

/*PID to control right DC motor*/
void rightPID() {
  /*Initial right PID variables*/
  double rightError;    // Error in right position
  double prevRightError;    // Previous error in right position
  double rightI;    // Integral variable for right motor
  double rightD;    // Derivative variable for right motor

  /*Calculate PID components*/
  rightError = desiredRightPos - rightEncoderPos;    // Right proportional term for PID
  rightI = rightI + rightError;   // Right integral term for PID
  rightD = rightError - prevRightError;    // Right derivative term for PID

  /*Preventing rollover*/
  if (rightError == 0) {
    rightI = 0;
  }
  
  /*Compute PID output*/
  double rightOutput = Kp * rightError + Ki * rightI + Kd * rightD;

  /*Apply PID output to control the motor*/
  int rightMotorSpeed = abs(rightOutput);   // Set right motor speed to the absolute of the output
  if (rightOutput > 0) {
    /*Rotate clockwise*/
    analogWrite(rightEnable, rightMotorSpeed);
    digitalWrite(rightDirectionA, HIGH);
    digitalWrite(rightDirectionB, LOW);
    rightI -= rightError;   // Preventing rollover
  }
  else {
    /*Rotate counterclockwise*/
    analogWrite(rightEnable, rightMotorSpeed);
    digitalWrite(rightDirectionA, LOW);
    digitalWrite(rightDirectionB, HIGH);
    rightI -= rightError;   // Preventing rollover
  }
  prevRightError = rightError;    // Set current error to previous error for right position
}

/*PID to control left DC motor*/
void leftPID() {
  /*Initial right PID variables*/
  double leftError;   // Error in left position
  double prevLeftError;   // Previous error in left position
  double leftI;   // Integral variable for left motor
  double leftD;   // Derivative variable for left motor

  /*Calculate PID components*/
  leftError = desiredLeftPos - leftEncoderPos;    // Left proportional term for PID
  leftI = leftI + leftError;    // Left integral term for PID
  leftD = leftError - prevLeftError;    // Left derivative term for PID

  /*Preventing rollover*/
  if (leftError == 0) {
    leftI = 0;
  }
  
  /*Compute PID output*/
  double leftOutput = Kp * leftError + Ki * leftI + Kd * leftD;

  /*Apply PID output to control the motor*/
  int leftMotorSpeed = abs(leftOutput);   // Set left motor speed to the absolute of the output
  if (leftOutput > 0) {
    /*Rotate clockwise*/
    analogWrite(leftEnable, leftMotorSpeed);
    digitalWrite(leftDirectionA, LOW);
    digitalWrite(leftDirectionB, HIGH);
    leftI -= leftError;   // Preventing rollover
  }
  else {
    /*Rotate counterclockwise*/
    analogWrite(leftEnable, leftMotorSpeed);
    digitalWrite(leftDirectionA, HIGH);
    digitalWrite(leftDirectionB, LOW);
    leftI -= leftError;   // Preventing rollover
  }
  prevLeftError = leftError;    // Set current error to previous error for left position
}
