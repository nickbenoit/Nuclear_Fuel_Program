// Line following program
// Nick Benoit

// Initialize libraries
#include <Servo.h>           // servo library

// Initialize motor objects
Servo rightmotor;            // Servo object
Servo leftmotor;             // Servo object
Servo armmotor;              // Servo object

// Pin input signals from the photoresistor
const int psright = 3;      // Pin the right sensor is attached to
const int psleft = 2;       // Pin the left sensor is attached to

// Pin input signal from the potentionmeter
const int potPin = 0;       // Pin the potentiometer is attached to

// Initialize sensor state variables
int stateRight = 0;          // Current state of the right sensor
int stateLeft = 0;           // Current state of the left sensor
int statePot = 0;            // Current state of the potentiometer

// Initialize load and route state variables
boolean hasLoad = false;
boolean drivenToCP1 = false;
boolean drivenToCP2 = false;

// Robot control variables
const int STOP = 90;
const double CW = 180;
const double CCW = 0;

void setup() {
  // Initialize inputs from light sensors
  pinMode(psright, INPUT);
  pinMode(psleft, INPUT);
  // Begin serial communication
  Serial.begin(9600);
  // Initialize motors
  rightmotor.attach(4, 1000, 2000);   // right drive motor pin#, pulse time for 0, pulse time for 180
  leftmotor.attach(5, 1000, 2000);    // left drive motor pin#, pulse time for 0, pulse time for 180
  armmotor.attach(6, 1000, 2000);     // arm motor pin#, pulse time for 0, pulse time for 180
}

// ************************* Robot Driving Functions *************************
// Stop the motors
void stopRobot() {
  leftmotor.write(STOP);
  rightmotor.write(STOP);
}

// Turn right until line is encountered
void turnRight() {
  leftmotor.write(CW);
  rightmotor.write(CW);
  delay(10);
  //stopRobot();
}

// Turn left until line is encountered
void turnLeft() {
  leftmotor.write(CCW);
  rightmotor.write(CCW);
  delay(10);
  //stopRobot();
}

// Drive forward while the line is between the sensors
void goForward() {
  leftmotor.write(CCW+10);
  rightmotor.write(CW-10);
  delay(20);
}

// ************************* Arm Control Functions *************************
// Reads the potentiometer state and maps the value to a degree measurement
int potDegree () {
  statePot = analogRead(potPin);
  int mapPot = map(statePot, 595, 1023, -45, 90);
  return (mapPot);
}

// Calculates the error by subracting the goal position from the current position
int calcError (int goal) {
  return(potDegree() - goal);
}

// Find the error, map it to a motor value, then write the value to the arm motor
void moveArm(int degree) {
  // Add 45 degrees to the desired angle when calling the 
  // function so that it stops at the right value
  degree+=45;
  int error = calcError(degree);
  int motorVal = map(error, -135, 135, 180, 0);

  // If error is within acceptable bounds, stop motor and toggle hasLoad
  if ((60 < motorVal) && (motorVal < 150)) {
    hasLoad = true;
  }
  else {
    armmotor.write(motorVal);
    Serial.println(motorVal);
    delay(20);
  }
}

// ************************* Line Following Functions *************************
// Calls photosensorState to update the photosensor values
// Calls drive to tell the robot how to move based on the above values
void lineFollow() {
  photosensorState();
  drive();
}

// Updates global sensor state variables
void photosensorState() {
  stateRight = digitalRead(psright);
  stateLeft = digitalRead(psleft);
}

// Controls motor output based on sensor state
void drive() {
  // If the right photoresistor is on the line, but not the left
  if ((stateRight == 0) && (stateLeft == 1)) {
    turnRight();
  }
  // If the left photoresistor is on the line, but not the right
  else if ((stateRight == 1) && (stateLeft == 0)) {
    turnLeft();
  }
  // If both photoresistors are off the line
  else if ((stateRight == 1) && (stateLeft == 1)) {
    goForward();
  }
  // If both photoresistors are on the line, and it hasn't been to checkpoint 1
  else if ((stateRight == 0) && (stateLeft == 0) && (drivenToCP1 == false)) {
    stopRobot();
    drivenToCP1 = true;
  }
  // If both photoresistors are on the line, and it has been to checkpoint 1
  else if ((stateRight == 0) && (stateLeft == 0) && (drivenToCP1 == true)) {
    stopRobot();
    drivenToCP2 = true;
  }
}

// ************************* Motion Decision Function *************************
// Decide to drive or to pick up/drop off the load
void motion() {
  // At the beginning of the route, the robot neither has the load nor has it driven anywhere
  // Move the arm up to 45 degrees to hold the load
  if ((hasLoad == false) && (drivenToCP1 == false) && (drivenToCP2 == false)) {
    moveArm(45);               
  }
  // The robot has picked up the load, but hasn't driven to the first checkpoint yet
  // Follow the line to the first checkpoint
  else if ((hasLoad == true) && (drivenToCP1 == false) && (drivenToCP2 == false)) {
    armmotor.write(90);
    lineFollow();
  }
  // The robot has reached the checkpoint and still has the load
  // Move the arm down to release the load
  else if ((hasLoad == true) && (drivenToCP1 == true) && (drivenToCP2 == false)) {
    moveArm(-45);
  }
  // The robot has reached the checkpoint and dropped off the load
  // Follow the line to the final checkpoint
  else if ((hasLoad == false) && (drivenToCP1 == true) && (drivenToCP2 == false)) {
    lineFollow();
  }
}

// Continually call motion to decide what to do
void loop() {
  motion();
}
