// Line following program
// Nick Benoit

// Initialize libraries
#include <Servo.h>           // servo library

Servo rightmotor;            // Servo object
Servo leftmotor;             // Servo object
Servo armmotor;              // Servo object

// Input signals from the photoresistor
const int psright = 3;      // Pin the right sensor is attached to
const int psleft = 2;       // Pin the left sensor is attached to

// Input signal from the potentionmeter
const int potPin = 4;       // Pin the potentiometer is attached to

// State of the sensor variables
int stateRight = 0;          // Current state of the right sensor
int stateLeft = 0;           // Current state of the left sensor
int statePot = 0;            // Current state of the potentiometer

// Load and route state variables
boolean hasLoad = false;
boolean drivenRoute = false;

// Robot control variables
const int STOP = 90;
const double CW = 180 ;
const double CCW = 80;

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

// Stop the motors
void stopRobot() {
  leftmotor.write(STOP);
  rightmotor.write(STOP);
}

// Turn right until line is encountered
void turnRight() {
  leftmotor.write(CW-40);
  rightmotor.write(CW-40);
  delay(10);
  stopRobot();
}

// Turn left until line is encountered
void turnLeft() {
  leftmotor.write(CCW+40);
  rightmotor.write(CCW+40);
  delay(10);
  stopRobot();
}

// Drive forward while the line is between the sensors
void goForward() {
  leftmotor.write(CCW+50);
  rightmotor.write(CW-50);
  delay(20);
}

// Rotate the arm until the desired angle is reached
void moveArm(int degree) {
  int moveAngle = 0;
  moveAngle = potToDegree(statePot) - degree;
  if (moveAngle > 0) {
    armmotor.write(CW);
    delay(20);
  }
  else if (moveAngle < 0) {
    armmotor.write(CCW);
    delay(20);  
  }
  else if (moveAngle == 0) {
    hasLoad = true;
  }
}

// Convert potentiometer input to a degree measurement
int potToDegree (int potValue) {
  return (potValue*99999); // <--------find this value
}

// Decide to drive or to pick up/drop off the load
void motion() {
  if ((hasLoad == false) && (drivenRoute == false)) {
    moveArm(90);
  }
  else if ((hasLoad == true) && (drivenRoute == false)) {
    lineFollow();
  }
  else if ((hasLoad == true) && (drivenRoute == true)) {
    moveArm(45);
  }
}

// Calls sensorState and drive
void lineFollow() {
  sensorState();
  drive();
}

// Updates global sensor state variables
void sensorState() {
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
  // If both photoresistors are on the line
  else if ((stateRight == 0) && (stateLeft == 0)) {
    stopRobot();
    drivenRoute = true;
  }
}

// Continually call lineFollow
void loop() {
  motion();
}
