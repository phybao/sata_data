#include <math.h>
#include <TimerOne.h>
#include "ArduPID.h"

// Define motor control pins
int enL = 4;
int inL1 = 9;
int inL2 = 8;
int enR = 5;
int inR1 = 11;
int inR2 = 10;

// Define encoder pins
int enLA = 2;
int enLB = 3;
int enRA = 18;
int enRB = 19;

// Variables to count encoder pulses
volatile int leftEnCount = 0;
volatile int rightEnCount = 0;

// Position tracking variables
float xPos = 0.0, yPos = 0.0, theta = 0.0; // Robot's position and orientation
const float wheelCircumference = 45.0 * M_PI / 1000; // Wheel diameter in mm
const float wheelBase = 186.0 / 1000;                // Distance between wheels in mm
const int PPR = 2100;                         // Pulses per revolution
float distancePerPulse = wheelCircumference / PPR;

// Movement signs
int signL = 1;
int signR = 1;

// Minimum torque compensation
const int mintorqueL = 20;
const int mintorqueR = 20;
const int compensator = 15;

void setup() {
  Serial.begin(9600);

    // Wait for start signal from Python
  while (true) {
    if (Serial.available() > 0) {
      char command = Serial.read();
      if (command == 's') { // 's' is the start signal
        Serial.println("Start signal received. Beginning operation...");
        break;
      }
    }
  }

  // Set up motor control pins as outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(enLA), leftEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enLB), leftEnISRB, RISING);
  attachInterrupt(digitalPinToInterrupt(enRA), rightEnISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(enRB), rightEnISRB, RISING);

  // Initialize TimerOne for position updates (every 50 ms)
  Timer1.initialize(50000); // 50,000 µs = 50 ms
  Timer1.attachInterrupt(updatePosition);

  stop(); // Start with motors stopped
}

void loop() {
  // Follow the rectangular path
  delay(2000);
  rec_path();
  
  // Stop movement after completing the path
  stop();

  while (1); // Halt the program
}

// Interrupt function to update position
void updatePosition() {
  float leftDistance = leftEnCount * distancePerPulse * signL;
  float rightDistance = rightEnCount * distancePerPulse * signR;


  float deltaDistance = (leftDistance + rightDistance) / 2.0;
  float deltaTheta = (rightDistance - leftDistance) / wheelBase;

  theta += deltaTheta; // Update orientation
    // Normalize theta to [-PI, PI] or [0, 2PI] as needed
  if (theta > PI) theta -= 2 * PI;
  if (theta < -PI) theta += 2 * PI;  
  xPos += deltaDistance * cos(theta);
  yPos += deltaDistance * sin(theta);

  // Reset encoder counts
  leftEnCount = 0;
  rightEnCount = 0;

  // Print position and orientation to Serial
  Serial.print("x: ");
  Serial.print(xPos, 2); // Print x with 2 decimal places
  Serial.print(" m, y: ");
  Serial.print(yPos, 2); // Print y with 2 decimal places
  Serial.print(" m, theta: ");
  Serial.println(theta * 180.0 / M_PI, 2); // Convert theta to degrees and print
}

// Function to follow a rectangular path
void rec_path() {
  // Move forward 120 cm
  move(50, 50);
  delay(5000); // Adjust based on speed and distance
  stop();
  delay(1000);

  // Turn 90 degrees right
  turnR(120, 0);
  delay(1185); // Adjust based on turn speed and angle
  stop();
  delay(1000);

  // Move forward 120 cm
  move(50, 50);
  delay(3500);
  stop();
  delay(1000);

  // Turn 90 degrees right
  turnR(120, 0);
  delay(1170);
  stop();
  delay(1000);

  // Move forward 120 cm
  move(50, 50);
  delay(4500);
  stop();
  delay(1000);

  // Turn 90 degrees right
  turnR(120, 0);
  delay(1150);
  stop();
  delay(1000);

  // Move forward 60 cm
  move(50, 50);
  delay(2000); // Adjust for distance
  stop();
  delay(1000);
}

// Function to move robot with specified left and right speeds
void move(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    signL = 1;
    digitalWrite(inL1, LOW);
    digitalWrite(inL2, HIGH);
  } else {
    signL = -1;
    digitalWrite(inL1, HIGH);
    digitalWrite(inL2, LOW);
  }

  if (rightSpeed >= 0) {
    signR = 1;
    digitalWrite(inR1, HIGH);
    digitalWrite(inR2, LOW);
  } else {
    signR = -1;
    digitalWrite(inR1, LOW);
    digitalWrite(inR2, HIGH);
  }

  analogWrite(enR, abs(rightSpeed) + mintorqueR);
  analogWrite(enL, abs(leftSpeed) + mintorqueL + compensator);
}

// Function to turn robot right
void turnR(int leftSpeed, int rightSpeed) {
  analogWrite(enR, abs(rightSpeed));
  analogWrite(enL, abs(leftSpeed));
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, HIGH);
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, HIGH);
}

void stop() {
  // Turn off motors
  digitalWrite(inR1, LOW);
  digitalWrite(inR2, LOW);
  digitalWrite(inL1, LOW);
  digitalWrite(inL2, LOW);
}

void leftEnISRA() {
  leftEnCount++;
}

void leftEnISRB() {
  leftEnCount++;
}

void rightEnISRA() {
  rightEnCount++;
}

void rightEnISRB() {
  rightEnCount++;
}



