/*
   Final Project
   Brady Huntington, Quinn Marsh, Ankit Maurya, Ryo Takatori, Nick Williams
   AERO 465
   03/06/2020
*/

//Libraries
#include <Arduino.h>
#include <Servo.h>
#include "Header.h"

//Instances
Servo myServoHand;
Servo myServoArm;
Servo myServoBase;
Calibration calibration;
Kalman kalman;
UltraSonic ultraSonic;
Radar radar;
Arm arm;

//Assign pins and variables
int servoHandPin = 3; //Digital PWM pin used by the servo 1
int servoArmPin = 5; //Digital PWM pin used by the servo 2
int servoBasePin = 6;
int trigPin = 7;
int echoPin = 8;
int joyX = 0;   //Analog pin to which the joystick (X) is connected
int joyY = 1;   //Analog pin to which the joystick (Y) is connected
int pot = 2;
bool isThereSomething = false;
int currentAngle = 11;
int saveAngle = 11;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //Servos
  myServoHand.attach(servoHandPin);
  myServoArm.attach(servoArmPin);
  myServoBase.attach(servoBasePin);
  //Pins
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  //Calibration
  calibration.calibrationPrompt();
  calibration.calibrationSequence(trigPin, echoPin);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Radar
  radar.finder(trigPin, echoPin, myServoBase, &isThereSomething, &currentAngle, calibration.distCalValue, saveAngle);
  saveAngle = currentAngle;//Save angle it stopped at
  //Arm
  arm.armMover(pot, joyY, joyX, myServoHand, myServoArm, myServoBase, isThereSomething, currentAngle);
  Serial.println(currentAngle);
}
