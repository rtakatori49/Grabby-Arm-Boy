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

//Calibration prompt for user
void Calibration::calibrationPrompt() {
  Serial.println(F("Please set ultrasonic sensor 5 [cm] away from a flat object and press enter in console to begin calibration."));
  while (Serial.available() <= 0) {
    userInput = Serial.readStringUntil('\n');
    if (userInput == "start") {
      return;
    }
  }
}

// Ultrasonic sensor distance calculation
float UltraSonic::calculateDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2;
  return distance;
}

//Calibration sequence for ultrasonic sensor
void Calibration::calibrationSequence(int trigPin, int echoPin) {
  UltraSonic ultraSonic;
  Serial.println(F("Start Calibration"));
  //Collection of data for sample size
  do {
    distance = ultraSonic.calculateDistance(trigPin, echoPin);
    distCalSum = distCalSum + distance;
    Serial.println(distCalSum);
    count1++;
  } while (count1 != sampleSize); // 1024 times
  //Give average
  distCalSum = distCalSum / sampleSize;
  distCalValue = 5 / distCalSum;
  Serial.println(F("End Calibration"));
  Serial.print(F("Calibration Result (Offset): "));
  Serial.println(distCalValue);
}

//Kalman filter
float Kalman::kf(int trigPin, int echoPin, float distCalValue) {
  UltraSonic ultraSonic;
  distance = ultraSonic.calculateDistance(trigPin, echoPin);
  distance = distance * distCalValue;
  distMea = distance; //Measured distance
  distErrorEst = distErrorEst + distVarProcess; //Estimate error adding in variation
  distKg = distErrorEst / (distErrorEst + distErrorMea); //Kalman gain
  distEst = distEst + distKg * (distMea - distEst); //Estimate
  distErrorEst = (1 - distKg) * distErrorEst; //Estimate error
  return distEst;
}

//Radar object finder
void Radar::finder(int trigPin, int echoPin, Servo myServoBase, bool* isThereSomething, int* currentAngle, float distanceCalValue, int saveAngle) {
  Kalman kalman;
  //Returning turn
  for (int i = saveAngle; i > 10; i--) {
    myServoBase.write(i);
    delay(10);
    //Apply Kalman filter
    for (int j = 0; j <= 3; j++) {
      distance = kalman.kf(trigPin, echoPin, distanceCalValue);
    }
    Serial.print(i);
    Serial.print(" [deg],");
    Serial.print(distance);
    Serial.println(" [cm]");
    //Let user control teleop
    userInput = Serial.read();
    if (userInput == '\n') {
      *isThereSomething = false;
      *currentAngle = i;
      return;
    }
    //Stop when object is within 15 [cm]
    if (distance < 15) {
      *isThereSomething = true;
      *currentAngle = i;
      return;
    }
  }
  do {
    Serial.println("For teleoperated arm, press 'ENTER' into console.");
    //First turn
    for (int i = 10; i <= 170; i++) {
      myServoBase.write(i);
      delay(10);
      //Apply Kalman filter
      for (int j = 0; j <= 3; j++) {
        distance = kalman.kf(trigPin, echoPin, distanceCalValue);
      }
      Serial.print(i);
      Serial.print(" [deg],");
      Serial.print(distance);
      Serial.println(" [cm]");
      //Let user control teleop
      userInput = Serial.read();
      if (userInput == '\n') {
        *isThereSomething = false;
        *currentAngle = i;
        return;
      }
      //Stop when object is within 15 [cm]
      if (distance < 15) {
        *isThereSomething = true;
        *currentAngle = i;
        return;
      }
    }
    //Returning turn
    for (int i = 165; i > 10; i--) {
      myServoBase.write(i);
      delay(10);
      //Apply Kalman filter
      for (int j = 0; j <= 3; j++) {
        distance = kalman.kf(trigPin, echoPin, distanceCalValue);
      }
      Serial.print(i);
      Serial.print(" [deg],");
      Serial.print(distance);
      Serial.println(" [cm]");
      //Let user control teleop
      userInput = Serial.read();
      if (userInput == '\n') {
        *isThereSomething = false;
        *currentAngle = i;
        return;
      }
      //Stop when object is within 15 [cm]
      if (distance < 15) {
        *isThereSomething = true;
        *currentAngle = i;
        return;
      }
    }
    *isThereSomething = false;
  } while (Serial.available() <= 0);
}

//Arm movement
void Arm::armMover(int pot, int joyY, int joyX, Servo myServoHand, Servo myServoArm, Servo myServoBase, bool isThereSomething, int currentAngle) {
  //Restrict base rotation at specific angle when object is found
  if (isThereSomething == true) {
    while (Serial.available() <= 0) {
      //Load peripherals and map values
      valHand = analogRead(pot);
      valArm = analogRead(joyY); //Read the joystick Y value (value between 0 and 1023)
      valBase = analogRead(joyX); //Read the joystick X value (value between 0 and 1023)
      valHand = map(valHand, 0, 1023, 0, 180);
      valArm = map(valArm, 0, 1023, 10, 170); //Scale the joystick X value to use it with the servo
      myServoHand.write(valHand);
      delay(30);
      myServoArm.write(valArm);
      delay(30);
      myServoBase.write(currentAngle);
      delay(30);
      // Allow usuer to exit
      Serial.println("Finder Mode: Once completed with task, please press 'ENTER' into console.");
      userInput = Serial.read();
      if (userInput == '\n') {
        return;
      }
    }
  }
  //Free all movements when user asks
  else {
    while (Serial.available() <= 0) {
      //Load peripherals and map values
      valHand = analogRead(pot);
      valArm = analogRead(joyY); //Read the joystick Y value (value between 0 and 1023)
      valBase = analogRead(joyX); //Read the joystick X value (value between 0 and 1023)
      valHand = map(valHand, 0, 1023, 0, 180);
      valArm = map(valArm, 0, 1023, 10, 170); //Scale the joystick X value to use it with the servo
      valBase = map(valBase, 0, 1023, 10, 170); //Scale the joystick X value to use it with the servo
      myServoHand.write(valHand);
      delay(30);
      myServoArm.write(valArm);
      delay(30);
      myServoBase.write(valBase);
      delay(30);
      // Allow usuer to exit
      Serial.println("Teleop Mode: Once completed with task, please press 'ENTER' into console.");
      userInput = Serial.read();
      if (userInput == '\n') {
        return;
      }
    }
  }
}
