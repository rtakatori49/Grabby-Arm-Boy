/*
   Final Project
   Brady Huntington, Quinn Marsh, Ankit Maurya, Ryo Takatori, Nick Williams
   AERO 465
   03/06/2020
*/

//Define header and include libraries
#ifndef Header_h
#define Header_h
#include <Arduino.h>
#include <Servo.h>

//Classes, functions, and variables

//Ultrasonic sensor
class UltraSonic {
  public:
    float calculateDistance(int trigPin, int echoPin);
    float duration;
    float distance;
  private:
};

//Calibration
class Calibration: public UltraSonic {
  public:
    // Variables and Functions
    String userInput;
    void calibrationPrompt();
    void calibrationSequence(int trigPin, int echoPin);
    float distance;
    float distCalValue;
    float distCalSum = 0;
    int sampleSize = 100;
    unsigned int count1 = 0;
  private:
};

//Kalman filter
class Kalman: public UltraSonic {
  public:
    float kf(int trigPin, int echoPin, float distanceCalValue);
    float distance;
    double distKg;
    float distEst = 2;
    double distErrorEst = 1;
    float distMea;
    double distErrorMea = 0.073361037;
    float distVarProcess = 0.001;
  private:
};

//Radar
class Radar: public Kalman {
  public:
    void finder(int trigPin, int echoPin, Servo myServoBase, bool* isThereSomething, int* currentAngle, float distanceCalValue, int saveAngle);
    float distance;
    char userInput;
  private:
};

//Arm
class Arm {
  public:
    void armMover(int pot, int joyY, int joyX, Servo myServoHand, Servo myServoArm, Servo myServoBase, bool isThereSomething, int currentAngle);
    int valHand;
    int valArm;
    int valBase;
    char userInput;
  private:
};
#endif
