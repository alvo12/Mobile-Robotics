/*
  RoboCar.h - Custom Library for 4WD Mecanum Robot
  Abstracts motor, servo, and ultrasonic control for easy learning.
*/

#ifndef ROBOCAR_H
#define ROBOCAR_H

#include <Arduino.h>
#include <AFMotor.h>
#include <Servo.h>

class RoboCar {
  public:
    // Constructor
    RoboCar();

    // Initialization (Call this in setup)
    void begin();

    // --- Motor Control ---
    void moveForward(uint8_t speed = 180);
    void moveBackward(uint8_t speed = 180);
    void stop();
    void turnLeft(uint8_t speed = 180);
    void turnRight(uint8_t speed = 180);

    // --- Sensors & Actuators ---
    void look(int angle); // Controls the servo (0 to 180)
    long getDistance();   // Returns ultrasonic distance in cm

  private:
    // Hardware Objects (Hidden from the user)
    AF_DCMotor motorFL;
    AF_DCMotor motorFR;
    AF_DCMotor motorRL;
    AF_DCMotor motorRR;
    Servo scanServo;

    // Pin Definitions
    const uint8_t TRIG_PIN = A0;
    const uint8_t ECHO_PIN = A3;
    const uint8_t SERVO_PIN = 10;
};

#endif