/*
  RoboCar.cpp - Implementation of the RoboCar Library
*/

#include "RoboCar.h"

// Constructor: Initializes the AFMotor objects on their specific ports
RoboCar::RoboCar() : motorFL(1), motorFR(2), motorRL(3), motorRR(4) {
    // Initialization happens here before the sketch runs
}

void RoboCar::begin() {
    // Set up ultrasonic pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Set up and center the servo
    scanServo.attach(SERVO_PIN);
    look(90); // Default to center
    delay(500);

    // Ensure motors are stopped on boot
    stop();
}

// ===================== Motor Control =====================

void RoboCar::moveForward(uint8_t speed) {
    motorFL.setSpeed(speed);
    motorFR.setSpeed(speed);
    motorRL.setSpeed(speed);
    motorRR.setSpeed(speed);

    motorFL.run(FORWARD);
    motorFR.run(FORWARD);
    motorRL.run(FORWARD);
    motorRR.run(FORWARD);
}

void RoboCar::moveBackward(uint8_t speed) {
    motorFL.setSpeed(speed);
    motorFR.setSpeed(speed);
    motorRL.setSpeed(speed);
    motorRR.setSpeed(speed);

    motorFL.run(BACKWARD);
    motorFR.run(BACKWARD);
    motorRL.run(BACKWARD);
    motorRR.run(BACKWARD);
}

void RoboCar::stop() {
    motorFL.run(RELEASE);
    motorFR.run(RELEASE);
    motorRL.run(RELEASE);
    motorRR.run(RELEASE);
}

// Simple pivot turns (Tank style)
void RoboCar::turnLeft(uint8_t speed) {
    motorFL.setSpeed(speed);
    motorFR.setSpeed(speed);
    motorRL.setSpeed(speed);
    motorRR.setSpeed(speed);

    motorFL.run(RELEASE);
    motorRL.run(FORWARD);
    motorFR.run(RELEASE);
    motorRR.run(FORWARD);
}

void RoboCar::turnRight(uint8_t speed) {
    motorFL.setSpeed(speed);
    motorFR.setSpeed(speed);
    motorRL.setSpeed(speed);
    motorRR.setSpeed(speed);

    motorFL.run(FORWARD);
    motorRL.run(RELEASE);
    motorFR.run(FORWARD);
    motorRR.run(RELEASE);
}

// ===================== Sensors & Actuators =====================

void RoboCar::look(int angle) {
    // Constrain the angle between 0 and 180 for safety
    angle = constrain(angle, 0, 180);
    scanServo.write(angle);
}

long RoboCar::getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return 0; // Return 0 if no echo
    
    return duration * 0.0343 / 2; // Convert to cm
}