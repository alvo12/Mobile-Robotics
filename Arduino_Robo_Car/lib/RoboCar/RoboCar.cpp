/*
  RoboCar.cpp - Implementation of the RoboCar motor control library
*/

#include "RoboCar.h"

// ------------------------------------------------------------
// Constructor
// Ports match the physical shield terminals M1-M4
// ------------------------------------------------------------
RoboCar::RoboCar() : _motorFL(1), _motorFR(2), _motorRL(3), _motorRR(4) {}

void RoboCar::begin() {
    stop(); // Ensure motors are idle on boot
}

// ------------------------------------------------------------
// Basic motion
// ------------------------------------------------------------
void RoboCar::moveForward(uint8_t speed) {
    _setAll(FORWARD, speed);
}

void RoboCar::moveBackward(uint8_t speed) {
    _setAll(BACKWARD, speed);
}

void RoboCar::stop() {
    _motorFL.run(RELEASE);
    _motorFR.run(RELEASE);
    _motorRL.run(RELEASE);
    _motorRR.run(RELEASE);
}

// ------------------------------------------------------------
// Tank-style pivot turns
// One side drives, the other releases (freewheels)
// For a tighter spin, swap RELEASE for BACKWARD on the idle side
// ------------------------------------------------------------
void RoboCar::turnLeft(uint8_t speed) {
    // Right side drives forward, left side freewheels
    _motorFL.setSpeed(0);
    _motorRL.setSpeed(0);
    _motorFR.setSpeed(speed);
    _motorRR.setSpeed(speed);

    _motorFL.run(RELEASE);
    _motorRL.run(RELEASE);
    _motorFR.run(FORWARD);
    _motorRR.run(FORWARD);
}

void RoboCar::turnRight(uint8_t speed) {
    // Left side drives forward, right side freewheels
    _motorFL.setSpeed(speed);
    _motorRL.setSpeed(speed);
    _motorFR.setSpeed(0);
    _motorRR.setSpeed(0);

    _motorFL.run(FORWARD);
    _motorRL.run(FORWARD);
    _motorFR.run(RELEASE);
    _motorRR.run(RELEASE);
}

// ------------------------------------------------------------
// Arc turns - both sides move, one faster than the other
// Produces a smooth curve rather than a pivot
// ------------------------------------------------------------
void RoboCar::arcLeft(uint8_t outerSpeed, uint8_t innerSpeed) {
    // Right side is the outer (faster) arc
    _motorFL.setSpeed(innerSpeed);
    _motorRL.setSpeed(innerSpeed);
    _motorFR.setSpeed(outerSpeed);
    _motorRR.setSpeed(outerSpeed);

    _motorFL.run(FORWARD);
    _motorRL.run(FORWARD);
    _motorFR.run(FORWARD);
    _motorRR.run(FORWARD);
}

void RoboCar::arcRight(uint8_t outerSpeed, uint8_t innerSpeed) {
    // Left side is the outer (faster) arc
    _motorFL.setSpeed(outerSpeed);
    _motorRL.setSpeed(outerSpeed);
    _motorFR.setSpeed(innerSpeed);
    _motorRR.setSpeed(innerSpeed);

    _motorFL.run(FORWARD);
    _motorRL.run(FORWARD);
    _motorFR.run(FORWARD);
    _motorRR.run(FORWARD);
}

// ------------------------------------------------------------
// Individual motor control - for advanced users
// motor: 1=FL, 2=FR, 3=RL, 4=RR
// direction: FORWARD, BACKWARD, RELEASE (AFMotor constants)
// ------------------------------------------------------------
void RoboCar::setMotor(uint8_t motor, uint8_t direction, uint8_t speed) {
    AF_DCMotor* m = nullptr;
    switch (motor) {
        case 1: m = &_motorFL; break;
        case 2: m = &_motorFR; break;
        case 3: m = &_motorRL; break;
        case 4: m = &_motorRR; break;
        default: return; // Invalid motor number, do nothing
    }
    m->setSpeed(speed);
    m->run(direction);
}

// ------------------------------------------------------------
// Private helper
// ------------------------------------------------------------
void RoboCar::_setAll(uint8_t direction, uint8_t speed) {
    _motorFL.setSpeed(speed);
    _motorFR.setSpeed(speed);
    _motorRL.setSpeed(speed);
    _motorRR.setSpeed(speed);

    _motorFL.run(direction);
    _motorFR.run(direction);
    _motorRL.run(direction);
    _motorRR.run(direction);
}
