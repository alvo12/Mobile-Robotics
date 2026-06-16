/*
  RoboCar.cpp - Implementation of the RoboCar motor control library.

  All compass, PID, and heading math is private to this file.
  Users interact only through the public API declared in RoboCar.h.
*/

#include "RoboCar.h"

// ---------------------------------------------------------------
// Static constant definitions
// Defined here (not in .h) so they live in flash only once
// ---------------------------------------------------------------
const float    RoboCar::_KP           = 3.0f;
const float    RoboCar::_KI           = 0.0f;
const float    RoboCar::_KD           = 1.0f;
const uint8_t  RoboCar::_MAX_TRIM     = 80;
const float    RoboCar::_TURN_SPEED   = 180.0f;
const float    RoboCar::_TURN_TOLERANCE = 5.0f;
const unsigned long RoboCar::_TURN_TIMEOUT = 3000UL;

// ---------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------
RoboCar::RoboCar()
    : _motorFL(1), _motorFR(2), _motorRL(3), _motorRR(4),
      _lastHeading(0),
      _holdTarget(0), _holdIntegral(0),
      _holdLastError(0), _holdLastTime(0),
      _headingLocked(false)
{}

// ---------------------------------------------------------------
// begin()
// ---------------------------------------------------------------
void RoboCar::begin() {
    Wire.begin();
    _compassInit();
    delay(200);           // let the sensor settle before first read
    stop();
}

// ===============================================================
// PUBLIC MOTION API
// ===============================================================

void RoboCar::driveForward(uint8_t speed) {
    // Lock target heading on the first call after a stop or turn
    if (!_headingLocked) {
        _resetHeadingHold();
        _headingLocked = true;
    }
    _applyHeadingHold(speed);
}

void RoboCar::driveBackward(uint8_t speed) {
    // Backward driving does not use heading correction:
    // compass correction logic reverses meaning in reverse gear.
    // User can call this for fixed-time reversals.
    _headingLocked = false; // force re-lock next time driveForward is called
    _setAll(BACKWARD, speed);
}

void RoboCar::stop() {
    _headingLocked = false; // re-lock heading on next driveForward call
    _motorFL.run(RELEASE);
    _motorFR.run(RELEASE);
    _motorRL.run(RELEASE);
    _motorRR.run(RELEASE);
}

void RoboCar::turnLeft(float degrees) {
    float current = _getHeading();
    float target  = _wrapAngle(current - degrees);
    _turnToHeading(target, true);
    _headingLocked = false; // re-lock on next driveForward
}

void RoboCar::turnRight(float degrees) {
    float current = _getHeading();
    float target  = _wrapAngle(current + degrees);
    _turnToHeading(target, false);
    _headingLocked = false;
}

void RoboCar::pivotLeft(uint8_t speed) {
    // Raw freewheel pivot - right side drives, left freewheels
    _headingLocked = false;
    _motorFL.setSpeed(0);     _motorRL.setSpeed(0);
    _motorFR.setSpeed(speed); _motorRR.setSpeed(speed);

    _motorFL.run(RELEASE);  _motorRL.run(RELEASE);
    _motorFR.run(FORWARD);  _motorRR.run(FORWARD);
}

void RoboCar::pivotRight(uint8_t speed) {
    // Raw freewheel pivot - left side drives, right freewheels
    _headingLocked = false;
    _motorFL.setSpeed(speed); _motorRL.setSpeed(speed);
    _motorFR.setSpeed(0);     _motorRR.setSpeed(0);

    _motorFL.run(FORWARD);  _motorRL.run(FORWARD);
    _motorFR.run(RELEASE);  _motorRR.run(RELEASE);
}

void RoboCar::arcLeft(uint8_t outerSpeed, uint8_t innerSpeed) {
    _headingLocked = false;
    // Right side is the outer (faster) arc, robot curves left
    _motorFL.setSpeed(innerSpeed); _motorRL.setSpeed(innerSpeed);
    _motorFR.setSpeed(outerSpeed); _motorRR.setSpeed(outerSpeed);

    _motorFL.run(FORWARD); _motorRL.run(FORWARD);
    _motorFR.run(FORWARD); _motorRR.run(FORWARD);
}

void RoboCar::arcRight(uint8_t outerSpeed, uint8_t innerSpeed) {
    _headingLocked = false;
    // Left side is the outer (faster) arc, robot curves right
    _motorFL.setSpeed(outerSpeed); _motorRL.setSpeed(outerSpeed);
    _motorFR.setSpeed(innerSpeed); _motorRR.setSpeed(innerSpeed);

    _motorFL.run(FORWARD); _motorRL.run(FORWARD);
    _motorFR.run(FORWARD); _motorRR.run(FORWARD);
}

void RoboCar::setMotor(uint8_t motor, uint8_t direction, uint8_t speed) {
    AF_DCMotor* m = nullptr;
    switch (motor) {
        case 1: m = &_motorFL; break;
        case 2: m = &_motorFR; break;
        case 3: m = &_motorRL; break;
        case 4: m = &_motorRR; break;
        default: return;
    }
    m->setSpeed(speed);
    m->run(direction);
}

// ===============================================================
// PRIVATE - MOTOR HELPER
// ===============================================================

void RoboCar::_setAll(uint8_t direction, uint8_t speed) {
    _motorFL.setSpeed(speed); _motorFR.setSpeed(speed);
    _motorRL.setSpeed(speed); _motorRR.setSpeed(speed);

    _motorFL.run(direction);  _motorFR.run(direction);
    _motorRL.run(direction);  _motorRR.run(direction);
}

// ===============================================================
// PRIVATE - HEADING HOLD PID
// ===============================================================

void RoboCar::_resetHeadingHold() {
    _holdTarget    = _getHeading();
    _holdIntegral  = 0;
    _holdLastError = 0;
    _holdLastTime  = millis();
}

void RoboCar::_applyHeadingHold(uint8_t baseSpeed) {
    float currentHeading = _getHeading();
    float error = _angleDifference(_holdTarget, currentHeading);

    unsigned long now = millis();
    float dt = (now - _holdLastTime) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    _holdLastTime = now;

    _holdIntegral += error * dt;
    float derivative = (error - _holdLastError) / dt;
    _holdLastError = error;

    float correction = _KP * error + _KI * _holdIntegral + _KD * derivative;
    correction = constrain(correction, -(float)_MAX_TRIM, (float)_MAX_TRIM);

    // Positive error = drifted left of target, speed up left to veer right
    int leftSpeed  = constrain((int)baseSpeed + (int)correction, 0, 255);
    int rightSpeed = constrain((int)baseSpeed - (int)correction, 0, 255);

    _motorFL.setSpeed(leftSpeed);  _motorRL.setSpeed(leftSpeed);
    _motorFR.setSpeed(rightSpeed); _motorRR.setSpeed(rightSpeed);

    _motorFL.run(FORWARD); _motorRL.run(FORWARD);
    _motorFR.run(FORWARD); _motorRR.run(FORWARD);
}

// ===============================================================
// PRIVATE - TURN TO HEADING (blocks until done or timeout)
// ===============================================================

void RoboCar::_turnToHeading(float targetHeading, bool goLeft) {
    unsigned long startTime = millis();

    while (true) {
        float current = _getHeading();
        float error   = _angleDifference(targetHeading, current);

        if (abs(error) <= _TURN_TOLERANCE)     break;
        if (millis() - startTime > _TURN_TIMEOUT) break;

        if (goLeft) {
            // Full pivot left: rear wheels drive, front freewheel
            _setAll(RELEASE, 0);
            _motorRL.setSpeed((uint8_t)_TURN_SPEED);
            _motorRR.setSpeed((uint8_t)_TURN_SPEED);
            _motorRL.run(FORWARD);
            _motorRR.run(FORWARD);
        } else {
            // Full pivot right: front wheels drive, rear freewheel
            _setAll(RELEASE, 0);
            _motorFL.setSpeed((uint8_t)_TURN_SPEED);
            _motorFR.setSpeed((uint8_t)_TURN_SPEED);
            _motorFL.run(FORWARD);
            _motorFR.run(FORWARD);
        }

        delay(20); // poll compass every 20 ms
    }

    stop();
}

// ===============================================================
// PRIVATE - COMPASS (QMC5883L raw I2C)
// ===============================================================

void RoboCar::_compassInit() {
    _compassWriteReg(_REG_SET_RESET, 0x01);
    _compassWriteReg(_REG_CTRL1,     0x1D); // continuous, 200Hz, 8G, OSR512
    _compassWriteReg(_REG_CTRL2,     0x40); // pointer roll-over enabled
}

void RoboCar::_compassWriteReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_QMC_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

bool RoboCar::_compassRead(int16_t &x, int16_t &y, int16_t &z) {
    // Check data-ready bit in status register
    Wire.beginTransmission(_QMC_ADDR);
    Wire.write(_REG_STATUS);
    Wire.endTransmission();
    Wire.requestFrom(_QMC_ADDR, (uint8_t)1);
    if (Wire.available() < 1) return false;
    if (!(Wire.read() & 0x01)) return false; // DRDY bit not set

    // Read 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    Wire.beginTransmission(_QMC_ADDR);
    Wire.write(_REG_DATA_X_LSB);
    Wire.endTransmission();
    Wire.requestFrom(_QMC_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return false;

    x = (int16_t)(Wire.read() | Wire.read() << 8);
    y = (int16_t)(Wire.read() | Wire.read() << 8);
    z = (int16_t)(Wire.read() | Wire.read() << 8);
    return true;
}

float RoboCar::_getHeading() {
    int16_t x, y, z;
    if (_compassRead(x, y, z)) {
        _lastHeading = _computeHeading(x, y);
    }
    // If no new sample is ready, return the last known heading
    // so callers always get a usable value without blocking
    return _lastHeading;
}

float RoboCar::_computeHeading(int16_t x, int16_t y) {
    // Sensor mounted X=RIGHT, Y=BACKWARD
    // Negate Y to recover the car's true forward axis
    float heading = atan2((float)x, (float)(-y)) * 180.0f / PI;
    if (heading < 0) heading += 360.0f;
    return heading;
}

// ===============================================================
// PRIVATE - ANGLE MATH
// ===============================================================

float RoboCar::_wrapAngle(float angle) {
    while (angle <    0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// Shortest signed difference from current to target.
// Positive = target is clockwise (turn right).
// Negative = target is counter-clockwise (turn left).
float RoboCar::_angleDifference(float target, float current) {
    float diff = target - current;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}
