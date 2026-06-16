/*
  ScanServo.cpp - Implementation of the ScanServo library
*/

#include "ScanServo.h"

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------
ScanServo::ScanServo(uint8_t pin)
    : _pin(pin), _currentAngle(90) {}

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------
void ScanServo::begin() {
    attach();
    lookAt(90); // Centre on boot
}

void ScanServo::attach() {
    _servo.attach(_pin);
}

void ScanServo::detach() {
    _servo.detach();
}

// ------------------------------------------------------------
// Position control
// ------------------------------------------------------------
void ScanServo::lookAt(int angle, unsigned int settleMs) {
    _currentAngle = _clamp(angle);
    _servo.write(_currentAngle);
    delay(settleMs);
}

int ScanServo::getAngle() const {
    return _currentAngle;
}

// ------------------------------------------------------------
// Sweep utilities
// ------------------------------------------------------------
void ScanServo::sweep(int startAngle, int endAngle,
                      int stepDeg, unsigned int stepDelayMs) {
    // Clamp boundaries
    startAngle = _clamp(startAngle);
    endAngle   = _clamp(endAngle);
    stepDeg    = max(1, stepDeg); // Guard against zero/negative step

    if (startAngle <= endAngle) {
        // Forward sweep
        for (int a = startAngle; a <= endAngle; a += stepDeg) {
            lookAt(a, stepDelayMs);
        }
    } else {
        // Reverse sweep
        for (int a = startAngle; a >= endAngle; a -= stepDeg) {
            lookAt(a, stepDelayMs);
        }
    }
}

void ScanServo::sweepWithCallback(int startAngle, int endAngle,
                                  int stepDeg, unsigned int stepDelayMs,
                                  void (*callback)(int angle)) {
    startAngle = _clamp(startAngle);
    endAngle   = _clamp(endAngle);
    stepDeg    = max(1, stepDeg);

    if (startAngle <= endAngle) {
        for (int a = startAngle; a <= endAngle; a += stepDeg) {
            lookAt(a, stepDelayMs);
            if (callback) callback(a); // Fire user function at each step
        }
    } else {
        for (int a = startAngle; a >= endAngle; a -= stepDeg) {
            lookAt(a, stepDelayMs);
            if (callback) callback(a);
        }
    }
}

// ------------------------------------------------------------
// Private
// ------------------------------------------------------------
int ScanServo::_clamp(int angle) const {
    return constrain(angle, 0, 180);
}
