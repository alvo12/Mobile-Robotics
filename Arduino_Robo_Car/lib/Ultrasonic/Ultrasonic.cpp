/*
  Ultrasonic.cpp - Implementation of the Ultrasonic sensor driver
*/

#include "Ultrasonic.h"

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------
Ultrasonic::Ultrasonic(uint8_t trigPin, uint8_t echoPin)
    : _trig(trigPin), _echo(echoPin), _timeoutUs(ULTRASONIC_DEFAULT_TIMEOUT_US) {}

// ------------------------------------------------------------
// Public API
// ------------------------------------------------------------
void Ultrasonic::begin() {
    pinMode(_trig, OUTPUT);
    pinMode(_echo, INPUT);
    digitalWrite(_trig, LOW);
    delayMicroseconds(2); // Settle the line before first reading
}

void Ultrasonic::setTimeout(unsigned long timeoutUs) {
    _timeoutUs = timeoutUs;
}

long Ultrasonic::getDistanceCm() {
    long duration = _ping();
    if (duration == 0) return 0;
    // Integer math: 1 cm = 58 us round-trip at ~343 m/s
    return duration / 58;
}

long Ultrasonic::getDistanceMm() {
    long duration = _ping();
    if (duration == 0) return 0;
    // 1 mm = 5.8 us round-trip; multiply first to keep integer precision
    return (duration * 10) / 58;
}

long Ultrasonic::getDistanceInches() {
    long duration = _ping();
    if (duration == 0) return 0;
    // 1 inch = ~148 us round-trip
    return duration / 148;
}

bool Ultrasonic::isWithin(long thresholdCm) {
    long dist = getDistanceCm();
    if (dist == 0) return false; // No echo = no obstacle detected
    return dist < thresholdCm;
}

long Ultrasonic::getRawDuration() {
    return _ping();
}

// ------------------------------------------------------------
// Private
// ------------------------------------------------------------
long Ultrasonic::_ping() {
    // Ensure trig is low before pulse
    digitalWrite(_trig, LOW);
    delayMicroseconds(2);

    // Send 10 us HIGH pulse
    digitalWrite(_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig, LOW);

    // Measure echo pulse width; returns 0 on timeout
    return pulseIn(_echo, HIGH, _timeoutUs);
}
