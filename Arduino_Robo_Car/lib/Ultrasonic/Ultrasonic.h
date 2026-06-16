#ifndef ULTRASONIC_H
#define ULTRASONIC_H

/*
  Ultrasonic.h - Driver for HC-SR04 and compatible ultrasonic sensors.

  Usage:
    Ultrasonic sonar(7, 6);       // trig pin, echo pin - user chooses
    sonar.begin();

    long cm  = sonar.getDistanceCm();
    long mm  = sonar.getDistanceMm();
    long in  = sonar.getDistanceInches();
    bool hit = sonar.isWithin(20); // true if object closer than 20 cm

  Timeout:
    By default, getDistance returns 0 if no echo is received within
    ULTRASONIC_DEFAULT_TIMEOUT_US microseconds (equivalent to ~4 m).
    Override with sonar.setTimeout(us).
*/

#include <Arduino.h>

// Default pulse timeout in microseconds (~4 m range at 343 m/s)
#define ULTRASONIC_DEFAULT_TIMEOUT_US 23200UL

class Ultrasonic {
public:
    // trig: OUTPUT pin connected to sensor TRIG
    // echo: INPUT  pin connected to sensor ECHO
    Ultrasonic(uint8_t trigPin, uint8_t echoPin);

    // Call once in setup()
    void begin();

    // Override the echo wait timeout (microseconds)
    void setTimeout(unsigned long timeoutUs);

    // ---- Distance readings ----
    // Returns 0 if no echo received within timeout
    long getDistanceCm();
    long getDistanceMm();
    long getDistanceInches();

    // ---- Convenience ----
    // Returns true if an object is detected closer than thresholdCm
    bool isWithin(long thresholdCm);

    // Raw pulse duration in microseconds (for advanced users)
    long getRawDuration();

private:
    uint8_t       _trig;
    uint8_t       _echo;
    unsigned long _timeoutUs;

    // Fires a 10us trigger pulse and returns echo duration in us
    long _ping();
};

#endif // ULTRASONIC_H
