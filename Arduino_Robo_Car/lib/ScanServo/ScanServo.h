#ifndef SCANSERVO_H
#define SCANSERVO_H

/*
  ScanServo.h - Driver for a servo motor mounted on the Adafruit
                Motor Shield v1 servo header (fixed pin).

  The Adafruit Shield v1 exposes two dedicated servo headers
  hardwired to digital pins 9 and 10. This library defaults to
  pin 9 but the pin can be overridden in the constructor if needed.

  Usage:
    ScanServo scanner;            // default pin 9
    ScanServo scanner(10);        // override to pin 10
    scanner.begin();

    scanner.lookAt(90);           // point to angle (0-180)
    scanner.lookAt(90, 300);      // point with explicit settle delay (ms)
    int pos = scanner.getAngle(); // read current angle

    // Sweep from 30 to 150 in steps of 5, pausing 50ms each step:
    scanner.sweep(30, 150, 5, 50);

    // Sweep and call your function at each step:
    scanner.sweepWithCallback(0, 180, 10, 100, myCallback);
    // where myCallback signature is: void myCallback(int angle)
*/

#include <Arduino.h>
#include <Servo.h>

// Default shield servo pin
#define SCANSERVO_DEFAULT_PIN 9

// Minimum settle time after a move (ms) - prevents servo chatter
#define SCANSERVO_MIN_SETTLE_MS 15

class ScanServo {
public:
    // pin: the signal pin the servo is attached to
    explicit ScanServo(uint8_t pin = SCANSERVO_DEFAULT_PIN);

    // Call once in setup()
    void begin();

    // ---- Position control ----
    // Moves to angle (0-180 degrees)
    // settleMs: how long to wait after moving (default SCANSERVO_MIN_SETTLE_MS)
    void lookAt(int angle, unsigned int settleMs = SCANSERVO_MIN_SETTLE_MS);

    // Convenience shortcuts
    void lookLeft()   { lookAt(0);   }
    void lookCenter() { lookAt(90);  }
    void lookRight()  { lookAt(180); }

    // Returns the last commanded angle
    int getAngle() const;

    // Detach the servo (stops PWM signal, reduces power draw when idle)
    void detach();

    // Re-attach after detach (needed before moving again)
    void attach();

    // ---- Sweep utilities ----
    // Sweeps from startAngle to endAngle in stepDeg increments,
    // pausing stepDelayMs milliseconds at each position.
    // Works in both directions (start > end sweeps backward).
    void sweep(int startAngle, int endAngle,
               int stepDeg = 5, unsigned int stepDelayMs = 50);

    // Same as sweep() but calls a user-supplied callback at each step.
    // Callback receives the current angle so you can take a sensor reading.
    // Signature: void myCallback(int angle)
    void sweepWithCallback(int startAngle, int endAngle,
                           int stepDeg, unsigned int stepDelayMs,
                           void (*callback)(int angle));

private:
    Servo    _servo;
    uint8_t  _pin;
    int      _currentAngle;

    // Clamp angle to safe 0-180 range
    int _clamp(int angle) const;
};

#endif // SCANSERVO_H
