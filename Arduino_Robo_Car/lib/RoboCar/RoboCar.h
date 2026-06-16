#ifndef ROBOCAR_H
#define ROBOCAR_H

/*
  RoboCar.h - Motor control library for a 4-wheel drive robot
              using the Adafruit Motor Shield v1, with built-in
              compass heading-hold and degree-precise turning.

  ---------------------------------------------------------------
  PUBLIC API - everything a user needs:
  ---------------------------------------------------------------

    RoboCar car;
    car.begin();

    // --- Straight driving (compass-corrected automatically) ---
    car.driveForward();          // locks heading on first call, holds it
    car.driveBackward(180);      // backward at fixed speed, no correction
    car.stop();

    // --- Degree-precise turns (compass-guided) ---
    car.turnLeft(90);            // rotate exactly 90 degrees left
    car.turnRight(45);           // rotate exactly 45 degrees right

    // --- Raw speed-based pivots (no compass, instant) ---
    car.pivotLeft(150);          // spin left at speed 150, no angle target
    car.pivotRight(150);         // spin right at speed 150, no angle target

    // --- Arc turns (both sides move, one slower) ---
    car.arcLeft(200, 100);       // curve left: outer 200, inner 100
    car.arcRight(200, 100);      // curve right: outer 200, inner 100

    // --- Per-motor access (advanced) ---
    car.setMotor(1, FORWARD, 200); // 1=FL, 2=FR, 3=RL, 4=RR

  ---------------------------------------------------------------
  Compass-related internals are entirely private.
  The user never calls anything magnetometer-related directly.
  ---------------------------------------------------------------

  Motor port mapping (Adafruit Shield v1):
    M1 -> Front Left   (FL)
    M2 -> Front Right  (FR)
    M3 -> Rear Left    (RL)
    M4 -> Rear Right   (RR)

  Magnetometer: GY-271 / QMC5883L over I2C (SDA=A4, SCL=A5)
    Mount orientation: X pointing RIGHT, Y pointing BACKWARD
*/

#include <Arduino.h>
#include <AFMotor.h>
#include <Wire.h>

class RoboCar {
public:
    // Constructor
    RoboCar();

    // Call once in setup() - initialises motors, compass, and I2C
    void begin();

    // ---------------------------------------------------------------
    // PUBLIC MOTION API
    // ---------------------------------------------------------------

    // Drives forward, automatically correcting drift using compass PID.
    // Locks the target heading on the first call after a stop/turn.
    // speed: 0-255
    void driveForward(uint8_t speed = 180);

    // Drives backward at a fixed speed (no heading correction applied).
    // speed: 0-255
    void driveBackward(uint8_t speed = 180);

    // Stops all motors (coast / freewheel)
    void stop();

    // Rotates left by exactly `degrees` using the compass to know when done.
    // Blocks until the heading is reached or timeout expires.
    void turnLeft(float degrees);

    // Rotates right by exactly `degrees` using the compass to know when done.
    // Blocks until the heading is reached or timeout expires.
    void turnRight(float degrees);

    // Raw speed-based pivot left (no compass, no angle target).
    // Useful for timed turns or when compass data is unreliable.
    // speed: 0-255
    void pivotLeft(uint8_t speed);

    // Raw speed-based pivot right (no compass, no angle target).
    // speed: 0-255
    void pivotRight(uint8_t speed);

    // Arc turns: both sides move forward, one faster than the other.
    // outerSpeed: faster side (creates the curve direction)
    // innerSpeed: slower side
    void arcLeft(uint8_t outerSpeed, uint8_t innerSpeed);
    void arcRight(uint8_t outerSpeed, uint8_t innerSpeed);

    // Per-motor control for advanced users.
    // motor: 1=FL, 2=FR, 3=RL, 4=RR
    // direction: FORWARD, BACKWARD, RELEASE (AFMotor constants)
    void setMotor(uint8_t motor, uint8_t direction, uint8_t speed);

private:
    // ---------------------------------------------------------------
    // MOTORS
    // ---------------------------------------------------------------
    AF_DCMotor _motorFL;
    AF_DCMotor _motorFR;
    AF_DCMotor _motorRL;
    AF_DCMotor _motorRR;

    void _setAll(uint8_t direction, uint8_t speed);

    // ---------------------------------------------------------------
    // COMPASS - QMC5883L / GY-271 (raw I2C, no external library)
    // ---------------------------------------------------------------

    // I2C address and register map
    static const uint8_t _QMC_ADDR       = 0x0D;
    static const uint8_t _REG_DATA_X_LSB = 0x00;
    static const uint8_t _REG_STATUS     = 0x06;
    static const uint8_t _REG_CTRL1      = 0x09;
    static const uint8_t _REG_CTRL2      = 0x0A;
    static const uint8_t _REG_SET_RESET  = 0x0B;

    void     _compassInit();
    void     _compassWriteReg(uint8_t reg, uint8_t value);
    bool     _compassRead(int16_t &x, int16_t &y, int16_t &z);
    float    _getHeading();           // returns 0-360, cached if no new sample
    float    _computeHeading(int16_t x, int16_t y);

    float    _lastHeading;            // cached heading between samples

    // ---------------------------------------------------------------
    // HEADING-HOLD PID (used by driveForward)
    // ---------------------------------------------------------------
    static const float    _KP          ;  // proportional gain
    static const float    _KI          ;  // integral gain
    static const float    _KD          ;  // derivative gain
    static const uint8_t  _MAX_TRIM    ;  // max speed correction per side

    float         _holdTarget;            // locked heading to maintain
    float         _holdIntegral;
    float         _holdLastError;
    unsigned long _holdLastTime;
    bool          _headingLocked;         // false until first driveForward call

    void  _resetHeadingHold();
    void  _applyHeadingHold(uint8_t baseSpeed);

    // ---------------------------------------------------------------
    // TURN-TO-HEADING (used by turnLeft / turnRight)
    // ---------------------------------------------------------------
    static const float         _TURN_SPEED    ;  // motor speed during compass turns
    static const float         _TURN_TOLERANCE;  // degrees - close enough to stop
    static const unsigned long _TURN_TIMEOUT  ;  // ms safety cap

    void  _turnToHeading(float targetHeading, bool goLeft);

    // ---------------------------------------------------------------
    // ANGLE MATH HELPERS
    // ---------------------------------------------------------------
    float _wrapAngle(float angle);
    float _angleDifference(float target, float current);
};

#endif // ROBOCAR_H
