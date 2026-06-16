#ifndef ROBOCAR_H
#define ROBOCAR_H

/*
  RoboCar.h - Motor control library for a 4-wheel drive robot
              using the Adafruit Motor Shield v1.

  Usage:
    RoboCar car;
    car.begin();
    car.moveForward(200);
    car.turnLeft(150);
    car.stop();

  Motor port mapping (Adafruit Shield v1):
    M1 -> Front Left
    M2 -> Front Right
    M3 -> Rear Left
    M4 -> Rear Right
*/

#include <Arduino.h>
#include <AFMotor.h>

class RoboCar {
public:
    // Constructor - initialises motors on their fixed shield ports
    RoboCar();

    // Call once in setup()
    void begin();

    // ---- Basic motion ----
    // speed: 0-255
    void moveForward(uint8_t speed);
    void moveBackward(uint8_t speed);
    void stop();

    // ---- Turns (tank-style pivot) ----
    // Left wheels reverse, right wheels forward (and vice versa)
    // speed: 0-255
    void turnLeft(uint8_t speed);
    void turnRight(uint8_t speed);

    // ---- Assisted turns (arc, one side slower) ----
    // outerSpeed: faster side, innerSpeed: slower side
    void arcLeft(uint8_t outerSpeed, uint8_t innerSpeed);
    void arcRight(uint8_t outerSpeed, uint8_t innerSpeed);

    // ---- Individual motor access (advanced users) ----
    // motor: 1=FL, 2=FR, 3=RL, 4=RR
    // direction: FORWARD, BACKWARD, RELEASE  (AFMotor constants)
    void setMotor(uint8_t motor, uint8_t direction, uint8_t speed);

private:
    AF_DCMotor _motorFL;
    AF_DCMotor _motorFR;
    AF_DCMotor _motorRL;
    AF_DCMotor _motorRR;

    // Internal helper - apply same speed and direction to all four motors
    void _setAll(uint8_t direction, uint8_t speed);
};

#endif // ROBOCAR_H
