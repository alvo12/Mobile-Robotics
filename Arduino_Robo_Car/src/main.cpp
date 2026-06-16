/*
  main.cpp - Example sketch using RoboCar, Ultrasonic, and ScanServo

  The compass, PID, and heading math are entirely hidden inside RoboCar.
  From here, the user just calls turnLeft(90) and the library handles the rest.

  Wiring:
    Motors       -> Adafruit Motor Shield M1-M4 (fixed)
    Servo        -> Shield servo header pin 9   (fixed by shield)
    Ultrasonic   -> Trig: A0, Echo: A3          (user declares below)
    Magnetometer -> GY-271 SDA: A4, SCL: A5     (standard I2C, auto)
*/

#include <Arduino.h>
#include "RoboCar.h"
#include "Ultrasonic.h"
#include "ScanServo.h"

// ---- Pin declarations ----
#define TRIG_PIN A0
#define ECHO_PIN A3

// ---- Objects ----
RoboCar    car;
Ultrasonic sonar(TRIG_PIN, ECHO_PIN);
ScanServo  scanner;

// ---- Thresholds ----
#define STOP_DISTANCE_CM 20
#define BACKUP_TIME_MS   300

void setup() {
    Serial.begin(9600);
    car.begin();      // starts motors + compass internally
    sonar.begin();
    scanner.begin();
    Serial.println("RoboCar ready.");
}

void loop() {
    long dist = sonar.getDistanceCm();

    if (dist == 0 || dist > STOP_DISTANCE_CM) {
        // Clear path - drive forward with automatic heading correction
        car.driveForward();

    } else {
        // Obstacle detected
        car.stop();
        delay(150);

        // Back off
        car.driveBackward(160);
        delay(BACKUP_TIME_MS);
        car.stop();
        delay(150);

        // Scan both sides to find the clearer direction
        scanner.lookAt(160);
        delay(400);
        long distLeft = sonar.getDistanceCm();

        scanner.lookAt(20);
        delay(600);
        long distRight = sonar.getDistanceCm();

        scanner.lookCenter();
        delay(300);

        // Turn exactly 90 degrees toward the clearer side
        // The compass inside RoboCar handles the actual rotation
        if (distLeft > distRight) {
            car.turnLeft(90);
        } else {
            car.turnRight(90);
        }
    }

    delay(20);
}