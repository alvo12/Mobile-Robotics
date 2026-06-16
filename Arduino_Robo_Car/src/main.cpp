/*
  main.cpp - Example sketch using RoboCar, Ultrasonic, and ScanServo libraries

  Wiring:
    Motors     -> Adafruit Motor Shield M1-M4 (fixed)
    Servo      -> Shield servo header pin 9   (fixed by shield)
    Ultrasonic -> Any free digital pins, declared below
*/

#include <Arduino.h>
#include "RoboCar.h"
#include "Ultrasonic.h"
#include "ScanServo.h"
// ---- Pin declarations (user configures these) ----
#define TRIG_PIN 7
#define ECHO_PIN 6

// ---- Object instantiation ----
RoboCar    car;
Ultrasonic sonar(TRIG_PIN, ECHO_PIN);  // user picks pins here
ScanServo  scanner;                    // defaults to shield pin 9

// ---- Obstacle threshold ----
#define STOP_DISTANCE_CM 20

// ---- Callback example: read distance at each servo step ----
void onSweepStep(int angle) {
    long dist = sonar.getDistanceCm();
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" deg  |  Distance: ");
    Serial.print(dist);
    Serial.println(" cm");
}

void setup() {
    Serial.begin(9600);

    car.begin();
    sonar.begin();
    scanner.begin();

    Serial.println("RoboCar ready.");
}

void loop() {
    long dist = sonar.getDistanceCm();

    if (dist == 0) {
        // No echo - sensor out of range, keep moving
        car.moveForward(180);

    } else if (sonar.isWithin(STOP_DISTANCE_CM)) {
        // Obstacle close - stop and scan to find clear path
        car.stop();
        delay(200);

        // Sweep servo and log distances at each step
        scanner.sweepWithCallback(0, 180, 15, 100, onSweepStep);

        // Return to center and reverse briefly before deciding turn
        scanner.lookCenter();
        car.moveBackward(150);
        delay(500);
        car.stop();

        // Simple decision: turn right and resume
        car.turnRight(160);
        delay(400);
        car.stop();

    } else {
        car.moveForward(180);
    }

    delay(50); // Loop cadence
}