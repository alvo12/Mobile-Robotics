#include <Arduino.h>
#include "RoboCar.h"
#include "Ultrasonic.h"

RoboCar    car;
Ultrasonic sonar(7, 6);   // trig, echo - user picks pins

void setup() {
    car.begin();
    sonar.begin();
}

void loop() {
    long distance = sonar.getDistanceCm();

    if (distance > 0 && distance <= 25) {
        car.stop();
        delay(200);
        car.driveBackward();
        delay(500);
        car.turnRight(90);
    } else {
        car.driveForward();
    }

    delay(20);
}