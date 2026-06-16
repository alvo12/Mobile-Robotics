#include <Arduino.h>
#include <Servo.h>     // Force the compiler to load this first!
#include "RoboCar.h"

RoboCar myCar;

void setup() {
  Serial.begin(9600);
  myCar.begin(); 
}

void loop() {
  long distance = myCar.getDistance();

  if (distance > 0 && distance < 20) {
    myCar.stop();
    delay(200);
    
    myCar.look(160); // Look Left
    delay(500);
    myCar.look(20);  // Look Right
    delay(500);
    
    myCar.look(90);  // Center
    myCar.moveBackward(150);
    delay(500);
    myCar.turnRight(180);
    delay(500);
  } else {
    myCar.moveForward(180);
  }
}