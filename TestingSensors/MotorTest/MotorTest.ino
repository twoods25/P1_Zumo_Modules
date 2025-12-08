#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;

void setup() {
  Serial.begin(9600);
}

void loop() {
  motors.setSpeeds(25, 25);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
  motors.setSpeeds(50, 50);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
  motors.setSpeeds(75, 75);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
  motors.setSpeeds(100, 100);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
  motors.setSpeeds(150, 150);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
  motors.setSpeeds(200, 200);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
  motors.setSpeeds(250, 250);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
  motors.setSpeeds(300, 300);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
  motors.setSpeeds(350, 350);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
  motors.setSpeeds(400, 400);
  delay(1000);
  motors.setSpeeds(0, 0);
  delay(10000);
}