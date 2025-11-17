#include <Zumo32U4.h>

// Devices
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED display;
Zumo32U4Buzzer buzzer;

void setup() {
  // Reset encoders
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  // Wait 
  display.clear();
  display.gotoXY(0, 0);
  display.print(F("Press A to start"));

  while (!buttonA.isPressed()) {}
  buttonA.waitForRelease();

  // Countdown
  for (int i = 3; i > 0; i--) {
    display.clear();
    display.gotoXY(0, 0);
    display.print(F("Starting in:"));
    display.gotoXY(0, 1);
    display.print(i);
    
    buzzer.playNote(NOTE_C(5), 200, 15);  
    delay(1000); // wait 1 second
  }

  display.clear();
  display.gotoXY(0, 0);
  display.print(F("Go!"));
  buzzer.playNote(NOTE_G(5), 400, 15);
  delay(500);
  display.clear();
  display.print(F("Driving..."));
}

void StraightDrive() {
  static int lastLeft = 0;
  static int lastRight = 0;

  int leftNow = encoders.getCountsLeft();
  int rightNow = encoders.getCountsRight();

  int leftDelta = leftNow - lastLeft;
  int rightDelta = rightNow - lastRight;

  lastLeft = leftNow;
  lastRight = rightNow;

  int correction = leftDelta - rightDelta;

  int baseSpeed = 200; //int baseSpeed = desiredSpeed;
  float kP = 1; // float kP = 60*(desiredSpeed/400);

  int leftSpeed = baseSpeed - correction * kP;
  int rightSpeed = baseSpeed + correction * kP;

  leftSpeed = constrain(leftSpeed, 0, 400);
  rightSpeed = constrain(rightSpeed, 0, 400);

  motors.setSpeeds(leftSpeed, rightSpeed);

  
}

void loop() {
  StraightDrive();
}
