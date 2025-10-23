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
void loop() {
  static int lastLeft = 0;
  static int lastRight = 0;

  // Get current encoder counts
  int leftNow = encoders.getCountsLeft();
  int rightNow = encoders.getCountsRight();

  // Calculate how much each wheel has moved since last loop
  int leftDelta = leftNow - lastLeft;
  int rightDelta = rightNow - lastRight;

  // Update previous counts
  lastLeft = leftNow;
  lastRight = rightNow;

  // Calculate correction (difference between wheel speeds)
  int correction = leftDelta - rightDelta;

  // Base speed
  int baseSpeed = 400;  // Adjust as needed

  // Apply proportional correction
  float kP = 60;  // Proportional gain (tune this value)
  int leftSpeed = baseSpeed - correction * kP;
  int rightSpeed = baseSpeed + correction * kP;

  // Constrain to safe limits
  leftSpeed = constrain(leftSpeed, 0, 400);
  rightSpeed = constrain(rightSpeed, 0, 400);

  // Apply speeds
  motors.setSpeeds(leftSpeed, rightSpeed);

  delay(5);
}
