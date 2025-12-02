/* This example uses the line sensors on the Zumo 32U4 to follow
a black line on a white background, using a PID-based algorithm.
It works decently on courses with smooth, 6" radius curves and
has been tested with Zumos using 75:1 HP motors.  Modifications
might be required for it to work well on different courses or
with different motors.

This demo requires a Zumo 32U4 Front Sensor Array to be
connected, and jumpers on the front sensor array must be
installed in order to connect pin 4 to DN4 and pin 20 to DN2. */

#include <Wire.h>
#include <Zumo32U4.h>

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const uint16_t maxSpeed = 400;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4OLED display;

int16_t lastError = 0;

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

void setup() {
  // Uncomment if necessary to correct motor directions:
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
Serial.begin(9600);
  lineSensors.initThreeSensors();

  // Wait for button A to be pressed and released.
  display.clear();
  display.print(F("Press A"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));
  buttonA.waitForButton();
}

void loop() {
  // Get the position of the line.  Note that we *must* provide
  // the "lineSensorValues" argument to readLine() here, even
  // though we are not interested in the individual sensor
  // readings.
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  for (int i = 0; i < 3; i++) {
    Serial.println(lineSensorValues[i]);
  }
  delay(400);
}
