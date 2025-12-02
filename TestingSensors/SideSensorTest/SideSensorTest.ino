/* This example uses the front proximity sensor on the Zumo 32U4
Front Sensor Array to locate an opponent robot or any other
reflective object. Using the motors to turn, it scans its
surroundings. If it senses an object, it turns on its yellow LED
and attempts to face towards that object. */

#include <Wire.h>
#include <Zumo32U4.h>

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4OLED display;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4ButtonA buttonA;

void setup() {
  Serial.begin(9600);
  proxSensors.initFrontSensor();

  // Wait for the user to press A before driving the motors.
  display.clear();
  display.print(F("Press A"));
  buttonA.waitForButton();
  display.clear();
}

void loop() {
  // Read the front proximity sensor and gets its left value (the
  // amount of reflectance detected while using the left LEDs)
  // and right value.
  proxSensors.read();
  uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
  uint8_t rightValue = proxSensors.countsFrontWithRightLeds();

Serial.print("Left: ");
  Serial.println(leftValue);
  Serial.print("Right: ");
  Serial.println(rightValue);
  delay(400);
}
