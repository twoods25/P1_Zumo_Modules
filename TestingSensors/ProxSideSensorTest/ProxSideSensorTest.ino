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
Zumo32U4ProximitySensors prox;
Zumo32U4ButtonA buttonA;

void setup() {
  Serial.begin(9600);
  prox.initThreeSensors();
}

void loop() {
  // Read the front proximity sensor and gets its left value (the
  // amount of reflectance detected while using the left LEDs)
  // and right value.
  prox.read();
  uint8_t leftValue = prox.countsFrontWithLeftLeds();
  uint8_t rightValue = prox.countsFrontWithRightLeds();

Serial.print("Left: ");
  Serial.println(leftValue);
  Serial.print("Right: ");
  Serial.println(rightValue);
  Serial.print("Distance: ");
  Serial.println(leftValue + rightValue);
  delay(400);
}
