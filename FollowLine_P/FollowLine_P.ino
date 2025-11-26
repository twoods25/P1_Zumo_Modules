#include <Wire.h>     // inkluderer bibliotek
#include <Zumo32U4.h> // inkluderer bibliotek

Zumo32U4Motors motors;           // inkluderer motorer
Zumo32U4LineSensors lineSensors; // inkluderer sensorer
Zumo32U4OLED OLED;               // inkluderer OLED-skærm
Zumo32U4ButtonA buttonA;         // inkluderer knap

#define NUM_SENSORS 3                         // definerer antal sensorer
unsigned short lineSensorValues[NUM_SENSORS]; // gemmer målingerne i en array

unsigned char speed = 100; // sætter hastighed (unsigned short = 0 til 65535)

unsigned long lastPrintTime = 0;   // tid for sidste print
unsigned long printInterval = 500; // print hvert 500 ms

void printFast()
{
  Serial.println(lineSensors.readLine(lineSensorValues) - 1000);
  Serial.println("Individual Sensor Values:");
  Serial.println(lineSensorValues[0]);
  Serial.println(lineSensorValues[1]);
  Serial.println(lineSensorValues[2]);
  delay(300);
}

void printSlow()
{
  unsigned long now = millis();
  if (now - lastPrintTime >= printInterval)
  {
    lastPrintTime = now;
    Serial.println(lineSensors.readLine(lineSensorValues) - 1000);
  }
}

void FollowLine()
{
  short position = lineSensors.readLine(lineSensorValues);
  // gør internt lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  // dvs. læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS] og sætter en position dvs. 0 1000 2000.
  int error = position - 1000;
  if (position == -1000 || position == 1000)
  {
    error = 0;
  }
  /*else if (error < 20)
  else if (error < -20)*/
  float Kp = 0.3;
  float Up = Kp * error;
  int leftMotorSpeed = constrain(speed + Up, 0, 100);
  int rightMotorSpeed = constrain(speed - Up, 0, 100); //Logic?
  motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
}

void setup()
{
  lineSensors.initThreeSensors();
  Serial.begin(9600);
  while (!buttonA.getSingleDebouncedPress())
  {
    lineSensors.calibrate();
    // gemmer min- og max-værdier for hver sensor fra arrayet [NUM_SENSORS] i
    // lineSensors.calibratedMinimum[NUM_SENSORS] og lineSensors.calibratedMaximum[NUM_SENSORS]
    printFast();
  }
  lineSensors.readCalibrated(lineSensorValues);
  // omregner min- og max-værdierne, således at de henholdsvis bliver 0 og 2000
}

void loop()
{
  FollowLine();
  printSlow();
  if (lineSensorValues[1] < 100)
  {
    printFast();
    motors.setSpeeds(0, 0);
    delay(1000);

  }
}
