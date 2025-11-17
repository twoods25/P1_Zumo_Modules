#include <Wire.h>      // inkluderer bibliotek
#include <Zumo32U4.h>  // inkluderer bibliotek

Zumo32U4Motors motors;            // inkluderer motorer
Zumo32U4LineSensors lineSensors;  // inkluderer sensorer
Zumo32U4OLED OLED;                // inkluderer OLED-skærm

#define NUM_SENSORS 3                          // definerer antal sensorer
unsigned short lineSensorValues[NUM_SENSORS];  // gemmer målingerne i en array

unsigned char speed = 100;  // sætter hastighed (unsigned short = 0 til 65535)

unsigned short hvid = 500;

unsigned char a, b, c, d, e, f;

void forward() {
  motors.setSpeeds(speed, speed);  // kør med defineret hastighed
}

void backwards() {
  motors.setSpeeds(-speed, -speed);  // kør baglæns med defineret hastighed
}

void turnLeft() {
  motors.setSpeeds(-speed, speed);  // drej til venstre med defineret hastighed
}

void turnRight() {
  motors.setSpeeds(speed, -speed);  // drej til højre med defineret hastighed
}

void stop() {
  motors.setSpeeds(0, 0);  // stop
}

void calibrateSensors() {
  lineSensors.calibrate();  // gemmer min-og max-værdier for hver sensor fra arrayet [NUM_SENSORS] i lineSensors.calibratedMinimum[NUM_SENSORS] og lineSensors.calibratedMaximum[NUM_SENSORS]
}

void normalize() {
  a = lineSensors.calibratedMinimumOn[0] / 2000.0 * 100;  // deler med 2000.0 som en float eller double - ikke int (heltal)
  b = lineSensors.calibratedMaximumOn[0] / 2000.0 * 100;
  c = lineSensors.calibratedMinimumOn[1] / 2000.0 * 100;
  d = lineSensors.calibratedMaximumOn[1] / 2000.0 * 100;
  e = lineSensors.calibratedMinimumOn[2] / 2000.0 * 100;
  f = lineSensors.calibratedMaximumOn[2] / 2000.0 * 100;
}

void printReadingsToSerial() {
  char buffer[120];                                                                                      // opretter tegn-array med plads til 80 tegn
  sprintf(buffer,                                                                                        // i stedet for Serial.print(), da sprintf() er pæn og i en streng
          "%4d %4d | %4d %4d | %4d %4d     %4d | %4d | %4d     %3d %3d | %3d %3d | %3d %3d     %4d \n",  // %d = udskriv et helt tal, 4 = mindst fire karakterer (da 2000 er max), \n = linjeskift
          lineSensors.calibratedMinimumOn[0],
          lineSensors.calibratedMaximumOn[0],
          lineSensors.calibratedMinimumOn[1],
          lineSensors.calibratedMaximumOn[1],
          lineSensors.calibratedMinimumOn[2],
          lineSensors.calibratedMaximumOn[2],
          lineSensorValues[0],
          lineSensorValues[1],
          lineSensorValues[2],
          a, b, c, d, e, f,
          lineSensors.readLine(lineSensorValues));
  Serial.print(buffer);
}

void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  // læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS]
}

void FollowLine() {
 unsigned short position = lineSensors.readLine(lineSensorValues);
 int error = position - 1000;
 if (position == 0 || position == 2000) {
  error = 0;
 }
 float Kp = 0.3;
 float Up = Kp * error;
 int leftMotorSpeed  = constrain(speed + Up, 0, 100);
 int rightMotorSpeed = constrain(speed - Up, 0, 100);
 motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
}

void setup() {
  lineSensors.initThreeSensors();  // initialiserer de tre sensorer
  Serial.begin(9600);
}

void loop() {
  calibrateSensors();
  normalize();
  printReadingsToSerial();
  FollowLine();
}
