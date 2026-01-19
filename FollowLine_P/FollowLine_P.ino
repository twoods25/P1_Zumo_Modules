#include <Wire.h>     // inkluderer bibliotek
#include <Zumo32U4.h> // inkluderer bibliotek

Zumo32U4Motors motors;           // inkluderer motorer
Zumo32U4LineSensors lineSensors; // inkluderer sensorer
Zumo32U4OLED OLED;               // inkluderer OLED-skærm
Zumo32U4ButtonA buttonA;         // inkluderer knap

#define NUM_SENSORS 3                         // definerer antal sensorer
unsigned short lineSensorValues[NUM_SENSORS]; // gemmer målingerne i en array

short speed = 100;  // sætter hastighed (unsigned short = 0 til 65535)

unsigned long lastPrintTime = 0;   // tid for sidste print
unsigned long printInterval = 500; // print hvert 500 ms

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
  lineSensors.calibrate();  
  // gemmer min- og max-værdier for hver sensor fra arrayet [NUM_SENSORS] i 
  // lineSensors.calibratedMinimum[NUM_SENSORS] og lineSensors.calibratedMaximum[NUM_SENSORS]
}

void printFast() {
  Serial.println(1000 - lineSensors.readLine(lineSensorValues));
}

/*void printSlow() {
  unsigned long now = millis(); // get the current time in milliseconds since the program started
  if (now - lastPrintTime >= printInterval) { // check if enough time has passed since the last print (based on printInterval)
    lastPrintTime = now; // update the last print time to the current time
    Serial.print("Position: ");
    Serial.println(lineSensors.readLine(lineSensorValues) - 1000);
  }
}*/

void FollowLine() {
  short position = lineSensors.readLine(lineSensorValues);
  // gør internt lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  // dvs. læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS] og sætter en position dvs. 0 1000 2000.
  // her er 0 = lineSensors.calibratedMinimumOn[NUM_SENSORS] og 2000 = lineSensors.calibratedMaximum[NUM_SENSORS].
  int error = 1000 - position;
  if (position == -1000 || position == 1000) {
    error = 0;
  }
  float Kp = 0.3;
  float Up = Kp * error; // Up fortæller, hvor meget Zumoen skal dreje i forhold til dens nuværende hastighed (styresignalet)
  float leftMotorSpeed  = speed - Up;
  float rightMotorSpeed = speed + Up;
  motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
}

void setup() {
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
  // omregner min- og max-værdierne og kalibrerer, således at de henholdsvis bliver 0 og 1000
}

void loop() {
  FollowLine();
  delay(200);
}

