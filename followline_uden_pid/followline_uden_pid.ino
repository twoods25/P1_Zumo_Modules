#include <Wire.h>      // inkluderer bibliotek
#include <Zumo32U4.h>  // inkluderer bibliotek

Zumo32U4Motors motors;            // inkluderer motorer
Zumo32U4LineSensors lineSensors;  // inkluderer
Zumo32U4OLED OLED;                // inkluderer OLED-skærm
Zumo32U4ButtonA buttonA;          // inkluderer knap

#define NUM_SENSORS 3                         // definerer antal sensorer
unsigned short lineSensorValues[NUM_SENSORS];  // gemmer målingerne i en array

unsigned char speed = 100;  // sætter hastighed (unsigned short = 0 til 65535)

unsigned long lastPrintTime = 0;
unsigned long printInterval = 500;

void forward() {
  motors.setSpeeds(speed, speed);  // kør med defineret hastighed
  Serial.print("OutputL: ");
  Serial.println(speed);
  Serial.print("OutputR: ");
  Serial.println(speed);
}

void backwards() {
  motors.setSpeeds(-speed, -speed);  // kør baglæns med defineret hastighed
  Serial.print("OutputL: ");
  Serial.println(-speed);
  Serial.print("OutputR: ");
  Serial.println(-speed);
}

void turnRight() {
  motors.setSpeeds(-speed, speed);  // drej til venstre med defineret hastighed
  Serial.print("OutputL: ");
  Serial.println(-speed);
  Serial.print("OutputR: ");
  Serial.println(speed);
}

void turnLeft() {
  motors.setSpeeds(speed, -speed);  // drej til højre med defineret hastighed
  Serial.print("OutputL: ");
  Serial.println(speed);
  Serial.print("OutputR: ");
  Serial.println(-speed);
}

void stop() {
  motors.setSpeeds(0, 0);  // stop
}

void calibrateSensors() {
  lineSensors.calibrate();  // gemmer min-og max-værdier for hver sensor fra arrayet [NUM_SENSORS] i lineSensors.calibratedMinimum[NUM_SENSORS] og lineSensors.calibratedMaximum[NUM_SENSORS]
}

void printFast() {
  Serial.print("Error: ");
  Serial.println(lineSensors.readLine(lineSensorValues) - 1000);
}

void printSlow() {
  unsigned long now = millis(); // get the current time in milliseconds since the program started
  if (now - lastPrintTime >= printInterval) { // check if enough time has passed since the last print (based on printInterval)
    lastPrintTime = now; // update the last print time to the current time
    Serial.print("Error: ");
    Serial.println(lineSensors.readLine(lineSensorValues) - 1000);
  }
}

void FollowLine() {
  short position = lineSensors.readLine(lineSensorValues);
  int error = position - 1000;
  if (error == 0) {
    forward();
  }
  else if (error < 0) {
    turnRight();
    }
  else if (error > 0) {
    turnLeft();
    }
  printFast();
  delay(250);  
}

void setup() {
  lineSensors.initThreeSensors();
  Serial.begin(9600);
  while(!buttonA.getSingleDebouncedPress()) {
    calibrateSensors();
    // omregner min- og max-værdierne, således at de henholdsvis bliver 0 og 2000
    delay(20);
    printFast();
  }
  lineSensors.readCalibrated(lineSensorValues); 
}

void loop() {
  FollowLine();
}
