#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED OLED;

const uint16_t speed = 100;
int16_t lastError = 0;

#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];

void calibrateSensors() {
  for (uint16_t i=0; i<120; i++) {
    if (i>30 && i<=90) {
      motors.setSpeeds(-100, 100);
    } 
    else {
      motors.setSpeeds(100, -100);
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0,0);
}

void setup() {
  lineSensors.initThreeSensors();
  Serial.begin(9600);
  calibrateSensors();
}

void followLine(){
  int16_t position = lineSensors.readLine(lineSensorValues, QTR_EMITTERS_ON);
  int16_t error = position - 1000;
  int16_t speedDifference = error / 4 + 6 * (error - lastError);
  lastError = error;
  int16_t leftSpeed = (int16_t)speed + speedDifference;
  int16_t rightSpeed = (int16_t)speed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0, (int16_t)speed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)speed);
  motors.setSpeeds(leftSpeed, rightSpeed);
}

void printReadings() {
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d\n", lineSensorValues[0], lineSensorValues[1], lineSensorValues[2]);
  Serial.print(buffer);
}

void loop() {
  followLine();
}


/*
void forward() {
  motors.setSpeeds(speed,speed);
}

void stop() {
  motors.setSpeeds(0,0);
}

bool leftSensorWhite(){
  return (lineSensorValues[0] >= 190 && lineSensorValues[0] <= 270);
}

bool middleSensorBlack(){
  return (lineSensorValues[1] >= 700 && lineSensorValues[1] <= 950);
}

bool rightSensorWhite(){
  return (lineSensorValues[2] >= 190 && lineSensorValues[2] <= 270);
}


void loop() {
  readLineSensors();
  printReadings();
  
  switch() {
    case 0: 
      while (leftSensorWhite() && middleSensorBlack() && rightSensorWhite()) {
      forward();
      }
      break;

    case 1: 
      while (!rightSensorWhite()) {
      TurnByDegree(5,0);
      delay(1000);
      forward();
      }
      break;
    
    case 2:
      while (!leftSensorWhite()) {
      TurnByDegree(5,1)
      delay(1000);
      forward();
      }
      break;

  delay(200);
}
*/
