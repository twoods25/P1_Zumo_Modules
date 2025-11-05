#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED oled;

#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];

int speed = 100;

int Proces = 0;

String TR = "Right";
String TL = "Left";
String Find = "Find Maze";



void setup() {

  lineSensors.initThreeSensors();
  Serial.begin(9600);
  randomSeed(analogRead(0));
}

// Prints a line with all the sensor readings to the serial
// monitor.
void printReadingsToSerial() {
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d\n",
          lineSensorValues[0],
          lineSensorValues[1],
          lineSensorValues[2]);
  Serial.print(buffer);
}

// the uncalibrated line sensor reading are between 0 (very bright) and 2000 (very dark)
void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  printReadingsToSerial();
}
// stop the zumo
void stop() {
  motors.setSpeeds(0, 0);
}
// forward for the Zumo.
void forward() {
  motors.setSpeeds(speed, speed);
}
// back for the Zumo.
void back() {
  motors.setSpeeds(-speed, -speed);
}

// the TurnLeft function
void TurnLeft() {

  oled.clear();
  oled.print(TL);
  /*
The Zumo shall turn until it can see the line agian, because of it's diretion turinging.
*/

  while (lineSensorValues[0] < 1000) {
    readLineSensors();
    motors.setSpeeds(-100, 100);
  }
  stop();
  oled.clear();
}

//the TrunRight Funcion.

void TurnRight() {

  oled.clear();
  oled.print(TR);
  /*
The Zumo shall turn until it can not see the line, becuase of it's diretion turinging.
*/
  while (lineSensorValues[0] > 1000) {
    readLineSensors();
    motors.setSpeeds(100, -100);
  }
  stop();
  oled.clear();
}

void loop() {
  readLineSensors();

  if (lineSensorValues[0] > 500 || lineSensorValues[1] > 300 || lineSensorValues[2] > 500) {
    Proces = 1;
  }


  /*
We uses switch cases to dectect what stage we are in.
case 0 = find the maze
Case 1 = solve the maze
*/

  switch (Proces) {

    case 0:
      forward();

      oled.clear();
      oled.print(Find);
      break;

    case 1:
      StayOnTheLine();

      while (lineSensorValues[0] > 1000) {
        readLineSensors();
        forward();

        if (lineSensorValues[2] > 1000) {
          TurnRight();
        }
      }

      if (lineSensorValues[0] < 500 && lineSensorValues[1] < 300 && lineSensorValues[2] < 500) {
        TurnLeft();
      }
      break;
  }
}



void StayOnTheLine() {
  /*
If the zumo is about to go off the Line, this function will get it back on the line.
*/
  if (lineSensorValues[0] < 1000 && lineSensorValues[1] > 200) {
    readLineSensors();
    motors.setSpeeds(175, 75);
  }

  else if (lineSensorValues[0] < 1000 && lineSensorValues[1] < 200) {
    readLineSensors();
    motors.setSpeeds(75, 175);
  }
}
