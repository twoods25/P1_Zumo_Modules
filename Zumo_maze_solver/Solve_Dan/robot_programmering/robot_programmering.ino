#include <Wire.h>      // inkluderer bibliotek
#include <Zumo32U4.h>  // inkluderer bibliotek

Zumo32U4Motors motors;            // inkluderer motorer
Zumo32U4LineSensors lineSensors;  // inkluderer
Zumo32U4OLED OLED;                // inkluderer OLED-skærm

#define NUM_SENSORS 3                          // definerer antal sensorer
unsigned short lineSensorValues[NUM_SENSORS];  // gemmer målingerne i en array
int Direction = 1;                             // retning - bruges til cases
int subDirection = 2;                          // retning - bruges til cases

//                                 L     C     R
//Hvis det er hvid er det under = 300 , 300 , 300

unsigned short speed = 100;  // sætter hastighed (unsigned short = 0 til 65535)

void printReadingsToSerial() {
  char buffer[80];                                                                                  // opretter tegn-array med plads til 80 tegn
  sprintf(buffer, "%4d %4d %4d\n", lineSensorValues[0], lineSensorValues[1], lineSensorValues[2]);  // i stedet for Serial.print(), da sprintf() er pæn og i en streng
  Serial.print(buffer);                                                                             // print strengen
}

void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  // læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS]
}

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

void chooseDirection() {
  motors.setSpeeds(speed, speed);
  while (Direction == 1) {
    readLineSensors();
    if (lineSensorValues[0] > 300) {  // hvis venstre sensor ser linjen = følg venstre
      Direction = 0;
      subDirection = 2;
    } else if (lineSensorValues[2] > 300) {  // hvis højre sensor ser linjen = følg højre
      Direction = 2;
      subDirection = 0;
    }
  }
}

void followDirection() {
  while (lineSensorValues[Direction] > 500 && lineSensorValues[1] <= 300 && lineSensorValues[subDirection] <= 300) {
    forward();  // så længe kun venstre sensor ser linjen: kør
    readLineSensors();
  }
  while (lineSensorValues[Direction] < 300) {
    if (Direction == 0) turnLeft();  // så længe venstre sensor ikke ser linjen: drej til venstre
    else if (Direction == 2) turnRight();
    readLineSensors();
  }

  while (lineSensorValues[1] > 300 || lineSensorValues[subDirection] > 300) {
    if (Direction == 0) turnRight();  // så længe midt eller højre sensor ser linje: drej til højre
    else if (Direction == 2 && lineSensorValues[subDirection] > 300) turnLeft();
    else if (Direction == 2 && lineSensorValues[Direction] > 300) motors.setSpeeds(100,0);
    readLineSensors();
  }
}

void setup() {
  lineSensors.initThreeSensors();  // initialiserer de tre sensorer
  Serial.begin(9600);
  chooseDirection();  // vælg retning
}

void loop() {
  readLineSensors();        // læs sensorerne
  printReadingsToSerial();  // print til serial monitor
  switch (Direction) {
    case 0:
      followDirection();
      break;
    case 2:
      followDirection();
      break;
    /*case 1: 
    followLeft();
    break;
  case 2: 
    followRight();
    break;*/
    default:
      forward();  // hvis ingen linje er fundet: kør lige ud
  }
}