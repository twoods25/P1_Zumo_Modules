#include <Wire.h>      // inkluderer bibliotek
#include <Zumo32U4.h>  // inkluderer bibliotek

Zumo32U4Motors motors;            // inkluderer motorer
Zumo32U4LineSensors lineSensors;  // inkluderer
Zumo32U4OLED OLED;                // inkluderer OLED-skærm

// Konstanter og variabler for FollowLine(); funktionen.
#define NUM_SENSORS 3                          // definerer antal sensorer
unsigned int lineSensorValues[NUM_SENSORS];  // gemmer målingerne i en array
unsigned char speed = 100;  // sætter hastighed (unsigned short = 0 til 65535)
unsigned short hvid = 500;  // tærskelværdi for hvid (høj værdi = hvid overflade, lav værdi = sort overflade)

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

void followLine() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  // læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS]
  while (lineSensorValues[1] > hvid && lineSensorValues[0] <= hvid && lineSensorValues[2] <= hvid) {
    forward();  // så længe kun venstre sensor ser linjen: kør
    lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  // læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS]
  }
  while (lineSensorValues[0] > hvid || lineSensorValues[2] > hvid) {
    if (lineSensorValues[0] > hvid) {
      turnLeft();
      lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  // læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS]
    }
    else if (lineSensorValues[2] > hvid) {// så længe venstre sensor ikke ser linjen: drej til venstre
      turnRight();
      lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);  // læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS]
    }
  } 
}

void setup() {
  lineSensors.initThreeSensors();  // initialiserer de tre sensorer
  Serial.begin(9600);
}

void loop() {
  //followLine();

}
