#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;

#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];

int speed = 100;
int threshold = 1000;

void setup()
{
    // put your setup code here, to run once:
    lineSensors.initThreeSensors();
    Serial.begin(9600);
}

void loop()
{
    readLineSensors();
    Serial.print("Sensor 0: ");
    Serial.println(lineSensorValues[0]);
    Serial.print("Sensor 1: ");
    Serial.println(lineSensorValues[1]);
    Serial.print("Sensor 2: ");
    Serial.println(lineSensorValues[2]);
    delay(20);
}
