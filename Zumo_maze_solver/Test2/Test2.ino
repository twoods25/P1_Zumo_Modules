#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;

#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];

int speed = 100;
int threshold = 1000;
int Direction = 0;
int subDirection = 2;


// the uncalibrated line sensor reading are between 0 (very bright) and 2000 (very dark) 280-2000 left(0) and right(2): middle(1) = 140-1280
void readLineSensors()
{
    lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

void setup()
{
    // put your setup code here, to run once:
    lineSensors.initThreeSensors();
    Serial.begin(9600);
    while (1)
    {
        readLineSensors();
        motors.setSpeeds(speed, speed);

        if (lineSensorValues[0] > 1000)
        {
            Direction = 0;
            subDirection = 2;
            return;
        }
        else if (lineSensorValues[2] > 1000)
        {
            Direction = 2;
            subDirection = 0;
            return;
        }
    }
}

void options()
{
    if ((lineSensorValues[Direction] > 1000) && (lineSensorValues[1] < 600) && (lineSensorValues[subDirection] < 1000))
    {
        motors.setSpeeds(speed, speed);
    }
    else if ((lineSensorValues[Direction] < 1000) && (lineSensorValues[1] < 600) && (lineSensorValues[subDirection] < 1000))
    {
        while (lineSensorValues[Direction] < 1000)
        {
            readLineSensors();
            if (Direction == 0) motors.setSpeeds(-speed, speed);
            else motors.setSpeeds(speed, -speed);
        }
    }
    else if ((lineSensorValues[Direction] > 1000) && (lineSensorValues[1] > 600) && (lineSensorValues[subDirection] > 1000))
    {
        while (lineSensorValues[Direction] > 1000)
        {
            readLineSensors();
            if (Direction == 0) motors.setSpeeds(speed, -speed);
            else motors.setSpeeds(-speed, speed);
        }
    }
    else if ((lineSensorValues[Direction] > 1000) && (lineSensorValues[1] > 600) && (lineSensorValues[subDirection] < 1000)){
        if (Direction == 0) motors.setSpeeds(speed, speed/2);
            else motors.setSpeeds(speed/2, speed);
        
    }
}

void loop()
{
    readLineSensors();
    options();
}
