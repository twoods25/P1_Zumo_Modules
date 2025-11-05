#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4Encoders encoders;

#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];

int speed = 100;
int threshold = 1000;
bool Direction = false; // false = left, true = right

bool direction()
{
    readLineSensors();
    return (lineSensorValues[0] < lineSensorValues[2]);
} 
void setup()
{
    // put your setup code here, to run once:
    lineSensors.initThreeSensors();
    Serial.begin(9600);
    Direction = false;//direction();
}

// the uncalibrated line sensor reading are between 0 (very bright) and 2000 (very dark) 280-2000 left(0) and right(2): middle(1) = 140-1280
void readLineSensors()
{
    lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

void forward(int speedleft, int speedright)
{
    motors.setSpeeds(speedleft, speedright);
}

bool startDirection()
{
    bool sen0left = lineSensorValues[0] > 700;
    bool sen1mid = lineSensorValues[1] > 500;
    bool sen2right = lineSensorValues[2] > 700;
    if (!Direction && sen0left && !sen1mid && !sen2right)
    {
        return (sen0left && !sen1mid && !sen2right);
    }
    else if (Direction && !sen0left && !sen1mid && sen2right)
    {
        return (!sen0left && !sen1mid && sen2right);
    }
}

int options()
{
    bool sen0left = lineSensorValues[0] > 700;
    bool sen1mid = lineSensorValues[1] > 500;
    bool sen2right = lineSensorValues[2] > 700;

    if (sen0left && sen1mid && !sen2right) // Sort side
    {
        return 1; // drej ind mod labyrinten, hvis venstre side + midt sorte
    }
    else if (!sen0left && sen1mid && sen2right) // Sort side
    {
        return 2; // drej ind mod labyrinten, hvis højre side + midt sorte
    }
    else if (sen0left && sen1mid && sen2right) // alle sensor sorte
    {
        return 3; // back og kør til den rigtige direction 90 grader
    }
    else if (!sen0left && !sen1mid && !sen2right) // Alle sensor hvide
    {
        return 4; // back og kør til den forkerte side direction 90 grader
    }
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

    if (startDirection())
    {
        forward(speed, speed);
    }
    else
    {
        switch (options())
        {
        case 1:
            if (!Direction)
                forward(speed, speed / 2);
            break;
        case 2:
            if (Direction)
                forward(speed / 2, speed);
            break;
        case 3:
            //forward(-speed, -speed);
            if (!Direction)
            {
                while (lineSensorValues[0] > 700)
                {
                    readLineSensors();
                    forward(speed, -speed);
                    delay(10);
                }
            }
            else if (Direction)
            {
                while (lineSensorValues[2] > 700)
                {
                    readLineSensors();
                    forward(-speed, speed);
                    delay(10);
                }
            }
            break;
        case 4:
            // forward(-speed, -speed);
            // delay(100); //Use encoders here instead of delay
            if (!Direction){
                while (lineSensorValues[0] < 700)
                {
                    readLineSensors();
                    forward(-speed, speed);
                    delay(10);
                }
            }
            else {
                while (lineSensorValues[2] < 700)
                {
                    readLineSensors();
                    forward(speed, -speed);
                    delay(10);
                }
            }
            break;
        default:
            break;
        }
    }
}