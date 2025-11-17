#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4OLED OLED;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Motors motors;

// vars to count the accumulated counts of the encoders
int movementCommand = 0;   // used to select the command
int movementParameter = 0; // used to select distance or angle

bool startup_start_con = true;
int const countJump = 100;    // number of counts before jumping to next command in stage 0
int const maxCorridor = 1000; // max distance the robot should travel
int const maxRoom = 10;       // max angle the robot can turn
int speed = 100;              // max speed of the robot
float wheelCirc = 13.0;
int routeIndex = 0;
// control the flow of the program. 0 wait for command
//                                  1 wait for speed and time
//                                  2 running the command.
int stage = 0;

// control selection 1              0 Forward
//                                  1 Backwards
//                                  2 turn
int chosenCommand = 0;

const int sizeA = 3;            // size of first dimension of command array
const int sizeB = 2;            // size of second dimension of command array
int commandArray[sizeA][sizeB]; // Array holds five entries of [Command, parameter]

void setup()
{
    Serial.begin(9600);
    delay(100);
}

void loop()
{
    startup();
    printOLED("STARTUP", "COMPLETED");
    delay(2000);
    //PRINTING THE ARRAY FOR TESTING
    Serial.println("=== Command Array ===");
    for (int i = 0; i < sizeA; i++)  // loop over rows
    {
        Serial.print("Entry ");
        Serial.print(i);
        Serial.print(": ");

        for (int j = 0; j < sizeB; j++)  // loop over columns
        {
            Serial.print(commandArray[i][j]);
            if (j < sizeB - 1)
                Serial.print(", "); // separate command and parameter
        }

        Serial.println(); // move to next line
    }
    Serial.println("=====================");
}

bool startup()
{
    while (startup_start_con)
    {
        delay(50);
        switch (stage)
        {
        case 0:
            selectMovement();
            break;
        case 1:
            ProgramSelection();
            break;
        /*case 2:
            ParameterSelection();
            break;*/
        case 3:
            ExecuteProgram();
            break;
        }
    }
}
void ProgramSelection()
{
    switch (chosenCommand)
    {
    case 0:
        ResetRoute();
        break;
    case 1:
        AddNewRoute();
        break;
    case 2:
        if (buttonB.isPressed())
        {
            bip();
            stage = 3;
        }
        delay(50);
        break;
    }
}
void ResetRoute()
{
    if (buttonA.isPressed())
    {
        bip();
        clearCommandArray();
        printOLED("ROUTE", "CLEARED!");
        // Reset states
        movementCommand = 0;
        movementParameter = 0;
        resetEncoders();
        buttonA.waitForRelease();
        chosenCommand = 0;
        stage = 0;
        routeIndex = 0;
    }
}
void clearCommandArray()
{
    for (int i = 0; i < sizeA; i++)
    {
        for (int j = 0; j < sizeB; j++)
        {
            commandArray[i][j] = 0;
        }
    }
}
void AddNewRoute()
{
    int corridor = 0;
    int room = 0;
    int route_stage = 0; // 0 select corridor, 1 select room
    while (route_stage == 0)
    {
        printOLED("<Corridor", String(corridor));
        readEncodersParameter();
        if (movementParameter > countJump)
        {
            bip();
            movementParameter = 0;
            corridor += 100;
            if (corridor > maxCorridor)
                corridor = maxCorridor;
        }
        else if (movementParameter < -countJump)
        {
            bip();
            movementParameter = 0;
            corridor -= 100;
            if (corridor < 0)
                corridor = 0;
        }
        // OLEDSelectParameter();
        if (buttonA.isPressed())
        {
            bip();
            commandArray[routeIndex][0] = corridor;
            stage = 0;
            route_stage = 1;
            buttonA.waitForRelease();
        }
        delay(50);
    }
    while (route_stage == 1)
    {
        printOLED("<Room ", String(room));
        readEncodersParameter();
        if (movementParameter > countJump)
        {
            bip();
            movementParameter = 0;
            room += 1;
            if (room > maxRoom)
                room = maxRoom;
        }
        else if (movementParameter < -countJump)
        {
            bip();
            movementParameter = 0;
            room -= 1;
            if (room < 0)
                room = 0;
        }
        // OLEDSelectParameter();
        if (buttonA.isPressed())
        {
            bip();
            commandArray[routeIndex][1] = room;
            routeIndex++;
            stage = 0;
            route_stage = 0;
            buttonA.waitForRelease();
        }
        delay(50);
    }
}
void selectMovement()
{
    readEncodersMovement();
    if (movementCommand > countJump)
    {
        bip();
        movementCommand = 0;
        chosenCommand++;
        if (chosenCommand > 3)
            chosenCommand = 0;
    }
    else if (movementCommand < -countJump)
    {
        bip();
        movementCommand = 0;
        chosenCommand--;
        if (chosenCommand < 0)
            chosenCommand = 3;
    }
    OLEDSelectMovement();
    if (buttonA.isPressed())
    {
        bip();
        stage = 1;
        buttonA.waitForRelease(); // wait till the button is released
    }
}
void OLEDSelectMovement()
{
    switch (chosenCommand)
    {
    case 0:
        printOLED("Option>", "RESET");
        break;
    case 1:
        printOLED("Option>", "NEWPATH");
        break;
    case 2:
        printOLED("Option>", "EXECUTE");
        break;
    case 3:
        printOLED("PATHS: >", String(routeIndex));
        break;
    }
}
void ExecuteProgram()
{
    printOLED("EXECUTING", "PROGRAM");
    delay(1000);
    printOLED("PROGRAM", "COMPLETED");
    delay(2000);
    // Reset states
    movementCommand = 0;
    movementParameter = 0;
    resetEncoders();
    chosenCommand = 0;
    stage = 0;
    routeIndex = 0;

    // Terminating the startup loop
    startup_start_con = false;
}
// Sound functions
void bip()
{
    buzzer.playNote(NOTE_A(4), 20, 15);
    delay(30);
}
// Easy fucntion to print in the OLED
void printOLED(String s0, String s1)
{
    OLED.clear();
    OLED.print(s0);
    OLED.gotoXY(0, 1);
    OLED.print(s1);
}
void resetEncoders()
{
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
}
void readEncodersMovement()
{
    movementCommand = movementCommand + encoders.getCountsRight();
    resetEncoders();
}
void readEncodersParameter()
{
    movementParameter = movementParameter + encoders.getCountsLeft();
    resetEncoders();
}
