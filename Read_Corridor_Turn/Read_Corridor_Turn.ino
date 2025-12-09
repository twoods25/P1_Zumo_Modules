#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4OLED OLED;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors prox;

int RunRoute = 0;
int number = 1;

// ------------------------------------ Array initialization variables ------------------------------------
// vars to count the accumulated counts of the encoders
int movementCommand = 0;   // used to select the command
int movementParameter = 0; // used to select distance or angle

bool startup_start_con = true;
int const countJump = 100;    // number of counts before jumping to next command in stage 0
int const maxCorridor = 1000; // max distance the robot should travel
int const maxRoom = 10;       // max angle the robot can turn
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
// ------------------------------------ Array initialization variables ------------------------------------

// OptimalTurnSpeed for the gyro to be as precise as possible.
int optimalTurnSpeed = 100;
// Array initialization.
const int sizeA = 3;
const int sizeB = 2;
int commandArray[sizeA][sizeB]; // Array holds five entries of [Command, parameter]
int corridorCount = 0;
int hospitalState = 0; // 0 = searching for corridors, 1 = turning to room, 2 = going out from room, 4 = driving back on "highway". and many more states.
/* -------------------- Gyro Things --------------------
TurnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called.  This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
uint32_t turnAngle = 0;
uint32_t turnAngle2 = 0;
// turnRate is the current angular rate of the gyro, in units of 0.07 degrees per second.
int16_t turnRate;
int16_t turnRate2;
// This is the average reading obtained from the gyro's Z axis during calibration.
int16_t gyroOffset;
// This variable helps us keep track of how much time has passed between readings of the gyro.
uint16_t gyroLastUpdate = 0;
// drive turn variables
const int v_max = 150;               // average forward speed (−400 to 400)
const float Delta_max = 100;         // max speed difference between wheels
const unsigned long turnTime = 1250; // total turn duration in ms
const float pi = 3.1415926;
/* -------------------- Gyro Things -------------------- */

// Konstanter og variabler for FollowLine(); funktionen.
#define NUM_SENSORS 3                         // definerer antal sensorer
unsigned short lineSensorValues[NUM_SENSORS]; // gemmer målingerne i en array
unsigned char speed = 100;                    // sætter hastighed (unsigned short = 0 til 65535)
unsigned short hvid = 500;                    // tærskelværdi for hvid (høj værdi = hvid overflade, lav værdi = sort overflade)

void setup()
{
  Serial.begin(9600);
  // startup();
  delay(2000);
  turnSensorSetup(); // configure gyro / IMU
  delay(500);
  turnSensorReset();      // reset gyro heading to 0
  OLED.clear();           // clear display
  motors.setSpeeds(0, 0); // ensure motors stopped
  delay(1000);
  lineSensors.initThreeSensors(); // initialiserer de tre sensorer
  prox.initThreeSensors();
  delay(500);

  // Initialize commandArray with some example values
  commandArray[RunRoute][0] = 100; // Example: Turn after 3 corridors
  commandArray[RunRoute][1] = 3;   // Example: Turn to room number 2
  commandArray[1][0] = 100;        // Example: Turn after 3 corridors
  commandArray[1][1] = 4;          // Example: Turn to room number 2
  commandArray[2][0] = 100;        // Example: Turn after 3 corridors
  commandArray[2][1] = 1;          // Example: Turn to room number 2

  printOLED("LineCal", "MOVE");
  delay(400);
  unsigned long timeStartUp = millis();
  // FollowLine_P setup for sensor calibration
  while (!buttonA.getSingleDebouncedPress())
  {
    lineSensors.calibrate();
    // gemmer min- og max-værdier for hver sensor fra arrayet [NUM_SENSORS] i
    // lineSensors.calibratedMinimum[NUM_SENSORS] og lineSensors.calibratedMaximum[NUM_SENSORS]
    if (millis() - timeStartUp > 5000)
    {
      printOLED("DONE?", "PRESS A");
    }
  }
  lineSensors.readCalibrated(lineSensorValues);
  // omregner min- og max-værdierne, således at de henholdsvis bliver 0 og 2000
}

void loop()
{
  // AvoidObstacles();
  Corridor_Turn();
  followLine();

  /*/ lineSensors.read(lineSensorValues, QTR_EMITTERS_ON); // læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS]
  lineSensors.calibrate();
  lineSensors.readCalibrated(lineSensorValues);

  Serial.println("Line Sensor Values: ");
  Serial.print(lineSensorValues[0]);
  Serial.print(", ");
  Serial.print(lineSensorValues[1]);
  Serial.print(", ");
  Serial.println(lineSensorValues[2]);
  delay(1500);
  // Debugging line sensor values*/
}

void followLine()
{
  short position = lineSensors.readLine(lineSensorValues);
  // gør internt lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  // dvs. læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS] og sætter en position dvs. 0 1000 2000.
  int error = position - 1000;
  if (position == -1000 || position == 1000)
  {
    error = 0;
  }
  /*else if (error < 20)
      else if (error < -20)*/
  float Kp = 0.3;
  float Up = Kp * error;
  int leftMotorSpeed = constrain(speed + Up, 0, 100);
  int rightMotorSpeed = constrain(speed - Up, 0, 100);
  motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
}
/*//A solution for delay problem could be to use millis() to create a non-blocking delay.
static unsigned long lastCorridorUpdate = 0;
if (millis() - lastCorridorUpdate > 300) {  // 300ms between corridor detections
    lastCorridorUpdate = millis();
    corridorCount++;
}*/
void Corridor_Turn() // !!!!!!!!!!!!!IMPORTANT  I NEED SOME KIND OF DELAY HERE OTHERWISE IT READS THE SENSORS TOO FAST, AND corridorCount++; GETS MESSED UP!!!!!!!!!!!!
{
  /*the logic should be follow the line if the zumo reads a certain color of line count corridors and turns when commandArray[RunRoute][0],
      // which is equal to 100-200-300-...-1000 and commandArray[RunRoute][1] is equal to the room number 1-2-3-...-10 it should turn to.
      // I NEED to read the linesensors for a certain color to count corridors.*/
  // FEjl lineSensors.read(lineSensorValues, QTR_EMITTERS_ON); // læser sensorerne og gemmer værdierne i arrayet [NUM_SENSORS]
  lineSensors.calibrate();
  lineSensors.readCalibrated(lineSensorValues);
  switch (hospitalState)
  {
  case 0:
    // logic for searching for corridors
    if (lineSensorValues[1] < 200)
    {
      corridorCount++;
      // count corridors
      if (corridorCount == (commandArray[RunRoute][0] / 100))
      {
        printOLED("CORRIDOR", String(corridorCount * 100));
        DriveTurn(90, 1); // turn right 90 degrees (1 = left)
        corridorCount = 0;
        hospitalState = 1; // change state to turning to room
      }
      else
      {
        forward();
        printOLED("SKIPS", "ToNEXT");
        delay(800);
      }
    }
    break;
  case 1:
    // logic for turning to room
    if ((lineSensorValues[0] > 300) && (lineSensorValues[2] > 300) && (lineSensorValues[1] < 200))
    {
      corridorCount += 2;
      // count corridors
      if ((corridorCount == commandArray[RunRoute][1]) || ((corridorCount - 1) == commandArray[RunRoute][1]))
      {
        printOLED("ROOM", String(commandArray[RunRoute][1]));
        forward();
        delay(500);
        if ((commandArray[RunRoute][1] % 2) == 0)
        {
          // DriveTurn(90, 0); // turn left 90 degrees (0 = right)
          stop();
          TurnByDegree(90);
        }
        else
        {
          // DriveTurn(90, 1); // turn right 90 degrees (1 = left)
          stop();
          TurnByDegree(-90);
        }
        corridorCount = 0;
        hospitalState = 2; // change state to turning to room
      }
      else
      {
        forward();
        delay(800);
      }
    }
    break;
  case 2:
    // logic for going out from room
    if ((lineSensorValues[0] < 200) && (lineSensorValues[2] < 200) && (lineSensorValues[1] < 200))
    {
      printOLED("EXIT", "ROOM");
      TurnByDegree(90);  // turn around 180 degrees
      TurnByDegree(90);  // turn around 180 degrees
      hospitalState = 3; // change state to driving back on "highway"
      forward();
      delay(500);
    }
    break;
  case 3:
    // logic for turning to room, is the exact same logic for getting out of the room. if you turned left to get into the room, you turn left to get out.
    if (lineSensorValues[1] < 200)
    {
      forward();
      delay(500);
      if (commandArray[RunRoute][1] % 2 == 0)
      {
        // DriveTurn(90, 0); // turn left 90 degrees (0 = right)
        stop();
        TurnByDegree(90);
      }
      else
      {
        // DriveTurn(90, 1); // turn right 90 degrees (1 = left)
        stop();
        TurnByDegree(-90);
      }
      corridorCount = commandArray[RunRoute][1]; // reset corridor count to the room number
      hospitalState = 4;                         // change state to turning to room
    }
    break;
  case 4:
    // logic for driving back on "highway"
    if (lineSensorValues[1] < 200)
    {
      Serial.println("I was here!!!");
      if ((corridorCount == 3) || (corridorCount == 4))
      {
        forward();
        delay(200);
        printOLED("BACK", "HIGHWAY");
        DriveTurn(90, 1); // turn left 90 degrees (1 = left)
        corridorCount = 0;
        hospitalState = 5; // change state to searching for corridors
      }
      else
      {
        printOLED("SKIPS", "ToNEXT");
        // count corridors
        corridorCount += 2;
        forward();
        delay(700);
      }
    }
    break;
  case 5:
    // logic for going back to pickup point, (0,0)
    if (lineSensorValues[1] < 200)
    {
      printOLED("PICKUP", "REACHED");
      forward();
      delay(700);
      hospitalState = 0;
      RunRoute++;
    }
    break;
  default:
    break;
  }
}
// Returns the current turn angle in degrees based on 'turnAngle' fixed-point value
int32_t getTurnAngleInDegrees()
{
  // 360 degrees corresponds to a full 32-bit rotation (2^32 units)
  // So 1 degree = 2^32 / 360 ≈ 11930465.78 units
  return ((int64_t)turnAngle * 360) / 4294967296; // = 2^32
}
// Calibrates an angle to the range [-180, 180] degrees
int32_t calibrateTurnAng(int32_t Degree)
{
  if (Degree > 180)
  {
    return Degree - 360; // wrap angles > 180 to negative side
  }
  else if (Degree < -180)
  {
    return Degree + 360; // wrap angles < -180 to positive side
  }
  else
  {
    return Degree; // already in range
  }
}
// Converts a user-specified degree turn into an approximate duration in ms
unsigned long secondsToDegree(int32_t UserDegreeTurn1)
{
  return (unsigned long)(turnTime * (float)UserDegreeTurn1 / 90.0);
  // Assumes 'turnTime' is the time (ms) for a 90° turn; scales linearly
}
void TurnByDegree(int32_t userTurn)
{
  int32_t startAngle = getTurnAngleInDegrees();
  int32_t currentAngle = startAngle;
  int32_t delta = 0;

  int leftSpeed, rightSpeed;

  // figure out which way to spin
  if (userTurn > 0)
  { // turn right (CW)
    leftSpeed = optimalTurnSpeed;
    rightSpeed = -optimalTurnSpeed;
  }
  else if (userTurn < 0)
  { // turn left (CCW)
    leftSpeed = -optimalTurnSpeed;
    rightSpeed = optimalTurnSpeed;
  }
  else
  {
    motors.setSpeeds(0, 0);
    return;
  }
  motors.setSpeeds(leftSpeed, rightSpeed);
  unsigned long lastUpdate = millis();

  while (true)
  {
    turnSensorUpdate();
    currentAngle = getTurnAngleInDegrees();
    delta = currentAngle - startAngle;

    // keep angle in range [-180, 180]
    if (delta > 180)
      delta -= 360;
    if (delta < -180)
      delta += 360;

    // stop once we hit or pass target
    if ((userTurn > 0 && delta >= userTurn) || (userTurn < 0 && delta <= userTurn))
    {
      break;
    }

    // print every ~100ms for debug // I had to insert a way to watch how the robot is rotating.
    if (millis() - lastUpdate > 100)
    {
      Serial.print("Angle: ");
      Serial.print(currentAngle);
      Serial.print("  Δ=");
      Serial.println(delta);
      lastUpdate = millis();
    }
  }

  motors.setSpeeds(0, 0);
  delay(100);
}
// Returns the current turn angle in degrees based on 'turnAngle' fixed-point value
int32_t DriveTurnGetTurnAngleInDegrees()
{
  return ((int32_t)turnAngle2 >> 16) * 360 >> 16;
} // converts 32-bit fixed-point to int degrees
// Drive the robot through a smooth turn with sine-speed profile and gyro-based stopping
void DriveTurn(int32_t userDegreeTurn3, int userDirection)
{
  unsigned long startTime = millis(); // record start time of turn
  unsigned long t = 0;                // elapsed time
  bool angleBreakpointReached = true; // has robot reached target angle?
  bool runOnce = true;                // flag to initialize TurnByDegree once

  while (t <= secondsToDegree(userDegreeTurn3) && angleBreakpointReached)
  {
    // ---------------- SINE-SPEED PROFILE ----------------
    // Compute the progress of the turn (0.0 → 1.0) based on elapsed time
    float progress = (float)t / (float)secondsToDegree(userDegreeTurn3);

    // Compute the motor speed offset using a sine function
    // - At progress=0, sin(0)=0 → minimal offset, motors roughly equal → start turning slowly
    // - At progress=0.5, sin(pi/2)=1 → maximal offset → middle of turn, robot turns fastest
    // - At progress=1, sin(pi)=0 → offset returns to 0 → smooth deceleration near target
    float delta = Delta_max * sin(pi * progress);

    int vL = v_max - delta; // left motor slows for left turn, speeds up for right turn
    int vR = v_max + delta; // right motor speeds up for left turn, slows for right turn

    motors.setSpeeds(vL, vR); // apply smooth motor speeds

    delay(50);                // control loop interval ~50 Hz
    t = millis() - startTime; // update elapsed time

    // ---------------- GYRO-BASED STOPPING ----------------
    // Check current heading using TurnByDegree
    // - This ensures that even if the sine profile would "overshoot",
    //   the robot stops precisely when the gyro target angle is reached
    angleBreakpointReached = DriveTurnTurnByDegree(userDegreeTurn3, userDirection, runOnce);
    runOnce = false; // targets initialized only on first call
  }
  motors.setSpeeds(0, 0); // stop motors precisely at the target angle
}
// This func is needed for DriveTurn();
//  Returns true if the robot still needs to turn, false if the target angle is reached
bool DriveTurnTurnByDegree(int32_t UserDegreeTurn2, int userDirection, bool runOnce)
{
  static int32_t currentAngle;        // persists across multiple calls
  static int32_t RightCalibratedTurn; // right-turn target angle
  static int32_t LeftCalibratedTurn;  // left-turn target angle

  if (runOnce)
  {                                                                         // initialize targets only once
    currentAngle = DriveTurnGetTurnAngleInDegrees();                        // read initial heading
    RightCalibratedTurn = calibrateTurnAng(currentAngle - UserDegreeTurn2); // right target
    LeftCalibratedTurn = calibrateTurnAng(currentAngle + UserDegreeTurn2);  // left target
  }

  turnSensorUpdate();                              // read latest gyro values
  currentAngle = DriveTurnGetTurnAngleInDegrees(); // store current heading

  Serial.println("Degree: " + String(currentAngle)); // debug output
  OLED.clear();                                      // optional display refresh

  // check if robot still needs to turn
  switch (userDirection)
  {
  case 0:
    return currentAngle > RightCalibratedTurn; // right turn not complete?
  case 1:
    return currentAngle < LeftCalibratedTurn; // left turn not complete?
  default:
    return false; // invalid direction
  }
}
// Should be called in setup.
void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  OLED.clear();
  OLED.print(F("Gyro cal"));

  // Turn on the yellow LED in case the OLED is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady())
    {
    }
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  OLED.clear();
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease())
  {
    turnSensorUpdate();
    OLED.gotoXY(0, 0);
    // do some math and pointer magic to turn angle in seconds to angle in degrees
    OLED.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    OLED.print(F("   "));
  }
  OLED.clear();
}
// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}
// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = -(imu.g.z - gyroOffset);
  turnRate2 = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;
  int32_t d2 = (int32_t)turnRate2 * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
  turnAngle2 += (int64_t)d2 * 14680064 / 17578125;
}
void forward()
{
  motors.setSpeeds(speed, speed); // kør med defineret hastighed
}
void backwards()
{
  motors.setSpeeds(-speed, -speed); // kør baglæns med defineret hastighed
}
void turnLeft()
{
  motors.setSpeeds(-speed, speed); // drej til venstre med defineret hastighed
}
void turnRight()
{
  motors.setSpeeds(speed, -speed); // drej til højre med defineret hastighed
}
void stop()
{
  motors.setSpeeds(0, 0); // stop
}
// The startup sequence
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
      printOLED("PRESS", "B");
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
  printOLED("EXECUTING", "PATH");
  delay(1000);
  printOLED("GYRO", "START");
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

// Avoid AvoidObstacles()
// Placeholder for obstacle avoidance logic
void AvoidObstacles()
{
  int lightLevel = 12;
  if (SensorValue() >= lightLevel)
  {
    // case 0:
    forward();
    delay(200);
    // case 1:
    stop();
    TurnByDegree(-45);
    // case 2:
    bool breakcon = true;
    while (breakcon)
    {
      delay(500);
      if (SensorValue() <= lightLevel)
      {
        forward();
        delay(2500);
        stop();
        breakcon = false;
      }
      else
      {
        stop();
        TurnByDegree(-45);
        number = 2;
      }
    }
    // case 3:
    breakcon = true;
    while (breakcon)
    {
      TurnByDegree((45 * number));
      delay(500);
      if (SensorValue() <= lightLevel)
      { // hvis der ikke er noget foran den, må der være fri bane
        forward();
        delay(2500); // her skal der regnes ud hvor lang tid den skal kører, udfra hvor langt den stopper væk fra det på linjen.
        stop();
        breakcon = false;
      }
      else
      { // hvis der stadig ikke er fri, skal den kører endnu længere ud
        TurnByDegree(-90);
        forward();
        number = 2;
        delay(1000);
        stop();
      }
    }
    // case 4:
    breakcon = true;
    while (breakcon)
    {
      TurnByDegree(90);
      delay(500);
      if (SensorValue() <= lightLevel)
      {
        lineSensors.calibrate();
        lineSensors.readCalibrated(lineSensorValues);
        forward();
        while (lineSensorValues[1] < 600)
        {
          lineSensors.calibrate();
          lineSensors.readCalibrated(lineSensorValues);
        }
        delay(600);
        TurnByDegree(-90);
        breakcon = false;
      }
      else
      {
        TurnByDegree(-90);
        forward();
        delay(1500);
        stop();
      }
    }
  }
}
int *SensorValue()
{

  prox.read();
  static int values[3]; // [0]=front, [1]=left, [2]=right

  // Front
  values[0] = prox.countsFrontWithLeftLeds() + prox.countsFrontWithRightLeds();

  // Side
  values[1] = prox.countsLeftWithLeftLeds() + prox.countsLeftWithRightLeds();
  values[2] = prox.countsRightWithLeftLeds() + prox.countsRightWithRightLeds();

  int front = values[0];

  return front;
}
