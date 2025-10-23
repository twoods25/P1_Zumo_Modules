#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4OLED OLED;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;

/* turnAngle is a 32-bit unsigned integer representing the amount
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

// turnRate is the current angular rate of the gyro, in units of 0.07 degrees per second.
int16_t turnRate;
// This is the average reading obtained from the gyro's Z axis during calibration.
int16_t gyroOffset;
// This variable helps us keep track of how much time has passed between readings of the gyro.
uint16_t gyroLastUpdate = 0;

// OptimalTurnSpeed for the gyro to be as precise as possible.
int optimalTurnSpeed = 100;

void setup()
{
  Serial.begin(9600);
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  OLED.clear();
}

int32_t getTurnAngleInDegrees()
{
  return ((int32_t)turnAngle >> 16) * 360 >> 16;
}

int32_t calibrateTurnAng(int32_t Degree)
{
  //Handles logic if angle becomes greater than 180 or less than -180.
  if (Degree > 180)
  {
    return Degree - 360;
  }
  else if (Degree < -180)
  {
    return Degree + 360;
  }
  else
  {
    return Degree;
  }
}

void TurnByDegree(int32_t UserDegreeTurn, int userDirection)
{
  // Saves the current turn angle in degrees
  int32_t currentAngle = getTurnAngleInDegrees();
  // Calculates the calibrated turn angle, by taking the current angle and adding/subtracting the user input angle
  int32_t RightCalibratedTurn = calibrateTurnAng(currentAngle - UserDegreeTurn); // Need to handle a -270 degree turn case ----------------------------------
  int32_t LeftCalibratedTurn = calibrateTurnAng(currentAngle + UserDegreeTurn);

  // A switch to choose between right and left turn on 1 or 0 input.
  switch (userDirection)
  {
  case 0: // Right turn //While the current turn angle is greater than the calibrated turn angle, keep turning right
    while (currentAngle > RightCalibratedTurn)
    {
      // Updates the turn angle, by reading the gyro
      turnSensorUpdate();
      // saves the updated turn angle in degrees
      currentAngle = getTurnAngleInDegrees();
      // Sets the motors to turn right
      motors.setSpeeds(optimalTurnSpeed, -optimalTurnSpeed);

      // Visual
      Serial.println("Degree: " + (String)currentAngle);
      OLED.clear();
    }
    // Stops the motors after the turn is complete.
    motors.setSpeeds(0, 0);
    break;
  case 1: // Left turn //While the current turn angle is less than the calibrated turn angle, keep turning left
    while (currentAngle < LeftCalibratedTurn)
    {
      // Updates the turn angle, by reading the gyro
      turnSensorUpdate();
      // saves the updated turn angle in degrees
      currentAngle = getTurnAngleInDegrees();
      // Sets the motors to turn left
      motors.setSpeeds(-optimalTurnSpeed, optimalTurnSpeed);

      // Visual
      Serial.println("Degree: " + (String)currentAngle);
      OLED.clear();
    }
    // Stops the motors after the turn is complete.
    motors.setSpeeds(0, 0);
    break;
  default:
    break;
  }
}

void loop()
{
  TurnByDegree(90, 0); // Right (0) turn 90 degrees
  delay(3000);
  TurnByDegree(90, 1); // Left (1) turn 90 degrees
  delay(3000);
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
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}