#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4OLED OLED;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;

// Gyro Things
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

const int v_max = 150;                // average forward speed (−400 to 400)
const float Delta_max = 100;          // max speed difference between wheels
const unsigned long turnTime = 1250;  // total turn duration in ms
const float pi = 3.1415926;

// Returns the current turn angle in degrees based on 'turnAngle' fixed-point value
int32_t getTurnAngleInDegrees() {
  return ((int32_t)turnAngle >> 16) * 360 >> 16;
}  // converts 32-bit fixed-point to int degrees
// Calibrates an angle to the range [-180, 180] degrees
int32_t calibrateTurnAng(int32_t Degree) {
  if (Degree > 180) {
    return Degree - 360;  // wrap angles > 180 to negative side
  } else if (Degree < -180) {
    return Degree + 360;  // wrap angles < -180 to positive side
  } else {
    return Degree;  // already in range
  }
}
// Converts a user-specified degree turn into an approximate duration in ms
unsigned long secondsToDegree(int32_t UserDegreeTurn1) {
  return (unsigned long)(turnTime * (float)UserDegreeTurn1 / 90.0);
  // Assumes 'turnTime' is the time (ms) for a 90° turn; scales linearly
}

// Returns true if the robot still needs to turn, false if the target angle is reached
bool TurnByDegree(int32_t UserDegreeTurn2, int userDirection, bool runOnce) {
  static int32_t currentAngle;         // persists across multiple calls
  static int32_t RightCalibratedTurn;  // right-turn target angle
  static int32_t LeftCalibratedTurn;   // left-turn target angle

  if (runOnce)  { // initialize targets only once
    currentAngle = getTurnAngleInDegrees();                                  // read initial heading
    RightCalibratedTurn = calibrateTurnAng(currentAngle - UserDegreeTurn2);  // right target
    LeftCalibratedTurn = calibrateTurnAng(currentAngle + UserDegreeTurn2);   // left target
  }

  turnSensorUpdate();                      // read latest gyro values
  currentAngle = getTurnAngleInDegrees();  // store current heading

  Serial.println("Degree: " + String(currentAngle));  // debug output
  OLED.clear();                                       // optional display refresh

  // check if robot still needs to turn
  switch (userDirection) {
    case 0:
      return currentAngle > RightCalibratedTurn;  // right turn not complete?
    case 1:
      return currentAngle < LeftCalibratedTurn;  // left turn not complete?
    default:
      return false;  // invalid direction
  }
}

// Drive the robot through a smooth turn with sine-speed profile and gyro-based stopping
void DriveTurn(int32_t userDegreeTurn3, int userDirection) {
  unsigned long startTime = millis();  // record start time of turn
  unsigned long t = 0;                 // elapsed time
  bool angleBreakpointReached = true;  // has robot reached target angle?
  bool runOnce = true;                 // flag to initialize TurnByDegree once

  while (t <= secondsToDegree(userDegreeTurn3) && angleBreakpointReached) {
    // ---------------- SINE-SPEED PROFILE ----------------
    // Compute the progress of the turn (0.0 → 1.0) based on elapsed time
    float progress = (float)t / (float)secondsToDegree(userDegreeTurn3);

    // Compute the motor speed offset using a sine function
    // - At progress=0, sin(0)=0 → minimal offset, motors roughly equal → start turning slowly
    // - At progress=0.5, sin(pi/2)=1 → maximal offset → middle of turn, robot turns fastest
    // - At progress=1, sin(pi)=0 → offset returns to 0 → smooth deceleration near target
    float delta = Delta_max * sin(pi * progress);

    int vL = v_max - delta;  // left motor slows for left turn, speeds up for right turn
    int vR = v_max + delta;  // right motor speeds up for left turn, slows for right turn

    motors.setSpeeds(vL, vR);  // apply smooth motor speeds

    delay(50);                 // control loop interval ~50 Hz
    t = millis() - startTime;  // update elapsed time

    // ---------------- GYRO-BASED STOPPING ----------------
    // Check current heading using TurnByDegree
    // - This ensures that even if the sine profile would "overshoot",
    //   the robot stops precisely when the gyro target angle is reached
    angleBreakpointReached = TurnByDegree(userDegreeTurn3, userDirection, runOnce);
    runOnce = false;  // targets initialized only on first call
  }
  motors.setSpeeds(0, 0);  // stop motors precisely at the target angle
}

// Arduino setup: initialize Serial, sensors, motors, display
void setup() {
  Serial.begin(9600);
  turnSensorSetup();  // configure gyro / IMU
  delay(500);
  turnSensorReset();       // reset gyro heading to 0
  OLED.clear();            // clear display
  motors.setSpeeds(0, 0);  // ensure motors stopped
  delay(1000);
}

// Main loop: execute one 90° left turn and drive forward once
void loop() {
  DriveTurn(90, 1);        // perform 90° left turn (1 = left)
  motors.setSpeeds(0, 0);  // stop motors

  while (1)
    ;  // infinite loop to end program
}

// Should be called in setup.
void turnSensorSetup() {
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
  for (uint16_t i = 0; i < 1024; i++) {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {
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
  while (!buttonA.getSingleDebouncedRelease()) {
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
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate() {
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