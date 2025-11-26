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

void setup() {
  Serial.begin(9600);
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  OLED.clear();
}
int32_t getTurnAngleInDegrees() {
  // 360 degrees corresponds to a full 32-bit rotation (2^32 units)
  // So 1 degree = 2^32 / 360 ≈ 11930465.78 units
  return ((int64_t)turnAngle * 360) / 4294967296;  // = 2^32
}
void TurnByDegree(int32_t userTurn) {
  int32_t startAngle = getTurnAngleInDegrees();
  int32_t currentAngle = startAngle;
  int32_t delta = 0;

  int leftSpeed, rightSpeed;

  // figure out which way to spin
  if (userTurn > 0) {         // turn right (CW)
    leftSpeed = optimalTurnSpeed;
    rightSpeed = -optimalTurnSpeed;
  } else if (userTurn < 0) {  // turn left (CCW)
    leftSpeed = -optimalTurnSpeed;
    rightSpeed = optimalTurnSpeed;
  } else {
    motors.setSpeeds(0, 0);
    return;
  }
  motors.setSpeeds(leftSpeed, rightSpeed);
  unsigned long lastUpdate = millis();

  while (true) {
    turnSensorUpdate();
    currentAngle = getTurnAngleInDegrees();
    delta = currentAngle - startAngle;

    // keep angle in range [-180, 180]
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;

    // stop once we hit or pass target
    if ((userTurn > 0 && delta >= userTurn) || (userTurn < 0 && delta <= userTurn)) {
      break;
    }

    // print every ~100ms for debug // I had to insert a way to watch how the robot is rotating.
    if (millis() - lastUpdate > 100) {
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
void loop() {
  //To use this func copy all code -loop, when the gyro is done calibrating click button A, to start the program in the loop.
  //Turns Left
  TurnByDegree(-90);
  delay(3000);
  
  //Turns Right
  TurnByDegree(90);
  delay(3000);
  while (1) {
  }
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
  turnRate = -(imu.g.z - gyroOffset);

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