#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4OLED oled;
Zumo32U4OLED OLED;
Zumo32U4Buzzer buzzer;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors prox;

int x = 0;
int number = 1;

//------------------------------------------- code from TurnByDegree()_Old 1.dec . 2025 -------------------------------------------
//------------------------------------------- ------------------------------------------- -------------------------------------------

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
  if (userTurn > 0) {  // turn right (CW)
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
//------------------------------------------- ------------------------------------------- -------------------------------------------



void setup() {

  Serial.begin(9600);
  prox.initThreeSensors();
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  OLED.clear();
}

void loop() {


  switch (x) {
    case 0:
      motors.setSpeeds(100, 100);
      if (SensorValue() >= 10 ) {
        x = 1;
      }
      delay(200);
      break;

    case 1:
      motors.setSpeeds(0, 0);
      TurnByDegree(-45);
      x = 2;
      break;

    case 2:
      delay(500);
      if (SensorValue() <= 10) {
        motors.setSpeeds(100, 100);
        delay(2000);
        motors.setSpeeds(0, 0);
        x = 3;
        break;
      } else {
        x = 1;
        number = 2;
        break;
      }

    case 3:
      TurnByDegree((45*number));
      delay(500);
      // int* sensorValues = SensorValue();

      if (SensorValue() <= 10) {  // hvis der ikke er noget foran den, må der være fri bane
        motors.setSpeeds(100, 100);
        delay(2000);  // her skal der regnes ud hvor lang tid den skal kører, udfra hvor langt den stopper væk fra det på linjen.
        motors.setSpeeds(0, 0);
        x = 4;
        break;
      } else {  //hvis der stadig ikke er fri, skal den kører endnu længere ud
        TurnByDegree(-90);
        motors.setSpeeds(100, 100);
        number = 2;
        delay(1000);
        motors.setSpeeds(0, 0);
        break;
      }
      
    case 4:
    Serial.print("I am here");
      TurnByDegree(90);
      delay(500);
      
      if (SensorValue() <= 10) {
        motors.setSpeeds(100, 100);
        delay(2000);              // her skal den stoppe når den møder stregen igen
        motors.setSpeeds(0, 0);
        x = 5;
        break;
      } else {
        TurnByDegree(-90);
        motors.setSpeeds(100, 100);
        delay(1500);
        motors.setSpeeds(0, 0);
        break;
      } 
    case 5:
    x = 6;
    break;

  }


  oled.clear();

  OLED.gotoXY(0, 1);
  OLED.print(x);

  delay(100);
}


int* SensorValue() {

  prox.read();
  static int values[3];  // [0]=front, [1]=left, [2]=right

  // Front
  values[0] = prox.countsFrontWithLeftLeds() + prox.countsFrontWithRightLeds();

  //Side
  values[1] = prox.countsLeftWithLeftLeds() + prox.countsLeftWithRightLeds();
  values[2] = prox.countsRightWithLeftLeds() + prox.countsRightWithRightLeds();

  int front = values[1];

  return front;
}
