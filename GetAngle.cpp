#include <Wire.h>
#include <Zumo32U4.h>

const int16_t maxSpeed = 400;
Zumo32U4OLED display;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4IMU imu;

uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint32_t gyroLastUpdate = 0;


/*Use this for when we want to reset the bots orientation */

/* Reset bot's orientation */
void turnSensorReset() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

/* Update gyro readings */
void turnSensorUpdate() {
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  
  uint32_t t = micros();
  uint32_t dt = t - gyroLastUpdate;
  gyroLastUpdate = t;
  int32_t d = (int32_t)turnRate * dt;

         /* Huh?
   So apparently this is convert the d (which is time in microseconds)
   we need to convert them to the degrees, which we used 2^29 for turnAngle
   which is converting gyro digits to degrees per second.
   But there is also a sensitivity of the gyro which is 0.07 dps per digit,
   so we have to multiplty that by those 2 constants to get it in degress, black magic frfr
   reference
  */
  // Convert raw gyro data to angle in degrees
  turnAngle += (int64_t)d * 14680064 / 17578125;
}


/* Calibrate gyro sensor */
void gyroSensorInit() {
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  display.clear();
  display.print(F("Gyro cal"));
  ledYellow(1);
  delay(500);

  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    while (!imu.gyroDataReady()) {}
    imu.readGyro();
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;

  display.clear();
  turnSensorReset();
}

void setup() {
      gyroSensorInit();

}

void loop() {
    turnSensorUpdate();
}
