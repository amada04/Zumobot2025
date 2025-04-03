#include "gyroSensor.h"

void gyroSensor() {
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

  while (!buttonA.getSingleDebouncedRelease()) {
    turnSensorUpdate();
    display.gotoXY(0, 0);
    display.print((((int32_t)turnAngle >> 16) * 360) >> 16);
    display.print(F("   "));
  }
  display.clear();
}

void setup() {
  gyroSensor();
}

void loop() {
  turnSensorUpdate();
}
