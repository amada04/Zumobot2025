#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuOLED.h>

/*Initilizing*/

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors; 

Zumo32U4IMU imu;
PololuSH1106 display(1, 30, 0, 17, 13);


 /*Display Variables*/

unsigned long lastDisplayUpdate = 0;
const int displayInterval = 500;



void setup() {
  Serial.begin(9600);
  Wire.begin();
  // Giving our screen a nice resolution
  display.setLayout21x8();
  // Using all 3 line sensors on the bot 
  lineSensors.initThreeSensors();
  lineSensors.calibrate();

  // Initialize IMU

  imu.enableDefault();  // Enable accelerometer & gyro

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  display.clear();


    delay(1000); // Give sensors time to initialize 
}

void loop() {
  int lcount = encoders.getCountsLeft();
  int rcount = encoders.getCountsRight();

  imu.read();  // Read sensor data

 // Check for white line detection
  if (!avoidWhiteLine()) {
    // No line detected, continue PID control
    if (lcount < targetCounts && rcount < targetCounts) {
      pidLoop(lcount, rcount, targetCounts);
    } else {
      motors.setSpeeds(0, 0);  // Stop when target is reached
    }
  }


  if (millis() - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = millis();
    updateDisplay(lcount, rcount);
  }
}
