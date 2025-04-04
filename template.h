#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuOLED.h>
#include <Zumo32U4Buttons.h>


Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4OLED display;
Zumo32U4IMU imu;
Zumo32U4LineSensors lineSensors; 

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

unsigned long lastDisplay = 0;
const int displayInterval = 500;
const int16_t maxSpeed = 400;


uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint32_t gyroLastUpdate = 0;


/* Gyro PID Constants */
float Kp = 56;
float Ki = 1;
float Kd = 4;
float bias = 0;
int baseSpeed = 400;


/*Line Sensors Variables*/
#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

#define QTR_THRESHOLD 1000  // Threshold for detecting a white line
#define REVERSE_SPEED 200   // Speed when reversing
#define TURN_SPEED 200      // Speed when turning
#define FORWARD_SPEED 250   // Speed for moving forward
#define REVERSE_DURATION 200  // Time to reverse (ms)
#define TURN_DURATION 300     // Time to turn (ms)






void setup(){

  
}



void loop(



)







