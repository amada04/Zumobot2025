#include <Wire.h>
#include <Zumo32U4.h>
#include <PololuOLED.h>
#include <Zumo32U4Buttons.h>
#include <Zumo32U4ProximitySensors.h>

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


// IR SENSOR CODE 

Zumo32U4ProximitySensors prox;

void setup(){
  prox.initThreeSensors(); // Initialize left, front, and right sensors

  
}



void loop(

  prox.read(); // Perform IR sensing

  // Get readings from all three sensors
  uint8_t leftL = prox.countsLeftWithLeftLeds();
  uint8_t leftR = prox.countsLeftWithRightLeds();

  uint8_t frontL = prox.countsFrontWithLeftLeds();
  uint8_t frontR = prox.countsFrontWithRightLeds();

  uint8_t rightL = prox.countsRightWithLeftLeds();
  uint8_t rightR = prox.countsRightWithRightLeds();


  // Print the readings to the Serial Monitor
  Serial.print("Left (L/R): ");
  Serial.print(leftL);
  Serial.print(" / ");
  Serial.print(leftR);

  Serial.print(" | Front (L/R): ");
  Serial.print(frontL);
  Serial.print(" / ");
  Serial.print(frontR);

  Serial.print(" | Right (L/R): ");
  Serial.print(rightL);
  Serial.print(" / ");
  Serial.println(rightR);

  delay(200); // Small delay between readings


turnSensorUpdate();
  // if (!avoidWhiteLine()) {
 // motors.setSpeeds(250, 250);
    if (buttonB.isPressed())
  {
    delay(1000);
    // Whenever the button is pressed, turn on the yellow LED.
    turn(90);
   delay(1000);
  }
  if(buttonA.isPressed())
  {
    delay(1000);
    turn(180);  
    delay(1000);
  }
  if(buttonA.isPressed())
  {
    delay(1000);
        turn(-1);
    delay(1000);
  }

)







