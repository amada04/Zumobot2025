/*Line Sensors Variables*/
#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

#define QTR_THRESHOLD 500  // Threshold for detecting a white line
#define REVERSE_SPEED 200   // Speed when reversing
#define TURN_SPEED 200      // Speed when turning
#define FORWARD_SPEED 250   // Speed for moving forward
#define REVERSE_DURATION 200  // Time to reverse (ms)
#define TURN_DURATION 300     // Time to turn (ms)





/* White Line Detection and Avoidance */
bool avoidWhiteLine() {
  lineSensors.read(lineSensorValues);
  
  if (lineSensorValues[0] < QTR_THRESHOLD) {
    // Left sensor detects line, reverse and turn right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED); // Turn right
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    return true;
  } 
  else if (lineSensorValues[NUM_SENSORS - 1] < QTR_THRESHOLD) {
    // Right sensor detects line, reverse and turn left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED); // Turn left
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    return true;
  }
  
  return false;
}
