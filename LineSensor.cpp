#include 'LineSensor.h'
  
class LineSensor {
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
  }
  
  return false;
}
