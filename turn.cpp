/* Function to turn robot to a specific angle */
/*IT TURNSSSSSSSSSSSSSSSSSSSSSS YIPPPPY*/

/*When the robot accelerates to much the gyro loses some persetion. But it doesn drift by its self anymore
Now on to prosition tracting
*/
void turn(int16_t targetAngle) {
  if (targetAngle > 360) {
    Serial.print("Error");
    return;
  }

  int16_t error = targetAngle;
  int16_t lastError = error;
  float integral = 0;

  while (abs(error) > 5) {
    turnSensorUpdate();

    int16_t currentAngle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    error = targetAngle - currentAngle;

    integral += error;
    integral = constrain(integral, -1000, 1000); // Integral windup prevention

    int16_t derivative = error - lastError;
    lastError = error;

    int16_t turnSpeed = (Kp * error) + (Ki * integral) + (Kd * derivative);
    turnSpeed = constrain(turnSpeed, -baseSpeed, baseSpeed);

    motors.setSpeeds(-turnSpeed, turnSpeed);
  }

  motors.setSpeeds(0, 0);
}
