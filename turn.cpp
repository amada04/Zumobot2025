/* Function to turn robot to a specific angle */
/*IT TURNSSSSSSSSSSSSSSSSSSSSSS YIPPPPY*/

/*When the robot accelerates to much the gyro loses some persetion. But it doesn drift by its self anymore
Now on to prosition tracting
*/

/* PID Constants */
float Kp = 56;
float Ki = 1;
float Kd = 4;
float bias = 0;
int baseSpeed = 200;


void turn(int16_t turnAngle) {
  if (targetAngle > 360) {
    Serial.print("Error");
    return;
  }

  on_contact_lost();

  int16_t currentAngle = (((int32_t)turnAngle >> 16) * 360) >> 16;
  int16_t targetAngle = currentAngle + turnAngle;
  int16_t error = targetAngle;
  int16_t lastError = error;
  float integral = 0;

  while (abs(error) > 5) {
    turnSensorUpdate();

    currentAngle = (((int32_t)turnAngle >> 16) * 360) >> 16;
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
  last_turn_time = millis();
}
