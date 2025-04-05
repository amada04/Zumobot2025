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


void turn(int16_t relativeAngle) {
  turnSensorUpdate();

  int16_t startAngle = turnAngle;
  
  int16_t = targetAngle = startAngle + relativeAngle;

  if (targetAngle >= 360) {
    targetAngle -= 360;
  } else if (targetAngle < 0) {
    targetAngle += 360;
  }
  
  on_contact_lost();

  

  int16_t error = targetAngle - startAngle;
  int16_t lastError = error;
  float integral = 0;
  
  while (abs(error) > 5) {
    turnSensorUpdate();
    
    int16_t currentAngle = turnAngle;

    int16_t deltaAngle = currentAngle - startAngle;
    
    error = targetAngle - deltaAngle;
    
    // Handle wrapping around 0/360 degrees, ensure shortest turn.
    if (error > 180) {
      error -= 360;
    } else if (error < -180) {
      error += 360;
    }

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
