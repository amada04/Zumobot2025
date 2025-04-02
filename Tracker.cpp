#include "Tracker.hpp"
#include <cmath>

Tracker::Tracker() :
  trig1(12), echo1(13), R_Sensor(2), RF_Sensor(3), L_Sensor(4), LF_Sensor(5),
  CONTACT_PIN(6), LIFTED_PIN(7), WHITE_LINE_PIN(8) {

  Serial.begin(9600);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(R_Sensor, INPUT);
  pinMode(RF_Sensor, INPUT);
  pinMode(L_Sensor, INPUT);
  pinMode(LF_Sensor, INPUT);
  pinMode(CONTACT_PIN, INPUT);
  pinMode(LIFTED_PIN, INPUT);
  pinMode(WHITE_LINE_PIN, INPUT);

  lastTargetTime = 0;
  lastTargetX = 0;
  lastTargetY = 0;
  isTargetFound = false;
  isTargetLost = true;
}

void Tracker::ping() {
  calculateTargetPosition();
  predictTargetPosition();
  updateContactStatus();
  updateLiftedStatus();
}

void Tracker::move_to(Coordinate target) {
  while (true) {
    findLookAhead();
    double curvature = findCurvature();

    if (curvature == -1) {
      Serial.println("Target reached or no lookahead point.");
      break;
    }

    double steering_angle = atan(curvature * WHEELBASE);
    applyMotorControl(steering_angle);

    updateRobotPosition();

    if (isTargetReached(target)) {
      Serial.println("Target reached!");
      stopMotors();
      break;
    }
  }
}

void Tracker::knockout() {
  // max motor speed until reaches white line
}

void Tracker::calculateTargetPosition() {
  double distance, duration;
  double veer = 0;

  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  duration = pulseIn(echo1, HIGH, 30000);

  distance = (duration / 2.0) * 0.0343;

  if (distance < 76.2 && duration > 0) {
    if ((digitalRead(R_Sensor) == HIGH) && (digitalRead(RF_Sensor) == HIGH) && (digitalRead(LF_Sensor) == HIGH)) {
      veer = 10;
    } else if ((digitalRead(L_Sensor) == HIGH) && (digitalRead(LF_Sensor) == HIGH) && (digitalRead(RF_Sensor) == HIGH)) {
      veer = -10;
    } else if ((digitalRead(R_Sensor) == HIGH) && (digitalRead(RF_Sensor) == HIGH)) {
      turnToFace(45);
    } else if ((digitalRead(L_Sensor) == HIGH) && (digitalRead(LF_Sensor) == HIGH)) {
      turnToFace(-45);
    } else if ((digitalRead(RF_Sensor) == HIGH) && (digitalRead(LF_Sensor) == HIGH)) {
      veer = 0;
    } else if (digitalRead(R_Sensor) == HIGH) {
      turnToFace(90);
    } else if (digitalRead(RF_Sensor) == HIGH) {
      veer = 20;
    } else if (digitalRead(L_Sensor) == HIGH) {
      turnToFace(-90);
    } else if (digitalRead(LF_Sensor) == HIGH) {
      veer = -20;
    } else {
      turnToFace(180);
    }
  }

  double veerRad = veer * M_PI / 180.0;
  target.x = FuzeBot.getX() + (distance * cos(FuzeBot.getHeading() + veerRad));
  target.y = FuzeBot.getY() + (distance * sin(FuzeBot.getHeading() + veerRad));
  target.heading = FuzeBot.getHeading() + veerRad;

  if (distance > 0) {
    isTargetFound = true;
    isTargetLost = false;
  } else {
    isTargetFound = false;
    isTargetLost = true;
  }
}

void Tracker::predictTargetPosition() {
  unsigned long currentTime = millis();
  if (lastTargetTime != 0) {
    double deltaTime = (currentTime - lastTargetTime) / 1000.0;
    double targetVelX = (target.x - lastTargetX) / deltaTime;
    double targetVelY = (target.y - lastTargetY) / deltaTime;
    lastTargetTime = currentTime;
    target.x += targetVelX * predictionTime;
    target.y += targetVelY * predictionTime;
  } else {
    lastTargetX = target.x;
    lastTargetY = target.y;
    lastTargetTime = currentTime;
  }
}

void Tracker::findLookAhead() {
  double x = FuzeBot.getX();
  double y = FuzeBot.getY();
  double p1x = target.x;
  double p1y = target.y;

  double a = pow((p1x - x), 2) + pow((p1y - y), 2);
  double c = -pow(lookAheadDistance, 2);

  if (-c / a >= 0) {
    double t = sqrt(-c / a);
    double lx = x + t * (p1x - x);
    double ly = y + t * (p1y - y);

    if (t <= 1 && sqrt(pow(lx - x, 2) + pow(ly - y, 2)) >= 0.001) {
      lookAhead.x = lx;
      lookAhead.y = ly;
    } else {
      lookAhead.x = -1;
      lookAhead.y = -1;
    }
  } else {
    lookAhead.x = -1;
    lookAhead.y = -1;
  }
}

double Tracker::findCurvature() {
  if (lookAhead.x == -1 && lookAhead.y == -1) {
    return -1;
  }

  double x = FuzeBot.getX();
  double y = FuzeBot.getY();
  double heading = FuzeBot.getHeading();

  double alpha = atan2(lookAhead.y - y, lookAhead.x - x) - heading;
  double curvature = 2 * sin(alpha) / lookAheadDistance;
  return curvature;
}

void Tracker::applyMotorControl(double steering_angle) {
  double leftMotorSpeed = 100.0 - steering_angle * 50.0;
  double rightMotorSpeed = 100.0 + steering_angle * 50.0;

  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  setLeftMotorSpeed(leftMotorSpeed);
  setRightMotorSpeed(rightMotorSpeed);
}

void Tracker::stopMotors() {
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
}

void Tracker::updateRobotPosition() {
  FuzeBot.updatePosition();
  double currentHeading = getAccelerometerHeading();
  if (isLifted()) {
    FuzeBot.robotPosition.heading = currentHeading;
  } else {
    // Implement heading fusion here
  }
}

bool Tracker::isTargetReached(Coordinate target) {
  double distance_to_target = sqrt(pow(target.x - FuzeBot.getX(), 2) + pow(target.y - FuzeBot.getY(), 2));
  return distance_to_target < .5;
}

void Tracker::isContact() {
  double currX = FuzeBot.getX();
  double currY = FuzeBot.getY();
  double currHeading = FuzeBot.getHeading();

  double distanceToTarget = sqrt(pow(target.x - currX, 2) + pow(target.y - currY, 2));
  double headingDifference = abs(currHeading - target.heading);
  
  // determine direction of deviation
  double deviation = atan2(target.y - currY, target.x - currX) - currHeading;
    if (deviation < -M_PI) {
        moveToSafePosition(ESCAPE_LEFT);
    } else if (deviation > M_PI) {
        moveToSafePosition(ESCAPE_RIGHT);
    }
}

bool Tracker::isLifted() {

  // read accelerometer data
  m = readMag();

  if (imu.m[2] > (declination + 0.5) || imu.m[2] < (declination - 0.5)) {
    isLifted = true;
  } else {
    isLifted = false;
  }

}

bool Tracker::isTargetDetectedWithin10cm() {
  double distance, duration;

  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  duration = pulseIn(echo1, HIGH, 30000);

  distance = (duration / 2.0) * 0.0343;

  if (distance < 10.0) {
    isTargetDetectedWithin10cm = true;
  } else {
    isTargetDetectedWithin10cm = false;
  }
}

void Tracker::setKnockoutSpeed() {
  motors.setSpeed(255, 255);
  // end when white line is detected
}

void Tracker::moveToSafePosition(EscapeDirection direction) {
  // Implement movement to safe position based on direction
  switch (direction) {
    case ESCAPE_LEFT:
      turnToFace(-90);
      moveForward(30);
    break;
    case ESCAPE_RIGHT:
      turnToFace(90);
      moveForward(30);
    break;
    case ESCAPE_BACK:
      turnToFace(180);
      moveForward(30);
    break;
}

double Tracker::getAccelerometerHeading() {
  // Implement accelerometer heading reading
  return 0.0;
}

void Tracker::setLeftMotorSpeed(int speed) {
  // Implement left motor control
}

void Tracker::setRightMotorSpeed(int speed) {
  // Implement right motor control
}
