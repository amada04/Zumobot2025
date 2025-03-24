#include "Position.h"

Position::Position(){
  imu.enableDefault();
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

// returns robot longitudinal displacement in in
double Position::getX(){
  return robotPosition.x;
}

// returns robot lateral displacement in in
double Position::getY(){
  return robotPosition.y;
}

// returns robot heading in radians
double Position::getHeading(){
  return robotPosition.heading;
}

// returns robot velocity in in/s
double Position::getVelocity(){
  return robotVelocity;
}

// updates robot position
void Position::updatePosition(){

  // retrieve encoder values of right and left motors
  currRight = encoders.getCountsLeft();
  currLeft = encoders.getCountsRight();

  // find change in encoder units
  double deltaLeft = currLeft - lastLeft;
  lastLeft = currLeft;
 
  double deltaRight = currRight - lastRight;
  lastRight = currRight;

  // average change of right and left side
  double deltaRobot = (deltaLeft + deltaRight) / 2;

  // retrieve heading
  heading = getHeading();

  // update robot position
  robotPosition.x += (deltaRobot * sin(heading)) / ENCODERS_PER_CM;
  robotPosition.y += (deltaRobot * cos(heading) / ENCODERS_PER_CM;

  // ***** ADD HEADING UPDATES ******

  // ***** IF ROBOT IS LIFTED TERMINATE ENCODER TRACKING OPERATION AND USE ACCELERAOMETER
}
