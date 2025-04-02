#ifndef TRACKER_HPP
#define TRACKER_HPP

#include "Arduino.h"
#include "Position.h" // Assuming you have a Position class defined

struct Coordinate {
  double x;
  double y;
  double heading;
};

class Tracker {
public:
  Tracker();

  void ping();
  void move_to(Coordinate target);
  void knockout();
  void escape();

  // Set escape conditions
  void setEscapeConditions(bool left, bool right, bool back);

private:
  Position FuzeBot;
  Coordinate target;
  Coordinate lookAhead;
  unsigned long lastTargetTime;
  double lastTargetX;
  double lastTargetY;
  double predictionTime;
  double lookAheadDistance;
  const double WHEELBASE;
  bool isTargetFound;
  bool isTargetLost;

  // Escape conditions
  bool escape_l;
  bool escape_r;
  bool escape_b;

  // Pin definitions
  const int CONTACT_PIN;
  const int LIFTED_PIN;
  const int WHITE_LINE_PIN;

  // Sensor Pins
  const int trig1;
  const int echo1;
  const int R_Sensor;
  const int RF_Sensor;
  const int L_Sensor;
  const int LF_Sensor;

  // Direction constants
  enum EscapeDirection {
    ESCAPE_LEFT,
    ESCAPE_RIGHT,
    ESCAPE_BACK
  };

  void calculateTargetPosition();
  void predictTargetPosition();
  void findLookAhead();
  double findCurvature();
  void applyMotorControl(double steering_angle);
  void stopMotors();
  void updateRobotPosition();
  bool isTargetReached(Coordinate target);
  bool isContact();
  bool isLifted();
  bool isTargetDetectedWithin10cm();
  void setKnockoutSpeed();
  void moveToSafePosition(EscapeDirection direction);
  double getAccelerometerHeading();
  void setLeftMotorSpeed(int speed);
  void setRightMotorSpeed(int speed);
};

#endif // TRACKER_HPP
