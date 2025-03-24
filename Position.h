#include "Zumobot2025.h"


class Position{

  private:
  Coordinate robotPosition;
  double robotVelocity;
  
  public:
    double getX();
    double getY();
    double getHeading();
    double getVelocity();
    void updatePosition();
};
