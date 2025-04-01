#include 'Tracker.hpp'

struct Coordinate {
double x;
double y;
double heading;
}
  
class Tracker:

  Tracker () {
    Serial.begin(9600);
    pinMode(trig1, OUTPUT);
    pinMode(echo1, INPUT);
  }
  public:
    Coordinate ping() {

      // create distance, duration, and delta heading variables
      double distance, duration;
      double dHeading;

      // trigger sensor
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      duration = pulseIn(echo, HIGH, 3000);

      // calculate distance
      distance = (duration/2) * 0.0343;

      // if distance is within the max diameter of the field (cm)
      if (distance < 76.2) {

        // comb through all IR sensor combination readings and assign a change in heading
        if ((digitalRead(R_Sensor) == HIGH) && (digitalRead(RF_Sensor) == HIGH) {
          dHeading = 45;
        } else if ((digitalRead(L_Sensor) == HIGH) && (digitalRead(LF_Sensor) == HIGH)) {
          dHeading -45;
        } else if ((digitalRead(RF_Sensor) == HIGH) && (digitalRead(LF_Sensor) == HIGH)) {
          dHeading = 0;
        } else if (digitalRead(R_Sensor) == HIGH) {
          dHeading = 90;
        } else if (digitalRead(RF_Sensor) == HIGH) {
          dHeading = 20;
        } else if (digitalRead(L_Sensor) == HIGH) {
          dHeading = -90;
        } else if (digitalRead(LF_Sensor) == HIGH) {
          dHeading = -20;
        }
      }

      // using calculated distance and dHeading calculate target coordinates
      target.x = FuzeBot.getX() + (distance * sin(heading));
      target.y = FuzeBot.getY() + (distance * cos(heading));
      target.heading = FuzeBot.heading + dHeading;     
    }

    void move_to (Coordinate target) {}
    void track () {
      
  private:
    Position FuzeBot;
    Coordinate target;
