#include 'Tracker.hpp'

struct Coordinate {
double x;
double y;
double heading;
}
  
class Tracker:

  public:
    Tracker () {
      Serial.begin(9600);
      pinMode(trig1, OUTPUT);
      pinMode(echo1, INPUT);
      pinMode(R_Sensor, INPUT);
      pinMode(RF_Sensor, INPUT);
      pinMode(L_Sensor, INPUT);
      pinMode(LF_Sensor, INPUT);
      lastTargetTime = 0;
      lastTargetX = 0;
      lastTargetY = 0;
    }

    
    void ping() {
      // create distance, duration, and delta heading variables
      double distance, duration;
      double dHeading = 0;

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
        if ((digitalRead(R_Sensor) == HIGH) && (digitalRead(RF_Sensor) == HIGH) && (digitalRead(LF_Sensor) == HIGH)) {
          dHeading = 10;
        } else if ((digitalRead(L_Sensor) == HIGH) && (digitalRead(LF_Sensor) == HIGH) && (digitalRead(RF_Sensor) == HIGH)) {
          dHeading -10;
        } else if ((digitalRead(R_Sensor) == HIGH) && (digitalRead(RF_Sensor) == HIGH)) {
          dHeading = 20;
        } else if ((digitalRead(L_Sensor) == HIGH) && (digitalRead(LF_Sensor) == HIGH)) {
          dHeading = -20;
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

      // get current time
      unsigned long currentTime = millis();
      if (lastTargetTime != 0) {

        // calculate target velocities
        double deltaTime = (currentTime - lastTargetTime) / 1000.0;
        double targetVelX = (target.x - lastTargetX) / deltaTime;
        double targetVelY = (target.y - lastTargetY) / deltaTime;
        lastTargetTime = currentTime;

        // predict future position
        target.x += targetVelX * predictionTime;
        target.y += targetVelY * predictionTime;
      } else {
        
        // initialize lastTarget on the first reading.
        lastTargetX = target.x;
        lastTargetY = target.y;
        lastTargetTime = currentTime;
      }
      
    }

    void move_to (Coordinate target) {

      // while true
      while (true) {

        // get look ahead point and curvature
        findLookAhead();
        double curvature = findCurvature();

        if (curvature == -1) {
          Serial.println("Target reached or no lookahead point.");
          break;
        }

        // calculate angle
        double steering_angle = atan(curvature * 10.0); // Adjust wheelbase
      // ... your motor control code here ...
        Serial.print("Curvature: ");
        Serial.print(curvature);
        Serial.print(" Steering angle: ");
        Serial.println(steering_angle);

        // 
        double delta_x = cos(FuzeBot.getHeading()) * 1.0;
        double delta_y = sin(FuzeBot.getHeading()) * 1.0;
        double delta_heading = steering_angle * 0.1;
        FuzeBot.updatePosition(delta_x, delta_y, delta_heading);
        delay(50);

        double distance_to_target = sqrt(pow(target.x - FuzeBot.getX(), 2) + pow(target.y - FuzeBot.getY(), 2));
        if (distance_to_target < 5.0) {
          Serial.println("Target reached!");
          break;
        }
      }
    }
      
  private:
    Position FuzeBot;
    Coordinate target;
    Coordinate lookAhead;
      unsigned long lastTargetTime;
    double lastTargetX;
    double lastTargetY;
    double predictionTime = 0.2;

    void findLookAhead() {

      // get current x and y
      double x = FuzeBot.getX();
      double y = FuzeBot.geyY();
      
      // get target x and y
      double p1x = target.x;
      double p2x = target.y;

      // calculate discriminant
      double a = pow((p1x - x), 2) + pow((p1y - y), 2);
      double c = -pow(lookAheadDistance, 2);

      if (-c / a >= 0) {
        double t = sqrt(-c/a);

        // calculate intersection
        double lx = x + t * (p1x - x);
        double ly = y + t * (p11y -y);
        
        // assign to look ahead coordinates
        if (t <= 1 && sqrt(pow(lx - x, 2) + pow(ly - y, 2)) >= 0.001) {
          lookAhead.x = lx;
          lookAhead.y = ly;
        } else {
          lookAhead.x = -1;
          lookAhead.y = -1;
        } 
      }

      double findCurvature() {

        if (lookAhead.x == -1 && lookAhead.y == -1) {
          return -1;
        }
        
        double x = FuzeBot.getX();
        double Y = FuzeBot.getY();
        double heading = FuzeBot.getHeading();
        
        double alpha = atan2(ly -y, lx - x) - heading;
        double curvature = 2 * sin(alpha) / lookAheadDistance;
      }
        return curvature;
    }
    
