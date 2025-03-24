/*Line Sensors Variables*/
#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

#define QTR_THRESHOLD 500  // Threshold for detecting a white line
#define REVERSE_SPEED 200   // Speed when reversing
#define TURN_SPEED 200      // Speed when turning
#define FORWARD_SPEED 250   // Speed for moving forward
#define REVERSE_DURATION 200  // Time to reverse (ms)
#define TURN_DURATION 300     // Time to turn (ms)

class LineSensor {
bool avoidWhiteLine();
}
