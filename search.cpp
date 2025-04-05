
bool isDetected() {
  prox.read();
    // Get readings from all three sensors
  uint8_t left = prox.countsLeftWithLeftLeds();

  uint8_t frontleft = prox.countsFrontWithLeftLeds();
  uint8_t frontright = prox.countsFrontWithRightLeds();

  uint8_t right = prox.countsRightWithRightLeds();
  
  uint8_t diff = right + frontright - frontleft - left;
  if (diff == 0 && frontright > 3 && frontleft > 3)
      return true;
  return false;
}

void search(uint8_t left, uint8_t right, uint8_t frontleft, uint8_t frontright) {
  uint8_t diff = right + frontright - frontleft - left;

  if (isDetected()) {
    knockout();
  }
   else if (diff < 0) {
    turn(-90);
  } 
   else if (diff > 0) {
    turn(90);
}
