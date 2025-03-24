void updateDisplay(int lcount, int rcount) {
  display.clear();
  display.print("Left:");
  display.print(lcount);
  /* Shifting down one slot to make room for other variables,
   Breaks if you go past 0, 4
   */ 
  display.gotoXY(0, 1);
  display.print("Right:");
  display.print(rcount);

}

