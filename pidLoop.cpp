/*PID Variables*/

int errorPrior = 0; 
int intergral_prior = 0;
int iteration_time = 1; 
unsigned long lastTime = 0;

float Kp= 0.2; 
float Ki = 0.001; 
float Kd = 0.3; 
float bias = 0;
int baseSpeed = 250;


void pidLoop(int lcount, int rcount, int target){


  unsigned long currTime =  millis();
  float interation_time = (currTime - lastTime)/1000;  // Puts it in seconds

  lastTime = currTime;

  float t1 = target = lcount; 
  float t2 = target - rcount; 

  float error = (t1+t2)/2;


  //float error = rcount - lcount; 

  float p = Kp * error;

  float i = intergral_prior + (error * iteration_time);
  float iTerm = Ki * i;

  float d = (error - errorPrior) / iteration_time;
  float dTerm = Kd * d; 

  // Our Output 

  float correction = p + iTerm + dTerm; 
  display.gotoXY(0,2);
  display.print(correction);



  /*Setting the values to there previous so it can be a closed loop system*/
  errorPrior = error;

  intergral_prior = i;

  

  int leftSpeed = baseSpeed + correction; 
  int rightSpeed = baseSpeed + correction; 

  leftSpeed = constrain(leftSpeed, -400, 400);
  rightSpeed= constrain(rightSpeed, -400, 400);



  motors.setSpeeds(leftSpeed, rightSpeed);
}
