#include "gyroSensor.h"


float Kp =0;
float Ki = 0;
float Kd = 0;


float bias = 0;
int baseSped = 200;


/*Does this work?
     Good question idk
        We can test it out tomarrow,
      */


void turn(int16_t targetAngle){


  if(targetAngle > 360){
    Serial.print("Error");
    return 0;
  }






  int16_t error = targetAngle;
  int16_t lastError = error;
  float intergral = 0;


  while(abs(error) > 5){

    turnSensorUpdate();

    int16_t currentAngle = (((int32_t)turnAngle >> 16) *360) >> 16;
    error = targetAngle - currentAngle;

    intergral += error;
    intergral = constrain(intergral, -1000, 1000); // Intergral windup sucks

    int16_t derivitive = error - lastError;
    lastError = error;

    int16_t turnSpeed = (Kp*error) + (Ki * intergral) +(Kd * derivitive);

    turnSpeed = constrain(turnSpeed, -baseSpeed, baseSpeed); // limiter so we don't blow up the bot

    motors.setSpeeds(-turnSpeed, turnSpeed);




  }

motors.setSpeed(0, 0);


} 
