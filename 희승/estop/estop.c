#include <stdio.h>
#include "car_lib.h"

#define estop

void main(void){
  short speed;
  int posInit;
  short angle;

  CarControlInit();

#ifdef estop
  printf("\n            estop!!           \n");
  printf("\n            estop!!           \n");
  printf("\n            estop!!           \n");

  CarLight_Write(ALL_OFF);
  Alarm_Write(OFF);
  Winker_Write(ALL_OFF);

  SpeedControlOnOff_Write(CONTROL);
  speed = 0;
  DesireSpeed_Write(speed);

  posInit = 0;
  EncoderCounter_Write(posInit);

  angle = 1500;
  SteeringServoControl_Write(angle);
  CameraXServoControl_Write(angle);
  CameraYServoControl_Write(angle);

#endif

}
