/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <stdio.h>
#include "car_lib.h"

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */
#define LIGHT_BEEP       // to test light and beep
#define POSITION_CONTROL  // to test postion control
#define SPEED_CONTROL     // to test speed control
#define SERVO_CONTROL     // to test servo control(steering & camera position)
#define LINE_TRACE              // to test line trace sensor
#define DISTANCE_SENSOR     // to test distance sensor
#define MIN_DIST 15
/*******************************************************************************
 *  Functions
 *******************************************************************************
 */

 unsigned char status;
 short speed;
 unsigned char gain;
 int position, posInit, posDes, posRead;
 short angle;
 int channel;
 int dist_front, dist_back;
 char sensor;
 int i, j;
 int tol;
 char byte = 0x80;


int data_transform(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void checkAnotherCar()
{
  for(i = 0; i < 10; ++i)
  {
    dist_front = DistanceSensor(1);
    dist_back = DistanceSensor(4);
    int volt_front = data_transform(dist_front, 0 , 4095 , 0 , 5000);
    int volt_back = data_transform(dist_back, 0, 4095, 0, 5000);

    //printf("channel = %d, distance = 0x%04X(%d) \n", channel, data, data);
    dist_front = (27.61 / (volt_front - 0.1696))*1000;
    dist_back = (27.61 / (volt_back - 0.1696)) *1000;
    printf("front : %d cm , back : %d cm \n", dist_front, dist_back);

    // drive condition
    SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
    if(dist_front <= MIN_DIST)        //  앞에 차가 존재할 경우 멈춤
      speed = 0;

    else if(dist_back <= MIN_DIST)
      speed = 120;                    // 뒤에 차가 존재할 경우 속도 최대 (곡선기준)

    else if(dist_back >= MIN_DIST && dist_front >= MIN_DIST)
      speed = 50;                     // 앞 뒤에 둘다 차가 존재하지 않을 경우

    DesireSpeed_Write(speed);
  }

}

void main(void)
{

    CarControlInit();

    // 5. distance sensor --------------------------------------------------------
    printf("\n\n 4. distance sensor\n");

    return 0;
}
