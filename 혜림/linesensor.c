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

/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
void main(void)
{
    unsigned char status;
    short speed;
    unsigned char gain;
    int position, posInit, posDes, posRead;
    short angle;
    int channel;
    int data;
    char sensor;
    int i, j;
    int tol;
    char byte = 0x80;

    CarControlInit();
    
#ifdef LINE_TRACE
    // 4. line trace sensor --------------------------------------------------------

  for(j=0; j<100000; ++j) {
    int flag = 0;
    sensor = LineSensor_Read();        // black:1, white:0
    printf("LineSensor_Read() = ");
    for(i=0; i<8; i++)
    {
        if((i % 4) ==0) printf(" ");
        if((sensor & byte)) printf("1");
        else
        {
          printf("0");
          if(i!=0) flag++;
        }
        sensor = sensor << 1;
    }
    printf("\n");
    if(flag >= 3)
      printf("LineSensor_Read() = STOP! \n");
    //  printf("LineSensor_Read() = %d \n", sensor);
  }
#endif
}

