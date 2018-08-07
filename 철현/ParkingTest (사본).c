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

 int data_transform(int x, int in_min, int in_max, int out_min, int out_max){
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 }
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

    int volt , debug;
    int isCollision[7]; //2 3 4 5 6

    printf("Parking Test!!!!!!!!!!!!!!!\n");
    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    printf("SpeedControlOnOff_Read() = %d\n", status);
    SpeedControlOnOff_Write(CONTROL);

    DesireSpeed_Write(50);
    SteeringServoControl_Write(1500);
    int flag1 = 0 , flag2 = 0 , vertical_parking = 0;

    while(1){
      data = DistanceSensor(3);
      volt = data_transform(data, 0 , 4095 , 0 , 5000);
      debug = (27.61 / (volt - 0.1696))*1000;
      if(!flag1&&debug < 20){
        flag1 = 1;
        printf("첫번쨰 장애물\n");
      }
      if(flag1 && debug >=25){
         vertical_parking = 1;
         printf("수직주차");
      }
      if(flag1&&vertical_parking&&debug < 20) {
        flag2 = 1;
        printf("두번쨰 장애물\n");
      }
      if(flag1 && flag2 && vertical_parking){
        usleep(500000);
        DesireSpeed_Write(0);
       break;
     }

    }


    data = DistanceSensor(3);
    volt = data_transform(data, 0 , 4095 , 0 , 5000);
    debug = (27.61 / (volt - 0.1696))*1000;

    if(debug < 5){
      angle = 1100;
      printf("벽쪽에 붙음\n");
    }
    else{
      angle = 1000;
      printf("벽쪽에서 떨어짐\n");
    }
    SteeringServoControl_Write(angle);
    SpeedPIDProportional_Write(gain);
    DesireSpeed_Write(-100);


    while(1){

      SteeringServoControl_Write(angle);

      for( i = 2 ; i <= 6 ; i++){
        data = DistanceSensor(i);
        volt = data_transform(data, 0 , 4095 , 0 , 5000);
        debug = (27.61 / (volt - 0.1696))*1000;
        isCollision[i] = debug;
      }
      if(isCollision[5] > 20)
        continue;

        if(isCollision[4] <= 10){
          DesireSpeed_Write(0);
          printf("Parking Finished \n");
          break;
        }


      if( (isCollision[5]-isCollision[3])*(isCollision[5]-isCollision[3]) < 100 ){
        angle = 1500;
        printf("%d\n" , isCollision[5]-isCollision[3]);
        printf("일자조향\n" );
        continue;
      }
      else if( isCollision[3] > isCollision[5] ){
        angle -= 100;
        printf("오른쪽조향\n" );
        continue;
      }
      else if( isCollision[3] == isCollision[5]){
        angle = 1500;
        printf("일자조향\n" );
        continue;
      }
      else if( isCollision[3] < isCollision[5] ){
        printf("왼쪽조향\n");

        angle += 100;
        continue;
      }


    }
      SteeringServoControl_Write(1500);
}
