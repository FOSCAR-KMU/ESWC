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

    short angle = 1500;
    int channel;
    int data1, data2;
    char sensor;
    int i, j;
    int tol;
    char byte = 0x80;

    CarControlInit();

    int volt1 , debug1, volt2 , debug2;
    int isCollision[7]; //2 3 4 5 6
    int interval;
    int pre ;


    printf("Tunnel Driving Test!!!!!!!!!!!!!!!\n");
    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    printf("SpeedControlOnOff_Read() = %d\n", status);
    SpeedControlOnOff_Write(CONTROL);

    DesireSpeed_Write(150);
    SteeringServoControl_Write(angle);

    while(1){
       data1 = DistanceSensor(2);
       volt1 = data_transform(data1, 0 , 4095 , 0 , 5000);
       debug1 = (27.61 / (volt1 - 0.1696))*1000;

      data2 = DistanceSensor(6);
      volt2 = data_transform(data2, 0 , 4095 , 0 , 5000);
      debug2 = (27.61 / (volt2 - 0.1696))*1000;

      //interval = debug1-debug2;
      if(debug2 < 50 && debug1 > 50){ //왼쪽만 인식할 때
        interval = debug2;
        interval = data_transform(interval , 5 , 20 , -500 , 500);
        pre = 1500 + interval;
      }
      else if(debug1 < 50 && debug2 > 50){ //오른쪽만 인식할때
        interval = debug1;
        interval = data_transform(interval , 5 , 20 , -500 , 500);
        pre = 1500 - interval;
      }
      else if(debug1 < 50 && debug2 < 50){
        interval = debug1 - debug2;
        interval = data_transform(interval , 0 , 20 , -500 , 500);
        pre = 1500 - interval;
      }


      if(angle != pre){
        angle = pre;

      printf("스티어링 값 : %d \n" , angle);
      SteeringServoControl_Write(angle);
      }

      if(debug2 > 50){
        DesireSpeed_Write(0);
        break;
      }

    }


  /*  SteeringServoControl_Write(angle);
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
      */
}

