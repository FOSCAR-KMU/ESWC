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
    int distance_sensor[7]; //2 3 4 5 6

    printf("Parking Test!!!!!!!!!!!!!!!\n");
    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    printf("SpeedControlOnOff_Read() = %d\n", status);
    SpeedControlOnOff_Write(CONTROL);

    DesireSpeed_Write(80);
    SteeringServoControl_Write(1520);
    int flag1 = 0 , flag2 = 0 , vertical_parking = 0;

    while(1){
      data = DistanceSensor(3);
      volt = data_transform(data, 0 , 4095 , 0 , 5000);
      debug = (27.61 / (volt - 0.1696))*1000;
      if(!flag1&&debug < 45){
        flag1 = 1;
        printf("첫번쨰 장애물%d\n" , debug);
      }
      if(flag1 && debug >=60){
         vertical_parking = 1;
         printf("수직주차%d" , debug);
      }
      if(flag1&&vertical_parking&&debug < 50) {
        flag2 = 1;
        printf("두번쨰 장애물%d\n" , debug);
      }
      if(flag1 && flag2 && vertical_parking){
        usleep(500000);
        DesireSpeed_Write(0);
       break;
     }

    }
    printf("주차 시작 \n");
    usleep(1000000);


    data = DistanceSensor(3);
    volt = data_transform(data, 0 , 4095 , 0 , 5000);
    debug = (27.61 / (volt - 0.1696))*1000;
    angle = data_transform(debug , 15 , 35 , 450 , 500);
    angle = 1520 - angle;

    SteeringServoControl_Write(angle);
    SpeedPIDProportional_Write(gain);

    DesireSpeed_Write(-100);

    while(1){

      SteeringServoControl_Write(angle);
     DesireSpeed_Write(-100);

      for( i = 3 ; i <= 5 ; i++){
        data = DistanceSensor(i);
        volt = data_transform(data, 0 , 4095 , 0 , 5000);
        debug = (27.61 / (volt - 0.1696))*1000;
        distance_sensor[i] = debug;
      }

        if(distance_sensor[4] <= 10){
          DesireSpeed_Write(0);
          printf("Parking Finished \n");
          break;
        }


      if(  (distance_sensor[3] > 100 && distance_sensor[5] > 100) ){
        //  DesireSpeed_Write(0);
        angle = 1520;
        printf("%d\n" , distance_sensor[5]-distance_sensor[3]);
        printf("일자조향\n" );
        continue;
      }
      else if((distance_sensor[3] < 100 && distance_sensor[5] < 100)){
        //DesireSpeed_Write(0);
        if( distance_sensor[3] - 4 < distance_sensor[5] ){ //오른쪽에 붙었을 때

          angle = data_transform(distance_sensor[3] - 4  - distance_sensor[5] , -11 , 0 , -200, 0);
          angle = 1520 - angle;
          printf("왼쪽조향\n" );
          continue;
        }
        else if( distance_sensor[3] - 4 > distance_sensor[5] ){ //왼쪽에 붙었을 때

          angle = data_transform(distance_sensor[5] - distance_sensor[3] + 4 , -11 , 0 , -200, 0);
          angle = 1520 + angle;
          printf("오른쪽조향\n" );
          continue;
        }
        else if( distance_sensor[3] - 4 == distance_sensor[5] ){ //왼쪽에 붙었을 때
          angle = 1520 ;
          printf("일자조향\n" );
          continue;
        }
      }

    }

    usleep(1000000);
    int temp;
    angle = 1500;
    DesireSpeed_Write(150);
    SteeringServoControl_Write(angle);
    while(1){

      SteeringServoControl_Write(angle);

      for( i = 2 ; i <= 6 ; i++){
        data = DistanceSensor(i);
        volt = data_transform(data, 0 , 4095 , 0 , 5000);
        debug = (27.61 / (volt - 0.1696))*1000;
        distance_sensor[i] = debug;
      }


      if(distance_sensor[4] < 20){
        int data1 = DistanceSensor(3);
        int volt1 = data_transform(data, 0 , 4095 , 0 , 5000);
        int debug1 = (27.61 / (volt - 0.1696))*1000;

        int data2 = DistanceSensor(5);
        int volt2 = data_transform(data, 0 , 4095 , 0 , 5000);
        int debug2 = (27.61 / (volt - 0.1696))*1000;

        if( debug1 < debug2 ){ //오른쪽에 붙었을 때

          angle = data_transform(debug1 - debug2 , -15 , 0 , -200, 0);
          angle = 1520 - angle;
          printf("왼쪽조향\n" );
          continue;
        }
        else if( debug1 > debug2 ){ //왼쪽에 붙었을 때

          angle = data_transform(debug2 - debug1 , -15 , 0 , -200, 0);
          angle = 1520 + angle;
          printf("오른쪽조향\n" );
          continue;
        }
        else if( debug1 == debug2 ){ //왼쪽에 붙었을 때
          angle = 1520 ;
          printf("일자조향\n" );
          continue;
        }


      }
      else if(distance_sensor[4] >= 20 && distance_sensor[4] <= 30){

        angle = 1520;
      }
      else if( distance_sensor[4] > 30 ){
        angle = 1000;
        SteeringServoControl_Write(angle);
        break;
      }
    }
    while(1){
      for( i = 2 ; i <= 6 ; i++){
        data = DistanceSensor(i);
        volt = data_transform(data, 0 , 4095 , 0 , 5000);
        debug = (27.61 / (volt - 0.1696))*1000;
        distance_sensor[i] = debug;
      }

      if(distance_sensor[4] > 200){
        DesireSpeed_Write(0);
        break;
      }
    }
    SteeringServoControl_Write(1500);
    // for(i = 0 ; i < 1000000 ; i++ ){
    //
    //     data = DistanceSensor(4);
    //     volt = data_transform(data, 0 , 4095 , 0 , 5000);
    //     debug = (27.61 / (volt - 0.1696))*1000;
    //     printf("%d CM\n" , debug);
    // }
}

