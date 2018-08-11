/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <stdio.h>
#include "car_lib.h"
#include <time.h>


int Parking_flag1 = 0  , Parking_flag2 = 0;
int vertical_parking = 0;

int isParking();
int verticalParking();

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

 int volt , debug;
 int distance_sensor[7]; //2 3 4 5 6

void main(void)
{

    clock_t start , end;
    float res;
    CarControlInit();

    printf("Parking Test!!!!!!!!!!!!!!!\n");
    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
    printf("SpeedControlOnOff_Read() = %d\n", status);
    SpeedControlOnOff_Write(CONTROL);

      if( Parking_flag1 == 0 && isParking() == 1){
        printf("첫번째 장애물\n");
        Parking_flag1 = 1;
        start = clock();
      }
      else if( Parking_flag1 == 1 && isParking() == 1){
        printf("두번째 장애물\n");
        Parking_flag2 = 1;
        end = clock();
        float res = (float)(end - start)/CLOCKS_PER_SEC;
      }

      if( 0 < res&& res < 10)
        vertical_parking = 1;

      if(vertical_parking == 1){
          verticalParking();
          vertical_parking = 0;
          Parking_flag1 = 0 ; Parking_flag2  = 0;
      }

}

int isParking(){
    int data = DistanceSensor(3);
    int volt = data_transform(data, 0 , 4095 , 0 , 5000);
    int distance = (27.61 / (volt - 0.1696))*1000;

    if(distance < 25) return 1;

    else  return 0;
}

int verticalParking(){


  data = DistanceSensor(3);
  volt = data_transform(data, 0 , 4095 , 0 , 5000);
  debug = (27.61 / (volt - 0.1696))*1000;
  angle = data_transform(debug , 15 , 35 , 450 , 500);
  angle = 1520 - angle;

  SteeringServoControl_Write(angle);
  SpeedPIDProportional_Write(gain);

  DesireSpeed_Write(-100);


  /*후진 주차 시작*/
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
       if( distance_sensor[3]  < distance_sensor[5] ){ //오른쪽에 붙었을 때

         angle = data_transform(distance_sensor[3] - distance_sensor[5] , -15 , 0 , -200, 0);
         angle = 1520 - angle;
         printf("왼쪽조향\n" );
         continue;
       }
       else if( distance_sensor[3] > distance_sensor[5] ){ //왼쪽에 붙었을 때

         angle = data_transform(distance_sensor[5] - distance_sensor[3] , -15 , 0 , -200, 0);
         angle = 1520 + angle;
         printf("오른쪽조향\n" );
         continue;
       }
       else if( distance_sensor[3]  == distance_sensor[5] ){ //왼쪽에 붙었을 때
         angle = 1520 ;
         printf("일자조향\n" );
         continue;
       }
     }

   }


   /*  차선 복귀 */

  angle = 1520;
  DesireSpeed_Write(80);
  SteeringServoControl_Write(angle);

  while(1){

    SteeringServoControl_Write(angle);

    for( i = 3 ; i <= 5 ; i++){
      data = DistanceSensor(i);
      volt = data_transform(data, 0 , 4095 , 0 , 5000);
      debug = (27.61 / (volt - 0.1696))*1000;
      distance_sensor[i] = debug;
    }


    if(distance_sensor[4] < 20){ //4번 센서랑 벽까지 거리가 20미만 이면 가운데 맞추면서 나옴
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
    else if( distance_sensor[4] > 30 ){ //30보다 커지면 오른쪽 최대조향
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
}

