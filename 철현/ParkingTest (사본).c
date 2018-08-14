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

    char sensor;
    int i, j;
    int tol;
    char byte = 0x80;
    int distance_sensor[7];
    int vertical_flag  = 0  , horizontal_flag = 0;
    CarControlInit();



    int data = DistanceSensor(3);
    int volt = data_transform(data, 0 , 4095 , 0 , 5000);
    int debug = (27.61 / (volt - 0.1696))*1000;


    angle = data_transform(debug , 15 , 37 , -500 , -400);

    angle = 1520 + angle;

    SpeedPIDProportional_Write(gain);

    while(1){

            SteeringServoControl_Write(angle);
            DesireSpeed_Write(-100);

            for( i = 2 ; i <= 6 ; i++){

                data = DistanceSensor(i);

                volt = data_transform(data, 0 , 4095 , 0 , 5000);

                debug = (27.61 / (volt - 0.1696))*1000;

                distance_sensor[i] = debug;
            }

            if(distance_sensor[5] < 40 ){  //주차공간을 찾아서 오른쪽 조향으로 들어감

                DesireSpeed_Write(0);

                usleep(1000000);

                break;
            }

    }

    /*수직주차인지 수평주차인지 판단*/
    //왼쪽공간을 판단하고 좁으면 수직주차 넓으면 수평주차로 인식

    data = DistanceSensor(5);

    volt = data_transform(data, 0 , 4095 , 0 , 5000);

    debug = (27.61 / (volt - 0.1696))*1000;

    printf("%d CM\n" , debug);


    if(debug > 20){

        horizontal_flag = 1;
        vertical_flag = 0;

        printf("수평주차\n");


        /* 들어갈 공간 확보를 위해 살짝 앞으로 뺌*/
        while(1){

            DesireSpeed_Write(50);

            SteeringServoControl_Write(1520);

            int data_horizontal = DistanceSensor(4);

            int volt_horizontal = data_transform(data_horizontal, 0 , 4095 , 0 , 5000);

            int debug_horizontal = (27.61 / (volt_horizontal - 0.1696))*1000;

            printf("%d CM\n" , debug);


            if(debug_horizontal > 35) {
                DesireSpeed_Write(0);

                break;
            }
        }

        /*왼쪽으로 최대조향하고 주차완료될 떄까지 후진*/

        while(1){
            for( i = 2 ; i <= 6 ; i++){
                int data_horizontal = DistanceSensor(i);
                int volt_horizontal = data_transform(data, 0 , 4095 , 0 , 5000);
                int debug_horizontal = (27.61 / (volt - 0.1696))*1000;
                distance_sensor[i] = debug_horizontal;
            }

            SteeringServoControl_Write(2000);

            DesireSpeed_Write(-100);

            if(distance_sensor[4] < 10  ){

                DesireSpeed_Write(0);

                break;
            }
        }


    }

    else{
          horizontal_flag = 0;
          vertical_flag = 1;

          printf("수직주차\n");

          int data_vertical, volt_vertical, debug_vertical;

          while(1){

              data_vertical = DistanceSensor(4);

              volt_vertical = data_transform(data_vertical, 0 , 4095 , 0 , 5000);

              debug_vertical = (27.61 / (volt_vertical - 0.1696))*1000;

              // distance_sensor[4] = debug;

              printf("%d CM\n" , debug_vertical);


              if(debug_vertical < 20){

                printf("뒤거리 %d\n " ,debug_vertical);

                DesireSpeed_Write(0);

                break;
              }

              SteeringServoControl_Write(1520);

              DesireSpeed_Write(-100);


          }
    }
    //주차 완료 신호
   Alarm_Write(ON);

   usleep(1000000);

   Alarm_Write(OFF);


   /* 차선복귀 시작 */

   if(vertical_flag == 1){

     angle = 1520;
     DesireSpeed_Write(80);

     while(1){

       SteeringServoControl_Write(angle);

       for( i = 3 ; i <= 5 ; i++){

         int data_vertical_return = DistanceSensor(i);
         int volt_vertical_return = data_transform(data_vertical_return, 0 , 4095 , 0 , 5000);
         int debug_vertical_return = (27.61 / (volt_vertical_return - 0.1696))*1000;

         distance_sensor[i] = debug_vertical_return;
       }


       if(distance_sensor[4] < 30){ //4번 센서랑 벽까지 거리가 20미만 이면 가운데 맞추면서 나옴

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

      else if( distance_sensor[4] > 30 ){ //30보다 커지면 오른쪽 최대조향

          angle = 1000;
          SteeringServoControl_Write(angle);
      }

      else if(distance_sensor[4] > 200){

          DesireSpeed_Write(0);
          break;
      }
    }
  }
  else if(horizontal_flag == 1){
      printf("뇌정직\n");
  }
}
