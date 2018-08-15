
#include "AllHeader.h"

////////////////////////////회전교차로 변수/////////////////////////////

const int max_rotary_threshold = 300;
const int min_rotary_threshold = 40;

int data_transform(int x, int in_min, int in_max, int out_min, int out_max) // 적외선 센서 데이터 변환 함수
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void checkAnotherCar()
{

  short speed;
  int i;
  for(i = 0; i < 10; ++i)
  {
    int dist_front = DistanceSensor(1);
    int dist_back = DistanceSensor(4);
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
      speed = 100;                    // 뒤에 차가 존재할 경우 속도 최대 (곡선기준)

    else if(dist_back >= MIN_DIST && dist_front >= MIN_DIST)
      speed = 50;                     // 앞 뒤에 둘다 차가 존재하지 않을 경우

    DesireSpeed_Write(speed);
  }

}

void mode_rotary()
{
  while(1) {
    // 정지선 인식 = 회전 교차로 시작전
    printf("ROTARY_MODE !! \n");
    if(rotary_flag == 0){

      int flag = 0;
      sensor = LineSensor_Read();   // black:1, white:0
      printf("LineSensor_Read() = ");

      for(i = 0; i < 8; i++){
        if((i % 4) ==0) printf(" ");
        if((sensor & byte)) printf("1");
        else{
          printf("0");
          if(i != 0) flag++;
        }
        sensor = sensor << 1;
      }

      printf("\n");
      printf("flag : %d\n", flag);

      if(flag >= 3){
        printf("LineSensor_Read() = STOP! \n");
        speed = 0;
        DesireSpeed_Write(speed);

        rotary_flag = 1; //정지선 인식
      }
    }

    // 회전 교차로 진행중 (진입 판단 구간)
    else if(rotary_flag == 1){

      printf("%d\n", rotary_enter_count);

      if(rotary_enter_count > max_rotary_threshold && rotary_state_flag == 0){ //차가 완전히 지나가기 전
        rotary_state_flag = 1;
      }

      else if(rotary_enter_count < min_rotary_threshold && rotary_state_flag == 1){ //차가 지나간 후에 주행 시작
        printf("rotaty_start!!!");
        rotary_flag = 2; // 회전교차로 진입 후
        speed = 50;
        DesireSpeed_Write(speed);
      }
      else if(rotary_state_flag == 0){
        printf("rotary_wait!!!\n" );
      }

    }
    else if(rotary_flag == 2){
      if(rotary_finish()) {
        rotary_flag = 3;
      }
      checkAnotherCar(); // 교차로 주행 중 다른 차 유무 확인
    }
    else if(rotary_flag == 3)  {// 교차로 탈출
      mode++;
      return 0;
    }

    SteeringServoControl_Write(angle);
  }
}
