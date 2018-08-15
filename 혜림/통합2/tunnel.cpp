#include "AllHeader.h"


const int max_tunnel_threshold = 500;
const int min_tunnel_threshold = 50;

extern "C" {

void tunnel_enter(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        tunnel_enter_count = is_the_tunnel(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}


void tunnel_run()
{
  int interval;
  int channel_left = 6, channel_right = 2;
  int data_left, data_right, volt_left, volt_right, debug_left, debug_right;

  data_right = DistanceSensor(channel_right);
  volt_right = data_transform(data_right, 0 , 4095 , 0 , 5000);
  debug_right = (27.61 / (volt_right - 0.1696))*1000;

  data_left = DistanceSensor(channel_left);
  volt_left = data_transform(data_left, 0 , 4095 , 0 , 5000);
  debug_left = (27.61 / (volt_left - 0.1696))*1000;

  interval = debug_right - debug_left;
  interval  = data_transform(interval , -20 , 20 , -500 , 500);
  angle = 1500 - interval;
  printf("스티어링 값 : %d \n" , angle);
  SteeringServoControl_Write(angle);

  // if(debug1 > 50){
  //     DesireSpeed_Write(0);
  //     break;
  // }
  printf("SteeringServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

}

void mode_tunnel()
{
  //while(1) {
    if((tunnel_flag == 0) && (tunnel_enter_count > max_tunnel_threshold)) { // 터널 진입 전
      tunnel_flag = 1;
      speed = 50;
      DesireSpeed_Write(speed);
      printf("tunnel_start!!!!\n");
    }

    else if(tunnel_flag == 0) {         // 회전 교차로 -> 터널 사이

    }

    else if ((tunnel_flag == 1) && (tunnel_enter_count < min_tunnel_threshold)) { //터널 탈출
      printf("tunnel_end!!!\n");
      tunnel_flag = 2;
      speed = 0; // 멈춤
      DesireSpeed_Write(speed);
    }

    else if(tunnel_flag == 1){    // 터널 주행 중
      printf("tunnel_run!!!\n" );
      if(tunnel_finish()) {
        tunnel_flag = 2;
      }
      tunnel_run();
    }

    else if(tunnel_flag == 2) {// 모드 변경  (차로 추월)
      mode++;
      return 0;
    }
  //}
}
}
