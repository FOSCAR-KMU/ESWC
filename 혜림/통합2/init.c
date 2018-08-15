#include "AllHeader.h"


////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  struct v4l2 *v4l2;
  struct vpe *vpe;
  struct thr_data tdata;
  int disp_argc = 3;
  char* disp_argv[] = {"dummy", "-s", "4:480x272", "\0"}; // ���� ���� ���� Ȯ�� �� ó��..
  int ret = 0;


  tdata.dump_state = DUMP_NONE;
  memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data));

  // open vpe
  vpe = vpe_open();
  if(!vpe) {
    return 1;
  }

  // vpe input (v4l cameradata)
  vpe->src.width  = CAPTURE_IMG_W;
  vpe->src.height = CAPTURE_IMG_H;
  describeFormat(CAPTURE_IMG_FORMAT, &vpe->src);

  // vpe output (disp data)
  vpe->dst.width  = VPE_OUTPUT_W;
  vpe->dst.height = VPE_OUTPUT_H;
  describeFormat (VPE_OUTPUT_FORMAT, &vpe->dst);

  vpe->disp = disp_open(disp_argc, disp_argv);
  if (!vpe->disp) {
    ERROR("disp open error!");
    vpe_close(vpe);
    return 1;
  }

  set_z_order(vpe->disp, vpe->disp->overlay_p.id);
  set_global_alpha(vpe->disp, vpe->disp->overlay_p.id);
  set_pre_multiplied_alpha(vpe->disp, vpe->disp->overlay_p.id);
  alloc_overlay_plane(vpe->disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);

  //vpe->deint = 0;
  vpe->translen = 1;

  MSG ("Input(Camera) = %d x %d (%.4s)\nOutput(LCD) = %d x %d (%.4s)",
  vpe->src.width, vpe->src.height, (char*)&vpe->src.fourcc,
  vpe->dst.width, vpe->dst.height, (char*)&vpe->dst.fourcc);

  if (    vpe->src.height < 0 || vpe->src.width < 0 || vpe->src.fourcc < 0 || \
    vpe->dst.height < 0 || vpe->dst.width < 0 || vpe->dst.fourcc < 0) {
    ERROR("Invalid parameters\n");
  }

  v4l2 = v4l2_open(vpe->src.fourcc, vpe->src.width, vpe->src.height);
  if (!v4l2) {
    ERROR("v4l2 open error!");
    disp_close(vpe->disp);
    vpe_close(vpe);
    return 1;
  }

  tdata.disp = vpe->disp;
  tdata.v4l2 = v4l2;
  tdata.vpe = vpe;
  tdata.bfull_screen = true;
  tdata.bstream_start = false;

  if(-1 == (tdata.msgq_id = msgget((key_t)DUMP_MSGQ_KEY, IPC_CREAT | 0666))) {
    fprintf(stderr, "%s msg create fail!!!\n", __func__);
    return -1;
  }

  pexam_data = &tdata;

  ret = pthread_create(&tdata.threads[0], NULL, capture_thread, &tdata);
  if(ret) {
    MSG("Failed creating capture thread");
  }
  pthread_detach(tdata.threads[0]);

  //
  // ret = pthread_create(&tdata.threads[1], NULL, capture_dump_thread, &tdata);
  // if(ret) {
  //     MSG("Failed creating capture dump thread");
  // }
  // pthread_detach(tdata.threads[1]);
  //
  // ret = pthread_create(&tdata.threads[2], NULL, input_thread, &tdata);
  // if(ret) {
  //     MSG("Failed creating input thread");
  // }
  // pthread_detach(tdata.threads[2]);


////////////////////////////////////////////////////////////////////////////////


  CarControlInit();

  cameraY = 1630;
  SteeringServoControl_Write(angle);
  CameraXServoControl_Write(angle);
  CameraYServoControl_Write(cameraY);

  SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
  speed = 50; // speed set     --> speed must be set when using position controller
  DesireSpeed_Write(speed);

  //control on/off
  status = PositionControlOnOff_Read();
  printf("PositionControlOnOff_Read() = %d\n", status);
  PositionControlOnOff_Write(CONTROL);

  //speed controller gain set(PID제어)
  //P-gain
  gain = SpeedPIDProportional_Read();        // default value = 10, range : 1~50
  printf("SpeedPIDProportional_Read() = %d \n", gain);
  gain = 20;
  SpeedPIDProportional_Write(gain);

  //I-gain
  gain = SpeedPIDIntegral_Read();        // default value = 10, range : 1~50
  printf("SpeedPIDIntegral_Read() = %d \n", gain);
  gain = 20;
  SpeedPIDIntegral_Write(gain);

  //D-gain
  gain = SpeedPIDDifferential_Read();        // default value = 10, range : 1~50
  printf("SpeedPIDDefferential_Read() = %d \n", gain);
  gain = 20;
  SpeedPIDDifferential_Write(gain);

  PositionControlOnOff_Write(UNCONTROL);


////////////////////////////////////////////////////////////////////////////////
// 1 	출발 및 도로주행
// 2 	고가도로 구간
// 3 	돌발(내리막) 구간
// 4 	우선정지 장애물 구간 (위치 랜덤)
// 5 	곡선(S)자 주행 코스
// 6 	수평주차, 수직주차 (위치랜덤)
// 7 	회전 교차로
// 8 	터널 코스
// 9 	차로 추월 구간
//10 	신호등 분기점 코스



  while(1){

    switch(int mode) {
      case 1 :  // 출발 및 도로주행
      case 2 :  // 고가도로 구간
      case 3 :  // 돌발(내리막) 구간
      case 4 :  // 우선정지 장애물
      case 5 :  // 곡선 주행
      case 6 :  // 주차
      case 7 :  // 회전 교차로
          mode_rotary();
          break;
      case 8 :  // 터널
          mode_tunnel();
          break;
      case 9 :  // 차로 추월

      case 10 : // 신호

      default :
        drive(vpe->disp, capt);
    }

///////////////////////////회전 교차로//////////////////////////////////

  //   // 정지선 인식 = 회전 교차로 시작전
  //   if(rotary_flag == 0){
  //
  //     int flag = 0;
  //     sensor = LineSensor_Read();   // black:1, white:0
  //     printf("LineSensor_Read() = ");
  //
  //     for(i = 0; i < 8; i++){
  //       if((i % 4) ==0) printf(" ");
  //       if((sensor & byte)) printf("1");
  //       else{
  //         printf("0");
  //         if(i != 0) flag++;
  //       }
  //       sensor = sensor << 1;
  //     }
  //
  //     printf("\n");
  //     printf("flag : %d\n", flag);
  //
  //     if(flag >= 3){
  //       printf("LineSensor_Read() = STOP! \n");
  //       speed = 0;
  //       DesireSpeed_Write(speed);
  //
  //       rotary_flag = 1; //정지선 인식
  //
  //     }
  //   }
  //
  //   // 회전 교차로 진행중 (진입 판단 구간)
  //   else if(rotary_flag == 1){
  //
  //     printf("%d\n", rotary_enter_count);
  //
  //     if(rotary_enter_count > max_rotary_threshold && rotary_state_flag == 0){ //차가 완전히 지나가기 전
  //       rotary_state_flag = 1;
  //     }
  //     else if(rotary_enter_count < min_rotary_threshold && rotary_state_flag == 1){ //차가 지나간 후에 주행 시작
  //       printf("rotaty_start!!!");
  //       rotary_flag = 2; // 회전교차로 진입 후
  //       speed = 50;
  //       DesireSpeed_Write(speed);
  //     }
  //     else if(rotary_state_flag == 0){
  //       printf("rotary_wait!!!\n" );
  //     }
  //
  //   }
  //   else if(rotary_flag == 2){
  //     checkAnotherCar(); // 교차로 주행 중 다른 차 유무 확인
  //   }
  //   SteeringServoControl_Write(angle);
  // }


  ///////////////////////////터널///////////////////////////////////
  // 터널 시작 전
  // if((tunnel_flag == 0) && (tunnel_enter_count > max_tunnel_threshold)) {// 터널 진입 전
  //     tunnel_flag = 1;
  //     speed = 50;
  //     DesireSpeed_Write(speed);
  //     printf("tunnel_start!!!!\n");
  // }
  //
  // else if ((tunnel_flag == 1) && (tunnel_enter_count < min_tunnel_threshold)) { //터널 탈출
  //     printf("tunnel_end!!!\n");
  //     tunnel_flag = 2;
  //     speed = 0; // 멈춤
  //     DesireSpeed_Write(speed);
  // }
  //
  // else if(tunnel_flag == 1){
  //     printf("tunnel_run!!!\n" );
  //     tunnelRun();
  // }
  //

/* register signal handler for <CTRL>+C in order to clean up */
  if(signal(SIGINT, signal_handler) == SIG_ERR) {
    MSG("could not register signal handler");
    closelog();
    exit(EXIT_FAILURE);
  }

  pause();

  return ret;
}
