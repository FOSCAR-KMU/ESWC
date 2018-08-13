#include <signal.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>
#include <stdio.h>
#include "car_lib.h"

#include "util.h"

#include "display-kms.h"
#include "v4l2.h"
#include "vpe-common.h"
#include "drawing.h"
#include "input_cmd.h"
#include "exam_cv.h"


#define CAPTURE_IMG_W       1280
#define CAPTURE_IMG_H       720
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy"

#define VPE_OUTPUT_W        320
#define VPE_OUTPUT_H        180

#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3)
#define VPE_OUTPUT_FORMAT       "bgr24"

#define OVERLAY_DISP_FORCC      FOURCC('A','R','2','4')
#define OVERLAY_DISP_W          480
#define OVERLAY_DISP_H          272

#define TIME_TEXT_X             385 //320
#define TIME_TEXT_Y             260 //240
#define TIME_TEXT_COLOR         0xffffffff //while

#define DUMP_MSGQ_KEY           1020
#define DUMP_MSGQ_MSG_TYPE      0x02


////////////////////////////////////////////////////////////////////////////////////

#define MIN_DIST 15

volatile bool stop = false;
volatile int angle = 1500;

volatile int outbreak_flag = 0; //돌발 표지판 flag
volatile int rotary_flag = 0;
// 0 : 정지선 진입전
// 1 : 회전교차로 진입전
// 2 : 회전교차로 주행중

volatile int rotary_enter_count = 0; //로타리 진입전 이진화 픽셀값 Count

volatile int passing_lane_number = -1;


volatile float slope[2] = {};


typedef enum {
    DUMP_NONE,
    DUMP_CMD,
    DUMP_READY,
    DUMP_WRITE_TO_FILE,
    DUMP_DONE
}DumpState;

typedef struct _DumpMsg{
    long type;
    int  state_msg;
}DumpMsg;

struct thr_data {
    struct display *disp;
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct buffer **input_bufs;

    DumpState dump_state;
    unsigned char dump_img_data[VPE_OUTPUT_IMG_SIZE];

    int msgq_id;
    bool bfull_screen;
    bool bstream_start;
    pthread_t threads[3];
};

/**
  * @brief  Alloc vpe input buffer and a new buffer object
  * @param  data: pointer to parameter of thr_data
  * @retval none
  */
static int allocate_input_buffers(struct thr_data *data)
{
    int i;
    struct vpe *vpe = data->vpe;

    data->input_bufs = calloc(NUMBUF, sizeof(*data->input_bufs));
    for(i = 0; i < NUMBUF; i++) {
        data->input_bufs[i] = alloc_buffer(vpe->disp, vpe->src.fourcc, vpe->src.width, vpe->src.height, false);
    }
    if (!data->input_bufs)
        ERROR("allocating shared buffer failed\n");

    for (i = 0; i < NUMBUF; i++) {
        /** Get DMABUF fd for corresponding buffer object */
        vpe->input_buf_dmafd[i] = omap_bo_dmabuf(data->input_bufs[i]->bo[0]);
        data->input_bufs[i]->fd[0] = vpe->input_buf_dmafd[i];
    }
    return 0;
}

/**
  * @brief  Free vpe input buffer and destroy a buffer object
  * @param  buffer: pointer to parameter of buffer object
                  n : count of buffer object
                  bmultiplanar : multipanar value of buffer object
  * @retval none
  */
static void free_input_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        if (buffer[i]) {
            close(buffer[i]->fd[0]);
            omap_bo_del(buffer[i]->bo[0]);
            if(bmultiplanar){
                close(buffer[i]->fd[1]);
                omap_bo_del(buffer[i]->bo[1]);
            }
        }
    }
    free(buffer);
}

/**
  * @brief  Draw operating time to overlay buffer.
  * @param  disp: pointer to parameter of struct display
                  time : operate time (ms)
  * @retval none
  */
static void draw_operatingtime(struct display *disp, uint32_t time)
{
    FrameBuffer tmpFrame;
    unsigned char* pbuf[4];
    char strtime[128];

    memset(strtime, 0, sizeof(strtime));

    sprintf(strtime, "%03d(ms)", time);

    if(get_framebuf(disp->overlay_p_bo, pbuf) == 0) {
        tmpFrame.buf = pbuf[0];
        tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc);//FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
        tmpFrame.stride = disp->overlay_p_bo->pitches[0];//tmpFrame.width*3;

        drawString(&tmpFrame, strtime, TIME_TEXT_X, TIME_TEXT_Y, 0, TIME_TEXT_COLOR);
    }
}

/**
  * @brief  Handle houht transform with opencv api
  * @param  disp: pointer to parameter of struct display
                 cambuf: vpe output buffer that converted capture image
  * @retval none
  */
static void outBreakMission(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        if(outbreak(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H)){
          MSG(" stop ");
          stop = true;
        }
        else{
          MSG(" start ");
        }


        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}

static void drive(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);


        if(rotary_flag == 2){
          angle = rotary_line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope);
        }
        else{
          angle = line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope);

        }


        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}

static void choosing_lane(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0 && passing_lane_number == -1) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        passing_lane_number = passing_lane_check(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}

static void rotary_enter(struct display *disp, struct buffer *cambuf)
{
    unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
    uint32_t optime;
    struct timeval st, et;

    unsigned char* cam_pbuf[4];
    if(get_framebuf(cambuf, cam_pbuf) == 0) {
        memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);

        gettimeofday(&st, NULL);

        rotary_enter_count = enter_the_rotary(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope);

        gettimeofday(&et, NULL);
        optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
        draw_operatingtime(disp, optime);
    }
}



/**
  * @brief  Camera capture, capture image covert by VPE and display after sobel edge
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * capture_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    struct v4l2 *v4l2 = data->v4l2;
    struct vpe *vpe = data->vpe;
    struct buffer *capt;
    bool isFirst = true;
    int index;
    int count = 0;
    int i;

    v4l2_reqbufs(v4l2, NUMBUF);

    // init vpe input
    vpe_input_init(vpe);

    // allocate vpe input buffer
    allocate_input_buffers(data);

    if(vpe->dst.coplanar)
        vpe->disp->multiplanar = true;
    else
        vpe->disp->multiplanar = false;
    printf("disp multiplanar:%d \n", vpe->disp->multiplanar);

    // init /allocate vpe output
    vpe_output_init(vpe);
    vpe_output_fullscreen(vpe, data->bfull_screen);

    for (i = 0; i < NUMBUF; i++)
        v4l2_qbuf(v4l2,vpe->input_buf_dmafd[i], i);

    for (i = 0; i < NUMBUF; i++)
        vpe_output_qbuf(vpe, i);

    v4l2_streamon(v4l2);
    vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

    vpe->field = V4L2_FIELD_ANY;

    while(1) {
        index = v4l2_dqbuf(v4l2, &vpe->field);
        vpe_input_qbuf(vpe, index);

        if (isFirst) {
            vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
            isFirst = false;
            MSG("streaming started...");
            data->bstream_start = true;
        }

        index = vpe_output_dqbuf(vpe);
        capt = vpe->disp_bufs[index];


        //////////////////////////미션 실행 함수들////////////////////////////


        // if(rotary_flag == 1){
        //   rotary_enter(vpe->disp, capt);
        // }
        // else{
        //   drive(vpe->disp, capt);
        // }

        if(passing_lane_number == -1) {
            choosing_lane(vpe->disp, capt);
        }
        else {
            printf("인직완료\n");
        }


        ///////////////////////////////////////////////////////////////////
        if (disp_post_vid_buffer(vpe->disp, capt, 0, 0, vpe->dst.width, vpe->dst.height)) {
            ERROR("Post buffer failed");
            return NULL;
        }
        update_overlay_disp(vpe->disp);

        if(data->dump_state == DUMP_READY) {
            DumpMsg dumpmsg;
            unsigned char* pbuf[4];

            if(get_framebuf(capt, pbuf) == 0) {
                switch(capt->fourcc) {
                    case FOURCC('Y','U','Y','V'):
                    case FOURCC('B','G','R','3'):
                        memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_IMG_SIZE);
                        break;
                    case FOURCC('N','V','1','2'):
                        memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H); // y data
                        memcpy(data->dump_img_data+VPE_OUTPUT_W*VPE_OUTPUT_H, pbuf[1], VPE_OUTPUT_W*VPE_OUTPUT_H/2); // uv data
                        break;
                    default :
                        MSG("DUMP.. not yet support format : %.4s\n", (char*)&capt->fourcc);
                        break;
                }
            } else {
                MSG("dump capture buf fail !");
            }

            dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
            dumpmsg.state_msg = DUMP_WRITE_TO_FILE;
            data->dump_state = DUMP_WRITE_TO_FILE;
            if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                MSG("state:%d, msg send fail\n", dumpmsg.state_msg);
            }
        }

        vpe_output_qbuf(vpe, index);
        index = vpe_input_dqbuf(vpe);
        v4l2_qbuf(v4l2, vpe->input_buf_dmafd[index], index);

    }

    MSG("Ok!");
    return NULL;
}

void * capture_dump_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    FILE *fp;
    char file[50];
    struct timeval timestamp;
    struct tm *today;
    DumpMsg dumpmsg;

    while(1) {
        if(msgrcv(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), DUMP_MSGQ_MSG_TYPE, 0) >= 0) {
            switch(dumpmsg.state_msg) {
                case DUMP_CMD :
                    gettimeofday(&timestamp, NULL);
                    today = localtime(&timestamp.tv_sec);
                    sprintf(file, "dump_%04d%02d%02d_%02d%02d%02d.%s", today->tm_year+1900, today->tm_mon+1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec,VPE_OUTPUT_FORMAT);
                    data->dump_state = DUMP_READY;
                    MSG("file name:%s", file);
                    break;

                case DUMP_WRITE_TO_FILE :
                    if((fp = fopen(file, "w+")) == NULL){
                        ERROR("Fail to fopen");
                    } else {
                        fwrite(data->dump_img_data, VPE_OUTPUT_IMG_SIZE, 1, fp);
                    }
                    fclose(fp);
                    data->dump_state = DUMP_DONE;
                    break;

                default :
                    MSG("dump msg wrong (%d)", dumpmsg.state_msg);
                    break;
            }
        }
    }

    return NULL;
}

/**
  * @brief  handling an input command
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * input_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;

    char cmd_input[128];
    char cmd_ready = true;

    while(!data->bstream_start) {
        usleep(100*1000);
    }

    MSG("\n\nInput command:");
    MSG("\t dump  : display image(%s, %dx%d) dump", VPE_OUTPUT_FORMAT, VPE_OUTPUT_W, VPE_OUTPUT_H, slope);
    MSG("\n");

    while(1)
    {
        if(cmd_ready == true) {
            /*standby to input command */
            cmd_ready = StandbyInput(cmd_input);     //define in cmd.cpp
        } else {
            if(0 == strncmp(cmd_input,"dump",4)) {
                DumpMsg dumpmsg;
                dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
                dumpmsg.state_msg = DUMP_CMD;
                data->dump_state = DUMP_CMD;
                MSG("image dump start");
                if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                    printf("dump cmd msg send fail\n");
                }

                while(data->dump_state != DUMP_DONE) {
                    usleep(5*1000);
                }
                data->dump_state = DUMP_NONE;
                MSG("image dump done");
            } else {
                printf("cmd_input:%s \n", cmd_input);
            }
            cmd_ready = true;
        }
    }

    return NULL;
}


static struct thr_data* pexam_data = NULL;

/**
  * @brief  handling an SIGINT(CTRL+C) signal
  * @param  sig: signal type
  * @retval none
  */
void signal_handler(int sig)
{
    if(sig == SIGINT) {
        pthread_cancel(pexam_data->threads[0]);
        pthread_cancel(pexam_data->threads[1]);
        pthread_cancel(pexam_data->threads[2]);

        msgctl(pexam_data->msgq_id, IPC_RMID, 0);

        v4l2_streamoff(pexam_data->v4l2);
        vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
        vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

        disp_free_buffers(pexam_data->vpe->disp, NUMBUF);
        free_input_buffers(pexam_data->input_bufs, NUMBUF, false);
        free_overlay_plane(pexam_data->vpe->disp);

        disp_close(pexam_data->vpe->disp);
        vpe_close(pexam_data->vpe);
        v4l2_close(pexam_data->v4l2);

        printf("-- 6_camera_opencv_disp example End --\n");
    }
}


int data_transform(int x, int in_min, int in_max, int out_min, int out_max)
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
  unsigned char status;
  short speed, rspeed;
  unsigned char gain;
  int position, posInit, posDes, posRead;
  short cameraY;
  int channel;
  int data;
  char sensor;
  int tol;
  int i, j;
  char byte = 0x80;


  CarControlInit();

  cameraY = 1630;
  SteeringServoControl_Write(angle);
  CameraXServoControl_Write(angle);
  CameraYServoControl_Write(cameraY);

//   SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
//   speed = 50; // speed set     --> speed must be set when using position controller
//   DesireSpeed_Write(speed);

//   //control on/off
//   status = PositionControlOnOff_Read();
//   printf("PositionControlOnOff_Read() = %d\n", status);
//   PositionControlOnOff_Write(CONTROL);

//   //speed controller gain set(PID제어)
//   //P-gain
//   gain = SpeedPIDProportional_Read();        // default value = 10, range : 1~50
//   printf("SpeedPIDProportional_Read() = %d \n", gain);
//   gain = 20;
//   SpeedPIDProportional_Write(gain);

//   //I-gain
//   gain = SpeedPIDIntegral_Read();        // default value = 10, range : 1~50
//   printf("SpeedPIDIntegral_Read() = %d \n", gain);
//   gain = 20;
//   SpeedPIDIntegral_Write(gain);

//   //D-gain
//   gain = SpeedPIDDifferential_Read();        // default value = 10, range : 1~50
//   printf("SpeedPIDDefferential_Read() = %d \n", gain);
//   gain = 20;
//   SpeedPIDDifferential_Write(gain);

//   PositionControlOnOff_Write(UNCONTROL);

////////////////////////////////////////////////////////////////////////////////



  while(1){
      if(passing_lane_number == -1)
      {
          printf("인식실패\n");
      }
      else
      {
          printf("인식성공 차선 : %d\n",passing_lane_number);
      }

    // // 정지선 인식 = 로타리 시작전
    // if(rotary_flag == 0){

    //   int flag = 0;
    //   sensor = LineSensor_Read();   // black:1, white:0
    //   printf("LineSensor_Read() = ");

    //   for(i = 0; i < 8; i++){
    //     if((i % 4) ==0) printf(" ");
    //     if((sensor & byte)) printf("1");
    //     else{
    //       printf("0");
    //       if(i != 0) flag++;
    //     }
    //     sensor = sensor << 1;
    //   }

    //   printf("\n");
    //   printf("flag : %d\n", flag);

    //   if(flag >= 3){
    //     printf("LineSensor_Read() = STOP! \n");
    //     speed = 0;
    //     DesireSpeed_Write(speed);

    //     rotary_flag = 1;

    //   }
    // }
    // else if(rotary_flag == 1){

    //   printf("%d\n", rotary_enter_count);

    //   if(rotary_enter_count > max_rotary_threshold && rotary_state_flag == 0){
    //     rotary_state_flag = 1;
    //   }
    //   else if(rotary_enter_count < min_rotary_threshold && rotary_state_flag == 1){
    //     printf("rotaty_start!!!");
    //     rotary_flag = 2;
    //     speed = 50;
    //     DesireSpeed_Write(speed);
    //   }
    //   else if(rotary_state_flag == 0){
    //     printf("rotary_wait!!!\n" );
    //   }

    // }
    // else if(rotary_flag == 2){
    //   checkAnotherCar();
    // }


    // SteeringServoControl_Write(angle);

  }




/* register signal handler for <CTRL>+C in order to clean up */
  if(signal(SIGINT, signal_handler) == SIG_ERR) {
    MSG("could not register signal handler");
    closelog();
    exit(EXIT_FAILURE);
  }

  pause();

  return ret;
}
