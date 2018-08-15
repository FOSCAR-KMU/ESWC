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

#define OVERPASS_MAX            25  // 고가도로 탈출 거리
#define MIN_DIST                15  // 회전교차로 차 간격
////////////////////////////////////////////////////////////////////////////////////

volatile bool cameraOnOff = 0;
volatile bool driveOnOff = 0;

volatile int mode = 1;

// 1 	출발

// 2 	고가도로 구간 +	돌발(내리막) 구간
// -> 터널 주행 코드
//////////////////////////////

// 3  우선정지 장애물 구간 (위치 랜덤)
// -> 차선인색 + 우선정지 장애

// 4	수평주차
// -> 차선인식 + 주차

// 5  수직주차 (위치랜덤)
// -> 차선인식 + 주차

// 주차끝나면

// -> 차선인식
// -> 정지선 인식(영상처리코드)
// -> 정지선 인식(라인센서)

// 6 	회전 교차로
// -> 진입전 -> 진입 후 주행 -> 종료
/////////////////////////////////

// 7 	터널 코스

// 8 	차로 추월 구간

// 9 	신호등 분기점 코스

//////////////// 변수 /////////////////
unsigned char status;
short speed;
unsigned char gain;
int position, posInit, posDes, posRead;
short cameraY;
int channel;
int data;
char sensor;
int tol;
int i, j;
char byte = 0x80;

volatile int angle = 1520;  //조향값
volatile int temp_angle;


volatile int start_flag = 0; // 1
// 0 : 출발 전
// 1 : 출발 전 신호
// 2 : 출발 후

volatile int overpass_flag = 0; // 2
// 0 : 고가도로 출발 전 & 주행 중
// 1 : 고가도로 탈출

volatile int outbreak_flag = 0; // 3
// 0 : 표지판 인식 전
// 1 : 표지판 인식
// 2 : 표지판 인식 후

volatile int parking_flag = 0; // 4
// 0 : 첫번째 장애물 인식
// 1 : 깊이 판단
// 2 : 두번째 장애물 인식
// 3 : 주차 시작

volatile int rotary_flag = 0; // 5
// 0 : 정지선 진입 전
// 1 : 회전교차로 진입 전
// 2 : 회전교차로 주행 중
// 3 : 회전교차로 탈출

volatile int tunnel_flag = 0; // 6
// 0 : 터널 진입 전
// 1 : 터널 주행 중
// 2 : 터널 탈출 후

volatile int overtake_flag = 0; // 7

volatile int traffic_light_flag = 0; // 8


volatile float slope[2] = {};

///////////////////////////고가 도로///////////////////////////

bool overpass_finish();
void mode_overpass();

///////////////////////////돌발 변수///////////////////////////

volatile int outbreak_count = 0; //돌발 표지판 이진화 픽셀값 Count

const int max_outbreak_thrshold = 8000;
const int min_outbreak_thrshold = 500;

void mode_outbreak();

////////////////////////////회전교차로 변수/////////////////////////////

volatile int rotary_enter_count = 0; //로타리 진입전 이진화 픽셀값 Count
volatile int stop_line_count = 0;    //로타리 탈출 확인 정지선 이진화 픽셀값 Count

const int max_rotary_threshold = 500;
const int min_rotary_threshold = 40;

volatile int rotary_ready_flag = 0;
// 0 : 차가 오기 전
// 1 : 차가 지나가는 중

volatile int rotary_finish_flag = 0;
// 0 : 정지선 인식 전
// 1 : 정지선 인식


const int max_stop_line_threshold = 4000;
const int min_stop_line_threshold = 30;

int data_transform(int x, int in_min, int in_max, int out_min, int out_max);
bool rotary_finish();
void checkAnotherCar();
void rotary_mode();

// 0 : 차가 지나가는 도중
// 1 : 차가 완전히 지나갔을 경우

////////////////////////////터널 변수////////////////////////////////

//volatile int tunnel_enter_count = 0; //터널 진입전 이진화 픽셀값 Count
volatile int tunnel_dist = 0;

const int start_tunnel_threshold = 25;
const int finish_tunnel_threshold = 20;

int tunnel_calc();
void tunnel_run();
void mode_tunnel();

///////////////////////////////////////////////////////////////////

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

        outbreak_count = outbreak(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);

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

        //drive mode
        //1 : normal drive
        //2 : rotary drive
        if(rotary_flag == 2){
          temp_angle = line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope, 2);
          stop_line_count = stop_line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);

        }
        else{
          temp_angle = line_detector(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H, slope, 1);
        }

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

        rotary_enter_count = enter_the_rotary(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);

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
        if(cameraOnOff == 0)
          continue;

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
        switch(mode) {
          case 1 :  // 출발 및 도로주행
            break;
          case 2 :  // 고가도로 구간
            break;
          case 3 :  // 우선정지 장애물
            outBreakMission(vpe->disp, capt);
            driveOnOff = 1;
            break;
          case 4 :  // 주차
            if(parking_flag == 3) driveOnOff = 0;
            break;
          case 5 :  // 회전 교차로
            driveOnOff = 1;
            if(rotary_flag == 1) {   // 정지선 인식 후 다른 차 대기하는 경우
              driveOnOff = 0;
              rotary_enter(vpe->disp, capt);
              break;
            }
            break;
          case 6 :  // 터널
            driveOnOff = 1;
            if(tunnel_flag == 1) driveOnOff = 0;
            break;
          case 7 :  // 차로 추월
            driveOnOff = 1;
            break;
          case 8 : // 신호등
            break;
        }
        if(driveOnOff)
          drive(vpe->disp, capt);
        ///////////// 회전교차로 ///////////
        // if(rotary_flag == 1){ // 정지선에서 대기하는 도중 픽셀값 읽어들임
        //   rotary_enter(vpe->disp, capt);
        // }
        // else{
        //   drive(vpe->disp, capt);
        // }
        ///////////// 터널 //////////////
        // if(tunnel_flag == 0) {
        //   tunnel_enter(vpe->disp, capt);
        // }
        // else {
        //   drive(vpe->disp, capt);
        // }


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

/**
  * @brief  Hough transform the captured image dump and save to file
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
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
    MSG("\t dump  : display image(%s, %dx%d) dump", VPE_OUTPUT_FORMAT, VPE_OUTPUT_W, VPE_OUTPUT_H);
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

/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////미션 함수/////////////////////////////////

///////////////////////////주행 전용 steering write /////////////////////
void driving_write_steer(){


  if(temp_angle != angle){
    angle = temp_angle;
    //회전교차로 진입전 회전교차로 왼쪽 차선 인식 고려 조건문(무조건 우조향)
    // if(mode == 5 && rotary_flag == 0 && angle >= 1600){
    //   angle = 1100;
    // }
    SteeringServoControl_Write(angle);
  }
}

///////////////////////////////////////////////////////////////////////

/************************* 1. 출발 *************************************/

bool start_condition()
{
  int channel_front = 1;
  int data_front, volt_front, debug_front;
  data_front = DistanceSensor(channel_front);
  volt_front = data_transform(data_front, 0 , 4095 , 0 , 5000);
  debug_front = (27.61 / (volt_front - 0.1696))*1000;

  if(start_flag == 0 && debug_front < 10) start_flag = 1;       // 출발전이고 손을 갖다대면 flag 1로
  else if(start_flag == 1 && debug_front > 20) start_flag = 2;  // 손을 치웠을 때 flag 2로
  else if(start_flag == 2) return true;                         // 손을 치우면 출발

  return false;
}

void mode_start()
{
  printf("START\n");
  if(start_condition())
    mode++; // 출발하면 고가도로 모드
}

/************************ 2. 고가도로 & 돌발 내리막 ***********************/

bool overpass_finish()
{
  int channel_left = 6, channel_right = 2;
  int data_left, data_right, volt_left, volt_right, debug_left, debug_right;

  data_right = DistanceSensor(channel_right);
  volt_right = data_transform(data_right, 0 , 4095 , 0 , 5000);
  debug_right = (27.61 / (volt_right - 0.1696))*1000;

  data_left = DistanceSensor(channel_left);
  volt_left = data_transform(data_left, 0 , 4095 , 0 , 5000);
  debug_left = (27.61 / (volt_left - 0.1696))*1000;

  if(debug_right > OVERPASS_MAX && debug_left > OVERPASS_MAX)
    return true;
  return false;
}

void mode_overpass()
{
  speed = 80; // speed set     --> speed must be set when using position controller
  DesireSpeed_Write(speed);
  printf("OVERPASS\n");
  if(overpass_finish()) overpass_flag = 1; // 고가도로 종료 조건
  if(overpass_flag == 0)      // 고가도로 시작 전 + 주행 중
    tunnel_run();
  else if(overpass_flag == 1) {
    mode++;
    cameraOnOff = 1;
  }
}



/************************* 3. 돌발 구간  *******************************/


void mode_outbreak()
{
  printf("OUTBREAK\n");
  printf("red pixel count : %d\n", outbreak_count);

  //돌발 장애물 미션 : 최초 인식
  if(outbreak_flag == 0 && outbreak_count > max_outbreak_thrshold) {
    printf("stop!!outbreak!!\n");
    outbreak_flag= 1;
    speed = 0;
    DesireSpeed_Write(speed);
  }

  //돌발 장애물 미션 : 돌발 표지판이 다시 올라간 후
  else if(outbreak_flag == 1 && outbreak_count < min_outbreak_thrshold) {
    printf("restart!!!\n");
    outbreak_flag = 2;
    speed = 80;
    DesireSpeed_Write(speed);
  }

  else if(outbreak_flag == 2)
    mode++;

}


/************************* 4. 주차 *************************************/


//장애물 인식
int isParking(){
    int data = DistanceSensor(3);
    int volt = data_transform(data, 0 , 4095 , 0 , 5000);
    int distance = (27.61 / (volt - 0.1696))*1000;

    if(distance < 40) return 1;
    else if(distance > 40) return 2;
    else  return 0;
}

int Parking_start(){

      int vertical_flag  = 0  , horizontal_flag = 0;

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

          flag = 1;

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

      while(1){
          for( i = 2 ; i <= 6 ; i++){
               data_horizontal = DistanceSensor(i);
               volt_horizontal = data_transform(data, 0 , 4095 , 0 , 5000);
               debug_horizontal = (27.61 / (volt - 0.1696))*1000;
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

void mode_parking(){

  if(parking_flag == 0 && isParking() == 1){
    printf("첫번째 장애물\n");
    parking_flag = 1;
    // start = clock();
  }
  else if(parking_flag == 1 && isParking() == 2){
    printf("수직주차 구간\n");
    parking_flag = 2;
  }
  else if(parking_flag == 2 && isParking() == 1){
    printf("두번째 장애물\n");
    parking_flag = 3;
    // end = clock();
    // float res = (float)(end - start)/CLOCKS_PER_SEC;
  }
  else if(parking_flag == 3){
    Parking_start();
    parking_flag = -1;
    mode ++;
    DesireSpeed_Write(80);

  }



}


/************************* 5. 회전 교차로 *******************************/

int data_transform(int x, int in_min, int in_max, int out_min, int out_max) // 적외선 센서 데이터 변환 함수
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool rotary_finish()
{
  printf("stop count %d\n", stop_line_count);
  if(stop_line_count > max_stop_line_threshold && rotary_finish_flag == 0){
    rotary_finish_flag = 1;
    printf("stop line check");
  }
  else if(stop_line_count < min_stop_line_threshold && rotary_finish_flag == 1){ //차가 지나간 후에 주행 시작
    printf("rotary_finish!!!");
    return true;
  }
  return false;


}

void checkAnotherCar()
{
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
      // speed = 50;                     // 앞 뒤에 둘다 차가 존재하지 않을 경우
      speed = 20;
    DesireSpeed_Write(speed);
  }

}

void mode_rotary()
{
//  while(1) {
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

      if(rotary_enter_count > max_rotary_threshold && rotary_ready_flag == 0){ //차가 완전히 지나가기 전
        rotary_ready_flag = 1;
      }

      else if(rotary_enter_count < min_rotary_threshold && rotary_ready_flag == 1){ //차가 지나간 후에 주행 시작
        printf("rotaty_start!!!");
        rotary_flag = 2; // 회전교차로 진입 후
        speed = 50;
        DesireSpeed_Write(speed);
      }
      else if(rotary_ready_flag == 0){
        printf("rotary_wait!!!\n" );
      }

    }
    else if(rotary_flag == 2){
      if(rotary_finish()) {
        rotary_flag = 3;
      }
      checkAnotherCar(); // 교차로 주행 중 다른 차 유무 확인
    }
    else if(rotary_flag == 3) { // 교차로 탈출
      mode++;
      rotary_flag = -1; // 교차로 완료 표시
    }


}

/********************************************************************/

/************************* 6.터널 **************************/

// 터널 진입 영상처리
// void tunnel_enter(struct display *disp, struct buffer *cambuf)
// {
//     unsigned char srcbuf[VPE_OUTPUT_W*VPE_OUTPUT_H*3];
//     uint32_t optime;
//     struct timeval st, et;
//
//     unsigned char* cam_pbuf[4];
//     if(get_framebuf(cambuf, cam_pbuf) == 0) {
//         memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W*VPE_OUTPUT_H*3);
//
//         gettimeofday(&st, NULL);
//
//         tunnel_enter_count = is_the_tunnel(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);
//
//         gettimeofday(&et, NULL);
//         optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
//         draw_operatingtime(disp, optime);
//     }
// }

int tunnel_calc() // 터널 벽과의 거리
{
  int channel_left = 6;
  int data_left, volt_left, debug_left;

  data_left = DistanceSensor(channel_left);
  volt_left = data_transform(data_left, 0, 4095, 0, 5000);
  debug_left = (27.61 / (volt_left - 0.1696))*1000;

  return debug_left;
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

  if(debug_left < 50 && debug_right > 50) { // 왼쪽만 인식할 때
    interval = debug_left;
    interval = data_transform(interval, 0, 20, -500, 500);
    angle = 1500 + interval;
  }

  else if(debug_left > 50 && debug_right < 50) { // 오른쪽만 인식할 때
    interval = debug_right;
    interval = data_transform(interval, 0, 20, -500, 500);
    angle = 1500 - interval;
  }

  else if(debug_left < 50 && debug_right < 50) {
    interval = debug_right - debug_left;
    interval = data_transform(interval, -20, 20, -500, 500);
    angle = 1500 - interval;
  }

  printf("스티어링 값 : %d \n" , angle);
  SteeringServoControl_Write(angle);

//  printf("SteeringServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

}

void mode_tunnel()
{
  //while(1) {
  printf("TUNNEL_MODE !!\n");
  tunnel_dist = tunnel_calc();

  if((tunnel_flag == 0) && (tunnel_dist < start_tunnel_threshold)) { // 터널 시작
    tunnel_flag = 1;
    speed = 100;
    DesireSpeed_Write(speed);
    printf("tunnel_start!!!!\n");
  }

  else if((tunnel_flag == 1) && (tunnel_dist > finish_tunnel_threshold)) { // 터널 탈출
    tunnel_flag = 2;
    DesireSpeed_Write(speed);
    printf("tunnel_end!!!\n");
  }

  else if(tunnel_flag == 1) {   // 터널 주행 중
    tunnel_run();
    printf("tunnel_run!!!\n" );
  }

  else if(tunnel_flag == 2) { // 터널 끝 !
    tunnel_flag = -1;
    mode++;
  }
  //}
}

/*******************************************************************/

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


///////////////////////////////////////////////////////////////////////////////

  CarControlInit();

  cameraY = 1630;
  SteeringServoControl_Write(angle);
  CameraXServoControl_Write(angle);
  CameraYServoControl_Write(cameraY);

  SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
  speed = 10; // speed set     --> speed must be set when using position controller
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
// 2 	고가도로 구간  +	돌발(내리막) 구간
// 3 	우선정지 장애물 구간 (위치 랜덤)
// 4 	수평주차, 수직주차 (위치랜덤)
// 5 	회전 교차로
// 6 	터널 코스
// 7 	차로 추월 구간
// 8 	신호등 분기점 코스

  // cameraOnOff = 1;
  // driveOnOff = 1;

  while(1){

    switch(mode) {
      case 1 :  // 출발 및 도로주행
          mode_start();
          break;
      case 2 :  // 고가도로 구간
          mode_overpass();
          break;
      case 3 :  // 우선정지 장애물
          mode_outbreak();
          break;
      case 4 :  // 주차
          mode_parking();
          break;
      case 5 :  // 회전 교차로
          mode_rotary();
          break;
      case 6 :  // 터널
          mode_tunnel();
          break;
      case 7 :  // 차로 추월
          printf("MODE_NEXT");
          break;
      case 8 : // 신호
          break;
    }

    if(driveOnOff){
      driving_write_steer();
    }


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
