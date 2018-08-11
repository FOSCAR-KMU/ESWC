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

//////////////////////////////

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


volatile int mode = 0;

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


volatile bool stop = false;
volatile int angle = 1500;

volatile int start_flag = 0; // 1
volatile int highway_flag = 0; // 2
volatile int outbreak_flag = 0; // 3 (확인필요)
//돌발 표지판 flag
volatile int stop_flag = 0; // 4
volatile int curve_flat = 0; // 5
volatile int parking_flag = 0; // 6
// 0 :
volatile int rotary_flag = 0; // 7
// 0 : 정지선 진입 전
// 1 : 회전교차로 진입 전
// 2 : 회전교차로 주행 중
// 3 : 회전교차로 탈출

volatile int tunnel_flag = 0; // 8
// 0 : 터널 진입 전
// 1 : 터널 주행 중
// 2 : 터널 탈출 후

volatile int overtake_flag = 0; // 9

volatile int traffic_light_flag = 0; // 10

int rotary_state_flag = 0;
// 0 : 차가 지나가는 도중
// 1 : 차가 완전히 지나갔을 경우

volatile int rotary_enter_count = 0; //로타리 진입전 이진화 픽셀값 Count
volatile int tunnel_enter_count = 0; //터널 진입전 이진화 픽셀값 Count

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


//ROTARY
int data_transform(int x, int in_min, int in_max, int out_min, int out_max); // 적외선 센서 데이터 변환 함수
void checkAnotherCar();
void mode_rotary();

//TUNNEL
void tunnel_enter(struct display *disp, struct buffer *cambuf);
void tunnel_run();
void mode_tunnel();
