#ifndef EXAM_CV_H_
#define EXAM_CV_H_

#ifdef __cplusplus
extern "C" {
#endif

//돌발 표지판 인식
int outbreak(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

//차선 인식(직선 + 곡선)
int line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, float slope[], int modeNum);

//카메라로 정지선 인식(회전교차로 진입전은 무조건 steer값 1500 이하)
int stop_line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

//회전교차로 진입전 차량 그림자 확인 함수
int enter_the_rotary(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

// 추월 차선 확인
int passing_lane_check(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, int hi[]);

#ifdef __cplusplus
}
#endif




#endif //EXAM_CV_H_
