#ifndef EXAM_CV_H_
#define EXAM_CV_H_

#ifdef __cplusplus
extern "C" {
#endif


//돌발 표지
bool outbreak(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

//차선 인식(직선 + 곡선))
int line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

//카메라로 정지선 인식(회전교차로 진입전은 무조건 steer값 1500 이하)
bool stop_line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
// int stop_line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);


//회전교차로
int enter_the_rotary(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
int rotary_line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

//tunnel 함수
int is_the_tunnel(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

#ifdef __cplusplus
}
#endif

#endif //EXAM_CV_H_
