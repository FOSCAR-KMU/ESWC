#ifndef EXAM_CV_H_
#define EXAM_CV_H_

#ifdef __cplusplus
extern "C" {
#endif

//돌발 표지
bool outbreak(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

//차선 인식(직선 + 곡선))
int line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, float slope[]);

//회전교차로
int enter_the_rotary(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, float slope[]);
int rotary_line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, float slope[]);

//추월차선
int passing_lane_check(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

#ifdef __cplusplus
}
#endif




#endif //EXAM_CV_H_
