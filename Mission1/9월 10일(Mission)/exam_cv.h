#ifndef EXAM_CV_H_
#define EXAM_CV_H_

#ifdef __cplusplus
extern "C" {
#endif


//신호등 진입 판단 함수
int traffic_light(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, int centerP[]);


#ifdef __cplusplus
}
#endif




#endif //EXAM_CV_H_
