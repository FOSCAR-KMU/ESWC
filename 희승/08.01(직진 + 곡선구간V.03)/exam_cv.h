#ifndef EXAM_CV_H_
#define EXAM_CV_H_




#ifdef __cplusplus
extern "C" {
#endif

bool outbreak(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
int line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, float slope[]);

#ifdef __cplusplus
}
#endif




#endif //EXAM_CV_H_
