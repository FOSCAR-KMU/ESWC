#ifndef EXAM_CV_H_
#define EXAM_CV_H_




#ifdef __cplusplus
extern "C" {
#endif

void OpenCV_canny_edge_image(char* file, unsigned char* outBuf, int nw, int nh);
void OpenCV_hough_transform(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
bool outbreak(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
void line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);

void OpenCV_merge_image(unsigned char* src1, unsigned char* src2, unsigned char* dst, int w, int h);

#ifdef __cplusplus
}
#endif




#endif //EXAM_CV_H_
