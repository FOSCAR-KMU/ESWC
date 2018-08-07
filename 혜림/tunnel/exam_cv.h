#ifndef EXAM_CV_H_
#define EXAM_CV_H_ 

#ifdef __cplusplus
extern "C" {
#endif

void OpenCV_load_file(char* file, unsigned char* outBuf, int nw, int nh);
void OpenCV_Bgr2RgbConvert(unsigned char* inBuf, int w, int h, unsigned char* outBuf);
void OpenCV_face_detection(char* file, unsigned char* outBuf, int nw, int nh);
void OpenCV_binding_image(char* file1, char* file2, unsigned char* outBuf, int nw, int nh);
void OpenCV_canny_edge_image(char* file, unsigned char* outBuf, int nw, int nh);
void OpenCV_hough_transform(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh);
void OpenCV_merge_image(unsigned char* src1, unsigned char* src2, unsigned char* dst, int w, int h);

#ifdef __cplusplus
}
#endif

#endif //EXAM_CV_H_

