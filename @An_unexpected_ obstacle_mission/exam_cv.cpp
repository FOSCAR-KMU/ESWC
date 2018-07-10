
#include <iostream>
#include <stdio.h>
#include <string.h>
//#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/gpu/device/utility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#define PI 3.1415926

using namespace std;
using namespace cv;

const Scalar COLOR_BLUE = Scalar(255, 0, 0);
const Scalar COLOR_RED = Scalar(0, 0, 255);
const Scalar COLOR_GREEN = Scalar(170, 170, 0);

const Vec3b HSV_YELLOW_LOWER = Vec3b(10, 20, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(30, 140, 255);

const Vec3b HSV_RED_LOWER = Vec3b(0, 100, 100);
const Vec3b HSV_RED_UPPER = Vec3b(10, 255, 255);
const Vec3b HSV_RED_LOWER1 = Vec3b(160, 100, 100);
const Vec3b HSV_RED_UPPER1 = Vec3b(179, 255, 255);

const Vec3b HSV_BLACK_LOWER = Vec3b(0, 0, 0);
const Vec3b HSV_BLACK_UPPER = Vec3b(180, 255, 50);

const int Unexpected_Obstacle_Threshold = 5000;


extern "C" {

/**
  * @brief  Apply canny edge algorithm and draw it on destination buffer.
  * @param  file: pointer for load image file in local path
             outBuf: destination buffer pointer to apply canny edge
             nw : width value of destination buffer
             nh : height value of destination buffer
  * @retval none
  */
void OpenCV_canny_edge_image(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
    Mat srcGRAY;
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    cvtColor(srcRGB, srcGRAY, CV_BGR2GRAY);
     // �ɴ� �˰����� ����
    Mat contours;
    Canny(srcGRAY, // �׷��̷��� ����
        contours, // ���� �ܰ���
        125,  // ���� ���谪
        350);  // ���� ���谪

    // ������ ȭ�ҷ� �ܰ����� ǥ���ϹǷ� ���� ���� ����
    //Mat contoursInv; // ���� ����
    //threshold(contours, contoursInv, 128, 255, THRESH_BINARY_INV);
    // ���� ���� 128���� ������ 255�� �ǵ��� ����

    cvtColor(contours, contours, CV_GRAY2BGR);

    resize(contours, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  Detect the hough and draw hough on destination buffer.
  * @param  srcBuf: source pointer to hough transform
             iw: width value of source buffer
             ih : height value of source buffer
             outBuf : destination pointer to hough transform
             nw : width value of destination buffer
             nh : height value of destination buffer
  * @retval none
  */
void OpenCV_hough_transform(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{
    Scalar lineColor = Scalar(255,255,255);

    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
    Mat resRGB(ih, iw, CV_8UC3);         //output
    //cvtColor(srcRGB, srcRGB, CV_BGR2BGRA);

    // ĳ�� �˰����� ����
    Mat originImg_left, originImg_right;
    Mat img_hsv;
    Mat contours;
    Mat gray;
    Mat blured1, blured2, blured3;

    // originImg_left = srcRGB(Rect(0, srcRGB.rows/2, srcRGB.cols/2, srcRGB.rows/2));
  	// originImg_right = srcRGB(Rect(srcRGB.cols/2, srcRGB.rows/2, srcRGB.cols/2, srcRGB.rows/2));


    Mat circle_image = srcRGB.clone();
    cvtColor(srcRGB, blured1, CV_BGR2HSV);
    cvtColor(srcRGB, gray, CV_BGR2GRAY);
    // //
    inRange(blured1, HSV_RED_LOWER, HSV_RED_UPPER, blured2);
    inRange(blured1, HSV_RED_LOWER1, HSV_RED_UPPER1, blured3);

    blured2 = blured2 | blured3;


    // //

    Mat edgeImg;

    Canny(blured2, edgeImg, 30, 200);


    vector<Vec3f> circles;

    HoughCircles(blured2, circles, CV_HOUGH_GRADIENT, 2,
             50,  200, 100, 25, 100 );

    // if(circles.size() == 0){
    //   return true;
    // }

    for (size_t i = 0; i < circles.size(); i++){
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);


      circle(srcRGB, center, 3, Scalar(0, 255, 0), 1);
      circle(srcRGB, center, radius, Scalar(0, 0, 0), 3);


    }




    cvtColor(edgeImg, edgeImg, CV_GRAY2BGR);


    resize(edgeImg, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);


}

bool outbreak(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{
    Scalar lineColor = Scalar(255,255,255);

    int redCount = 0;

    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
    Mat resRGB(ih, iw, CV_8UC3);         //output
    //cvtColor(srcRGB, srcRGB, CV_BGR2BGRA);

    // ĳ�� �˰����� ����
    Mat originImg_left, originImg_right;
    Mat img_hsv;
    Mat contours;
    Mat gray;
    Mat blured1, blured2, blured3;

    // originImg_left = srcRGB(Rect(0, srcRGB.rows/2, srcRGB.cols/2, srcRGB.rows/2));
  	// originImg_right = srcRGB(Rect(srcRGB.cols/2, srcRGB.rows/2, srcRGB.cols/2, srcRGB.rows/2));


    Mat circle_image = srcRGB.clone();
    cvtColor(srcRGB, blured1, CV_BGR2HSV);
    cvtColor(srcRGB, gray, CV_BGR2GRAY);
    // //
    inRange(blured1, HSV_RED_LOWER, HSV_RED_UPPER, blured2);
    inRange(blured1, HSV_RED_LOWER1, HSV_RED_UPPER1, blured3);

    blured2 = blured2 | blured3;

    for(int i = 0; i < blured2.cols; i++){
      for(int j = 0; j < blured2.rows; j++){
        if(blured2.at<uchar>(j, i) == 255) redCount++;
      }
    }

    cvtColor(blured2, blured2, CV_GRAY2BGR);



    resize(blured2, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

    if(redCount >= Unexpected_Obstacle_Threshold) return true;
    else return false;


}


// void red_circle(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
//
//   Mat dstRGB(nh, nw, CV_8UC3, outBuf);
//
//   Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
//   Mat resRGB(ih, iw, CV_8UC3);
//
//
// }


/**
  * @brief  Merge two source images of the same size into the output buffer.
  * @param  src1: pointer to parameter of rgb32 image buffer
             src2: pointer to parameter of bgr32 image buffer
             dst : pointer to parameter of rgb32 output buffer
             w : width of src and dst buffer
             h : height of src and dst buffer
  * @retval none
  */
void OpenCV_merge_image(unsigned char* src1, unsigned char* src2, unsigned char* dst, int w, int h)
{
    Mat src1AR32(h, w, CV_8UC4, src1);
    Mat src2AR32(h, w, CV_8UC4, src2);
    Mat dstAR32(h, w, CV_8UC4, dst);

    cvtColor(src2AR32, src2AR32, CV_BGRA2RGBA);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double opacity = ((double)(src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + 3])) / 255.;
            for (int c = 0; opacity > 0 && c < src1AR32.channels(); ++c) {
                unsigned char overlayPx = src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + c];
                unsigned char srcPx = src1AR32.data[y * src1AR32.step + x * src1AR32.channels() + c];
                src1AR32.data[y * src1AR32.step + src1AR32.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }

    memcpy(dst, src1AR32.data, w*h*4);
}

// void base_ROI(Mat& img, Mat& img_ROI) {
//
// 	Point a = Point(0, 0);
// 	Point b = Point(0, img.rows);
// 	Point c = Point(img.cols, img.rows);
// 	Point d = Point(img.cols, 0);
//
// 	vector <Point> Left_Point;
//
// 	Left_Point.push_back(a);
// 	Left_Point.push_back(b);
// 	Left_Point.push_back(c);
// 	Left_Point.push_back(d);
//
// 	Mat roi(img.rows, img.cols, CV_8U, Scalar(0));
//
// 	fillConvexPoly(roi, Left_Point, Scalar(255));
//
// 	Mat filteredImg_Left;
// 	img.copyTo(filteredImg_Left, roi);
//
// 	img_ROI = filteredImg_Left.clone();
//
// }
//
// void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2) {
//
//
// 	float slope = get_slope(p1, p2);
// 	float alphaY = 30.f / sqrt(slope*slope + 1);
// 	float alphaX = slope * alphaY;
//
// 	Point a(p1.x - alphaX, p1.y + alphaY);
// 	Point b(p1.x + alphaX, p1.y - alphaY);
// 	Point c(p2.x + alphaX, p2.y - alphaY);
// 	Point d(p2.x - alphaX, p2.y + alphaY);
//
// 	vector <Point> Left_Point;
//
// 	Left_Point.push_back(a);
// 	Left_Point.push_back(b);
// 	Left_Point.push_back(c);
// 	Left_Point.push_back(d);
//
//
// 	Mat roi(img.rows, img.cols, CV_8U, Scalar(0));
//
// 	fillConvexPoly(roi, Left_Point, Scalar(255));
//
// 	Mat filteredImg_Left;
// 	img.copyTo(filteredImg_Left, roi);
//
//
// 	img_ROI = filteredImg_Left.clone();
//
// }
//
//
// void region_of_interest_L(Mat& img, Mat& img_ROI) {
// 	Point a = Point(0, 0);
// 	Point b = Point(0, img.rows);
// 	Point c = Point(img.cols, img.rows);
// 	Point d = Point(img.cols, 0);
//
// 	vector <Point> Left_Point;
//
// 	Left_Point.push_back(a);
// 	Left_Point.push_back(b);
// 	Left_Point.push_back(c);
// 	Left_Point.push_back(d);
//
//
// 	Mat roi(img.rows, img.cols, CV_8U, Scalar(0));
//
// 	fillConvexPoly(roi, Left_Point, Scalar(255));
//
// 	Mat filteredImg_Left;
// 	img.copyTo(filteredImg_Left, roi);
//
// 	img_ROI = filteredImg_Left.clone();
//
// }
//
// void region_of_interest_R(Mat& img, Mat& img_ROI) {
// 	Point a = Point(0, 0);
// 	Point b = Point(0, img.rows);
// 	Point c = Point(img.cols, img.rows);
// 	Point d = Point(img.cols, 0);
//
// 	vector <Point> Left_Point;
//
// 	Left_Point.push_back(a);
// 	Left_Point.push_back(b);
// 	Left_Point.push_back(c);
// 	Left_Point.push_back(d);
//
//
// 	Mat roi(img.rows, img.cols, CV_8U, Scalar(0));
//
// 	fillConvexPoly(roi, Left_Point, Scalar(255));
//
// 	Mat filteredImg_Left;
// 	img.copyTo(filteredImg_Left, roi);
//
//
// 	img_ROI = filteredImg_Left.clone();
//
// 	Left_Point.push_back(c);
// }
//
// float get_slope(const Point& p1, const Point& p2) {
//
// 	float slope;
//
// 	if (p2.y - p1.y != 0.0) {
// 		slope = ((float)p2.y - (float)p1.y) / ((float)p2.x - (float)p1.x);
// 	}
// 	return slope;
// }
//
// bool Lane_Detector::get_intersectpoint(const Point& AP1, const Point& AP2,
// 	const Point& BP1, const Point& BP2, Point* IP)
// {
// 	double t;
// 	double s;
// 	double under = (BP2.y - BP1.y)*(AP2.x - AP1.x) - (BP2.x - BP1.x)*(AP2.y - AP1.y);
// 	if (under == 0) return false;
//
// 	double _t = (BP2.x - BP1.x)*(AP1.y - BP1.y) - (BP2.y - BP1.y)*(AP1.x - BP1.x);
// 	double _s = (AP2.x - AP1.x)*(AP1.y - BP1.y) - (AP2.y - AP1.y)*(AP1.x - BP1.x);
//
// 	t = _t / under;
// 	s = _s / under;
//
// 	if (t<0.0 || t>1.0 || s<0.0 || s>1.0) return false;
// 	if (_t == 0 && _s == 0) return false;
//
// 	IP->x = AP1.x + t * (double)(AP2.x - AP1.x);
// 	IP->y = AP1.y + t * (double)(AP2.y - AP1.y);
//
// 	return true;
// }
//
// bool hough_left(Mat& img, Point* p1, Point* p2, Mat& srcRGB) {
//
// 	vector<Vec2f> linesL;
//
//   Point point1;
//   Point point2;
//
//   int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
//   int threshold = 80;
//
//   for (int i = 10; i > 0; i--){
//     HoughLines(img, linesL, 1, CV_PI / 180, threshold, 0, 0, 0, CV_PI /2);
//     if (linesL.size() > 1){
//       for (size_t i = 0; i < linesL.size(); i++){
//         count ++;
//         float rho = linesL[i][0];
//         float theta = linesL[i][1];
//         double a = cos(theta), b = sin(theta);
//         double x0 = a * rho, y0 = b * rho;
//
//         int _x1 = int(x0 + 1000*(-b));
//         int _y1 = int(y0 + 1000*(a));
//         int _x2 = int(x0 - 1000*(-b));
//         int _y2 = int(y0 - 1000*(a));
//
//         point1.x = _x1; point1.y = _y1;
//         point2.x = _x2; point2.y = _y2;
//
// 				line(srcRGB, point1, point2, COLOR_RED, 1, CV_AA);
//
//
//         x1 += point1.x;
//         y1 += point1.y;
//
//         x2 += point2.x;
//         y2 += point2.y;
//
//       }
//       break;
//     }
//   }
//
//   if (count != 0){
//
//     x1 /= count;
//     y1 /= count;
//
//     x2 /= count;
//     y2 /= count;
//
//     p1->x = x1; p1->y = y1;
//     p2->x = x2; p2->y = y2;
//
//     return false;
//
//   }
//
//   return true;
// }
//
// bool hough_right(Mat& img, Point* p1, Point* p2, Mat& srcRGB) {
// 	vector<Vec2f> linesR;
//
// 	  Point point1;
// 	  Point point2;
//
// 	  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
// 	  int threshold = 30;
//
// 	  for (int i = 10; i > 0; i--){
// 	    HoughLines(img, linesR, 1, CV_PI / 180, threshold, 0, 0, CV_PI/2, CV_PI);
// 	    if (linesR.size() > 1){
// 	      for (size_t i = 0; i < linesR.size(); i++){
// 	        count ++;
// 	        float rho = linesR[i][0];
// 	        float theta = linesR[i][1];
// 	        double a = cos(theta), b = sin(theta);
// 	        double x0 = a * rho, y0 = b * rho;
//
// 	        int _x1 = int(x0 + 1000*(-b));
// 	        int _y1 = int(y0 + 1000*(a));
// 	        int _x2 = int(x0 - 1000*(-b));
// 	        int _y2 = int(y0 - 1000*(a));
//
// 	        point1.x = _x1; point1.y = _y1;
// 	        point2.x = _x2; point2.y = _y2;
//
// 					line(srcRGB, point1, point2, COLOR_RED, 1, CV_AA);
//
//
//
// 	        x1 += point1.x;
// 	        y1 += point1.y;
//
// 	        x2 += point2.x;
// 	        y2 += point2.y;
//
// 	      }
// 	      break;
// 	    }
// 	  }
//
// 	  if (count != 0){
//
// 	    x1 /= count;
// 	    y1 /= count;
//
// 	    x2 /= count;
// 	    y2 /= count;
//
// 	    p1->x = x1; p1->y = y1;
// 	    p2->x = x2; p2->y = y2;
//
// 	    return false;
//
// 	  }
//
// 	  return true;
// }

}
