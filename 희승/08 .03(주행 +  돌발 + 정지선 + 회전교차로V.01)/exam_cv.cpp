#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <string>
#include "util.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#define PI 3.1415926

using namespace std;
using namespace cv;

const Scalar COLOR_BLUE = Scalar(255, 0, 0);
const Scalar COLOR_RED = Scalar(0, 0, 255);
const Scalar COLOR_GREEN = Scalar(170, 170, 0);


const Vec3b RGB_WHITE_LOWER = Vec3b(100, 100, 170);
const Vec3b RGB_WHITE_UPPER = Vec3b(255, 255, 255);

const Vec3b HSV_YELLOW_LOWER = Vec3b(20, 40, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(50, 255, 255);

const Vec3b HSV_WHITE_LOWER = Vec3b(80, 0, 180);
const Vec3b HSV_WHITE_UPPER = Vec3b(180, 60, 255);

const Vec3b HSV_RED_LOWER = Vec3b(0, 100, 100);
const Vec3b HSV_RED_UPPER = Vec3b(10, 255, 255);
const Vec3b HSV_RED_LOWER1 = Vec3b(160, 100, 100);
const Vec3b HSV_RED_UPPER1 = Vec3b(179, 255, 255);

const Vec3b HSV_BLACK_LOWER = Vec3b(0, 0, 0);
const Vec3b HSV_BLACK_UPPER = Vec3b(180, 255, 50);

const Vec3b YUV_LOWER = Vec3b(10, 110, 120);
const Vec3b YUV_UPPER = Vec3b(70, 130, 140);

const int Unexpected_Obstacle_Threshold = 5000;
const int stop_line_threshold = 300;

bool get_intersectpoint(const Point& AP1, const Point& AP2,	const Point& BP1, const Point& BP2, Point* IP);
void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2);
float get_slope(const Point& p1, const Point& p2);
bool hough_left(Mat& img, Mat& srcRGB, Point* p1, Point* p2);
bool hough_right(Mat& img, Mat& srcRGB, Point* p1, Point* p2);
int curve_detector(Mat& leftROI, Mat& rightROI);
int rotary_curve_detector(Mat& leftROI, Mat& rightROI);
bool hough_curve(Mat& img, Mat& srcRGB, Point* p1, Point* p2);
float data_transform(float x, float in_min, float in_max, float out_min, float out_max);


extern "C" {


int outbreak(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
{

    int redCount = 0;

    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
    Mat resRGB(ih, iw, CV_8UC3);         //output

    Mat hsvImg;
    Mat binaryImg1, binaryImg2;

    cvtColor(srcRGB, hsvImg, CV_BGR2HSV);

    inRange(hsvImg, HSV_RED_LOWER, HSV_RED_UPPER, binaryImg1);
    inRange(hsvImg, HSV_RED_LOWER1, HSV_RED_UPPER1, binaryImg2);

    binaryImg1 = binaryImg1 | binaryImg2;

    for(int i = 0; i < binaryImg1.cols; i++){
      for(int j = 0; j < binaryImg1.rows; j++){
        if(binaryImg1.at<uchar>(j, i) == 255) redCount++;
      }
    }

    cvtColor(binaryImg1, binaryImg1, CV_GRAY2BGR);

    // resize(binaryImg1, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

    return redCount;

}



int line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, float slope[]){

  int angle = 1500;
  Point p1, p2, p3, p4, p5;

  volatile bool left_error = true;
  volatile bool right_error = true;


  Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
  Mat dstRGB(nh, nw, CV_8UC3, outBuf); //output

  Mat resRGB(ih, iw, CV_8UC3); //reuslt

  // ĳ ˰
  Mat leftROI, rightROI;
  Mat hsvImg1, hsvImg2;
  Mat binaryImg1, binaryImg2;
  Mat cannyImg1, cannyImg2;

  leftROI = srcRGB(Rect(0, srcRGB.rows/3 * 2, srcRGB.cols/2, srcRGB.rows/3));
  rightROI = srcRGB(Rect(srcRGB.cols/2, srcRGB.rows/3 * 2, srcRGB.cols/2, srcRGB.rows/3));

  hconcat(leftROI, rightROI, resRGB);

  resize(resRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);



  cvtColor(leftROI, hsvImg1, CV_BGR2HSV);
  cvtColor(rightROI, hsvImg2, CV_BGR2HSV);

  inRange(hsvImg1, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg1);
  inRange(hsvImg2, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg2);


  Canny(binaryImg1, cannyImg1, 150, 250);
  Canny(binaryImg2, cannyImg2, 150, 250);

  left_error = hough_left(cannyImg1, leftROI, &p1, &p2);
  right_error = hough_right(cannyImg2, rightROI, &p3, &p4);




  if(left_error || right_error){
    angle = curve_detector(leftROI, rightROI);
  }
  else{

    line(leftROI, p1, p2, COLOR_BLUE, 3, CV_AA);
    line(rightROI, p3, p4, COLOR_BLUE, 3, CV_AA);



/////////////////////////////소실점 주행////////////////////////////////////////////////
   get_intersectpoint(p1, p2, Point(p3.x + 160, p3.y), Point(p4.x + 160, p4.y), &p5);

//////////////////////////////////////////////////////////////////////////////////////

  float steer;
  float x_Difference = 160.0 - p5.x;

  if(x_Difference > 0.0){

    steer = 1520.0 + 3.2 * x_Difference;
  }
  else if(x_Difference < 0.0){
    steer = 1520.0 + (-3.2) * x_Difference;
  }
  else{
    steer = 1520.0;
  }







  angle = steer;

    angle = steer;



  }

  // resize(resRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

  if(angle > 2000){
    angle = 2000;
  }
  else if(angle < 1000){
    angle = 1000;
  }


  return angle;

}

bool stop_line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh){


  Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
  Mat dstRGB(nh, nw, CV_8UC3, outBuf); //ouput

  Mat resRGB(ih, iw, CV_8UC3);         //result

  Mat roiImg, hsvImg, binaryImg;

  int cnt = 0;

  roiImg = srcRGB(Rect(0, srcRGB.rows/4 * 3, srcRGB.cols, srcRGB.rows/4));

  cvtColor(roiImg, hsvImg, CV_BGR2HSV);


  inRange(hsvImg, HSV_WHITE_LOWER, HSV_WHITE_UPPER, binaryImg);

  for(int i = 0; i < binaryImg.cols; i++){
    for(int j = 0; j < binaryImg.rows; j++){
      if(binaryImg.at<uchar>(j, i) == 255) cnt ++;
    }
  }

  // cvtColor(binaryImg, binaryImg, CV_GRAY2BGR);
  // resize(binaryImg, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);


  if(cnt >= stop_line_threshold) return true;
  else return false;
}

int enter_the_rotary(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh){


  Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
  Mat dstRGB(nh, nw, CV_8UC3, outBuf); //ouput

  Mat resRGB(ih, iw, CV_8UC3);         //result

  Mat roiImg;
  Mat yuvImg;
  Mat binaryImg;

  int cnt = 0;

  roiImg = srcRGB(Rect(srcRGB.cols/6, srcRGB.rows/3 * 2, srcRGB.cols/6 * 5, srcRGB.rows/3));

  cvtColor(roiImg, yuvImg, CV_BGR2YUV);

  inRange(yuvImg, YUV_LOWER, YUV_UPPER, binaryImg);

  for(int i = 0; i < binaryImg.cols; i++){
    for(int j = 0; j < binaryImg.rows; j++){
      if(binaryImg.at<uchar>(j, i) == 255) cnt ++;
    }
  }

  cvtColor(binaryImg, binaryImg, CV_GRAY2BGR);

  resize(binaryImg, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

  return cnt;

}

float get_slope_curve(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh){


  Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
  Mat dstRGB(nh, nw, CV_8UC3, outBuf); //ouput

  Mat resRGB(ih, iw, CV_8UC3);         //result

  Mat roiImg;
  Mat hsvImg1, binaryImg1;
  Mat cannyImg1;

  Point p1, p2;

  bool error = true;

  // hconcat(leftROI, rightROI, originImg);

  roiImg = srcRGB(Rect(0, srcRGB.rows/3 * 2, srcRGB.cols, srcRGB.rows/3));

  resize(roiImg, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);


  cvtColor(roiImg, hsvImg1, CV_BGR2HSV);

  inRange(hsvImg1, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg1);

  Canny(binaryImg1, cannyImg1, 150, 250);


  error = hough_curve(cannyImg1, roiImg, &p1, &p2);

  float slope = get_slope(p1, p2);

  float steer =  data_transform(slope, -0.5, -0.2, 0, 500);

  steer = 1500.0 - steer;

  float x_L, x_R, y = 60;

  // return slope;


   if(slope < 0.0){ // right rotate

    x_L = (y - p1.y + slope * p1.x) / slope;

    float sex = data_transform(x_L, -200, 220, 0.1, 1);

    sex *= steer;


    slope = slope * -1.0;


    float temp = (1.5 - slope) * 100.0;


    int angle = 1500 - temp;

    if(angle < 1000){
      angle = 1000;
    }


    return steer;
  }
  else {

    x_R = (y - p1.y + slope * p1.x) / slope;

    return x_R;

    // if(x_R < 280) return 2000;

    float temp = (1.5 - slope) * 100.0;


    int angle = 1500 + temp;

    if(angle > 2000){
      angle = 2000;
    }

    return x_R;

  }






  // return slope;

  // float x_L, x_R, y = 30;
  //
  //
  // if(error){
  //   return 1520;
  // }
  // else if(slope < 0){ // right rotate
  //
  //   x_L = (y - p1.y + slope * p1.x) / slope;
  //
  //   if(x_L > 40) return 1000;
  //
  //   slope = slope * -1.0;
  //
  //
  //   float temp = (1.5 - slope) * 100.0;
  //
  //
  //   int angle = 1500 - temp;
  //
  //   if(angle < 1000){
  //     angle = 1000;
  //   }
  //
  //
  //   return angle;
  // }
  // else {
  //
  //   x_R = (y - p1.y + slope * p1.x) / slope;
  //
  //   if(x_R < 280) return 2000;
  //
  //   float temp = (1.5 - slope) * 100.0;
  //
  //
  //   int angle = 1500 + temp;
  //
  //   if(angle > 2000){
  //     angle = 2000;
  //   }
  //
  //   return angle;
  //
  // }


}

int rotary_line_detector(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh){

  int angle = 1500;
  Point p1, p2, p3, p4, p5;

  volatile bool left_error = true;
  volatile bool right_error = true;


  Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
  Mat dstRGB(nh, nw, CV_8UC3, outBuf); //output

  Mat resRGB(ih, iw, CV_8UC3); //reuslt

  // ĳ ˰
  Mat leftROI, rightROI;
  Mat hsvImg1, hsvImg2;
  Mat binaryImg1, binaryImg2;
  Mat cannyImg1, cannyImg2;

  leftROI = srcRGB(Rect(0, srcRGB.rows/3 * 2, srcRGB.cols/2, srcRGB.rows/3));
  rightROI = srcRGB(Rect(srcRGB.cols/3 * 2, srcRGB.rows/2, srcRGB.cols/2, srcRGB.rows/3));


  hconcat(leftROI, rightROI, resRGB);

  resize(resRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

////////////////////////disp//////////////////////////////////////////
  Mat checkROI = srcRGB(Rect(0, srcRGB.rows/16 * 13, srcRGB.cols, srcRGB.rows/16 * 3));
  Mat binaryImg3;

  inRange(checkROI, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg3);

  cvtColor(binaryImg3, binaryImg3, CV_GRAY2BGR);

///////////////////////////////////////////////////////////////////////


  // hconcat(leftROI, rightROI, resRGB);


  cvtColor(leftROI, hsvImg1, CV_BGR2HSV);
  cvtColor(rightROI, hsvImg2, CV_BGR2HSV);

  inRange(hsvImg1, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg1);
  inRange(hsvImg2, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg2);


  Canny(binaryImg1, cannyImg1, 150, 250);
  Canny(binaryImg2, cannyImg2, 150, 250);

  left_error = hough_left(cannyImg1, leftROI, &p1, &p2);
  right_error = hough_right(cannyImg2, rightROI, &p3, &p4);

  //곡선
  if(left_error || right_error){
    angle = rotary_curve_detector(leftROI, rightROI);
  }
  //직선
  else{

    // line(leftROI, p1, p2, COLOR_BLUE, 3, CV_AA);
    // line(rightROI, p3, p4, COLOR_BLUE, 3, CV_AA);

/////////////////////직진 코스 시 소실점 주행////////////////////////////////////////////////
   get_intersectpoint(p1, p2, Point(p3.x + 160, p3.y), Point(p4.x + 160, p4.y), &p5);

   float steer;
   float x_Difference = 160.0 - p5.x;

   if(x_Difference > 0.0){

     steer = 1520.0 + 3.2 * x_Difference;
   }
   else if(x_Difference < 0.0){
     steer = 1520.0 - 3.2 * x_Difference;
   }
   else{
     steer = 1520.0;
   }

   angle = steer;

   if(angle > 2000){
     angle = 2000;
   }
   else if(angle < 1000){
     angle = 1000;
   }
//////////////////////////////////////////////////////////////////////////////////////

  }

  // resize(resRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

  return angle;

}


}

////////////////////밑에 함수들은 exam_cv.cpp에서만 사용하는 함수들/////////////////////

void v_roi(Mat& img, Mat& img_ROI, const Point& p1, const Point& p2) {

	float slope = get_slope(p1, p2);
	float alphaY = 30.f / sqrt(slope*slope + 1);
	float alphaX = slope * alphaY;

	Point a(p1.x - alphaX, p1.y + alphaY);
	Point b(p1.x + alphaX, p1.y - alphaY);
	Point c(p2.x, p2.y);

	vector <Point> Left_Point;

	Left_Point.push_back(a);
	Left_Point.push_back(b);
	Left_Point.push_back(c);

	Mat roi(img.rows, img.cols, CV_8U, Scalar(0));

	fillConvexPoly(roi, Left_Point, Scalar(255));

	Mat filteredImg_Left;
	img.copyTo(filteredImg_Left, roi);

	img_ROI = filteredImg_Left.clone();

}

float get_slope(const Point& p1, const Point& p2) {

	float slope;

	if (p2.y - p1.y != 0.0) {
		slope = ((float)p2.y - (float)p1.y) / ((float)p2.x - (float)p1.x);
	}
	return slope;
}



bool get_intersectpoint(const Point& AP1, const Point& AP2,
	const Point& BP1, const Point& BP2, Point* IP)
{
	double t;
	double s;
	double under = (BP2.y - BP1.y)*(AP2.x - AP1.x) - (BP2.x - BP1.x)*(AP2.y - AP1.y);
	if (under == 0) return false;

	double _t = (BP2.x - BP1.x)*(AP1.y - BP1.y) - (BP2.y - BP1.y)*(AP1.x - BP1.x);
	double _s = (AP2.x - AP1.x)*(AP1.y - BP1.y) - (AP2.y - AP1.y)*(AP1.x - BP1.x);

	t = _t / under;
	s = _s / under;

	if (t<0.0 || t>1.0 || s<0.0 || s>1.0) return false;
	if (_t == 0 && _s == 0) return false;

	IP->x = AP1.x + t * (double)(AP2.x - AP1.x);
	IP->y = AP1.y + t * (double)(AP2.y - AP1.y);

	return true;
}

bool hough_left(Mat& img, Mat& srcRGB, Point* p1, Point* p2) {

	vector<Vec2f> linesL;
  vector<Vec2f> newLinesL;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 20;

  for (int i = 10; i > 0; i--){

    HoughLines(img, linesL, 1, CV_PI / 180, threshold);

    for(size_t i = 0; i < linesL.size(); i++){

      Vec2f temp;

      float rho = linesL[i][0];
      float theta = linesL[i][1];

      if(CV_PI / 18 >= theta || theta >= CV_PI / 18 * 8) continue;

      temp[0] = rho;
      temp[1] = theta;

      newLinesL.push_back(temp);

    }


    int clusterCount = 2;
  		Mat h_points = Mat(newLinesL.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (newLinesL.size() > 1) {
  			for (size_t i = 0; i < newLinesL.size(); i++) {
  				count++;
  				float rho = newLinesL[i][0];
  				float theta = newLinesL[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;


  		return false;
  	}
  	return true;
}

bool hough_right(Mat& img, Mat& srcRGB, Point* p1, Point* p2) {

	vector<Vec2f> linesR;
  vector<Vec2f> newLinesR;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 20;

  for (int i = 10; i > 0; i--){
    HoughLines(img, linesR, 1, CV_PI / 180, threshold);



    for(size_t i = 0; i < linesR.size(); i++){

      Vec2f temp;

      float rho = linesR[i][0];
      float theta = linesR[i][1];

      if(CV_PI / 18 * 10 >= theta || theta >= CV_PI / 18 * 17) continue;

      temp[0] = rho;
      temp[1] = theta;

      newLinesR.push_back(temp);

    }


    int clusterCount = 2;
  		Mat h_points = Mat(newLinesR.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (newLinesR.size() > 1) {
  			for (size_t i = 0; i < newLinesR.size(); i++) {
  				count++;
  				float rho = newLinesR[i][0];
  				float theta = newLinesR[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;

  		return false;
  	}
  	return true;
}

bool hough_curve(Mat& img, Mat& srcRGB, Point* p1, Point* p2) {

	vector<Vec2f> lines;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 20;

  for (int i = 10; i > 0; i--){
    HoughLines(img, lines, 1, CV_PI / 180, threshold);


    int clusterCount = 2;
  		Mat h_points = Mat(lines.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (lines.size() > 1) {
  			for (size_t i = 0; i < lines.size(); i++) {
  				count++;
  				float rho = lines[i][0];
  				float theta = lines[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;

  		return false;
  	}
  	return true;
}

int curve_detector(Mat& leftROI, Mat& rightROI){

  Mat originImg;
  Mat roiImg;
  Mat hsvImg1, binaryImg1;
  Mat cannyImg1;

  Point p1, p2;

  bool error = true;

  hconcat(leftROI, rightROI, originImg);

  roiImg = originImg(Rect(0, originImg.rows/2, originImg.cols, originImg.rows/2));

  cvtColor(roiImg, hsvImg1, CV_BGR2HSV);

  inRange(hsvImg1, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg1);

  Canny(binaryImg1, cannyImg1, 150, 250);


  error = hough_curve(cannyImg1, roiImg, &p1, &p2);

  float slope = get_slope(p1, p2);


  float x_L, x_R, y = 30;


  if(error){
    return 1520;
  }
  else if(slope < 0){ // right rotate

    float steer =  data_transform(slope, -1.0, -0.2, -500.0, 0.0);

    x_L = (y - p1.y + slope * p1.x) / slope;

    float dd = data_transform(x_L, -200.0, 220.0, 0 , 30.0);

    steer = 1500.0 + steer + dd;

    int angle = steer;

    if(slope < -1.0 || slope > -0.2) return 0;
    return angle;
  }
  else {

    x_R = (y - p1.y + slope * p1.x) / slope;

    float steer =  data_transform(slope, 0.2, 1.0, -500.0 , 0.0);


    float dd = data_transform(x_R, 120, 520, 0 , 30.0);

    steer = 1500.0 - steer - dd ;
    int angle = steer;

    if(slope < 0.2 || slope > 1.0) return 0;



    return angle;



  }





}

int rotary_curve_detector(Mat& leftROI, Mat& rightROI){

  Mat originImg;
  Mat grayImg;
  Mat roiImg;
  Mat hsvImg1, binaryImg1, binaryImg2;
  Mat cannyImg1;

  Point p1, p2;

  bool error = true;

  hconcat(leftROI, rightROI, originImg);

  // roiImg = originImg(Rect(0, originImg.rows/2, originImg.cols, originImg.rows/2));

  cvtColor(originImg, hsvImg1, CV_BGR2HSV);

  inRange(hsvImg1, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binaryImg1);
  inRange(roiImg, RGB_WHITE_LOWER, RGB_WHITE_UPPER, binaryImg2);

  binaryImg1 = binaryImg1 | binaryImg2;


  Canny(binaryImg1, cannyImg1, 150, 250);


  error = hough_curve(cannyImg1, roiImg, &p1, &p2);

  float slope = get_slope(p1, p2);


  float x_L, x_R, y = 30;


  if(error){
    return 1520;
  }
  else if(slope < 0){

    x_L = (y - p1.y + slope * p1.x) / slope;

    if(x_L > 20) return 1000;

    slope = slope * -1.0;


    float temp = (1.5 - slope) * 250.0;

    int angle = 1500 - temp;

    if(angle < 1000){
      angle = 1000;
    }

    return angle;
  }
  else {

    x_R = (y - p1.y + slope * p1.x) / slope;

    if(x_R < 300) return 2000;

    float temp = (1.5 - slope) * 250.0;

    int angle = 1500 + temp;

    if(angle > 2000){
      angle = 2000;
    }

    return angle;

  }

}

float data_transform(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
