
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

const Vec3b HSV_YELLOW_LOWER = Vec3b(20, 40, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(50, 255, 255);

const Vec3b HSV_RED_LOWER = Vec3b(0, 100, 100);
const Vec3b HSV_RED_UPPER = Vec3b(10, 255, 255);
const Vec3b HSV_RED_LOWER1 = Vec3b(160, 100, 100);
const Vec3b HSV_RED_UPPER1 = Vec3b(179, 255, 255);

const Vec3b HSV_GREEN_LOWER = Vec3b(60, 100, 50);
const Vec3b HSV_GREEN_UPPER = Vec3b(110, 255, 255);



float data_transform(float x, float in_min, float in_max, float out_min, float out_max);
void get_center_point(Mat& binaryImg, Point * p);


extern "C" {




int traffic_light(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh, int centerP[]){


  Mat srcRGB(ih, iw, CV_8UC3, srcBuf); //input
  Mat dstRGB(nh, nw, CV_8UC3, outBuf); //ouput
  Mat resRGB(ih, iw, CV_8UC3);         //result

  Mat roiImg, hsv, redBinaryImg, redBinaryImg1, yellowBinaryImg, greenBinaryImg;
  Mat result;

  cvtColor(srcRGB, hsv, COLOR_BGR2HSV);

  // inRange(hsv, G, HSV_YELLOW_UPPER, yellowBinaryImg);

  // inRange(hsv, HSV_RED_LOWER, HSV_RED_UPPER, redBinaryImg);
  // inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1, redBinaryImg1);
  //
  // redBinaryImg = redBinaryImg | redBinaryImg1;

  inRange(hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, yellowBinaryImg);


  cvtColor(yellowBinaryImg, resRGB, CV_GRAY2BGR);
  resize(resRGB, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);

  Point yellowCenterP(-1, -1);

  get_center_point(yellowBinaryImg, & yellowCenterP);


  centerP[0] = yellowCenterP.x;
  centerP[1] = yellowCenterP.y;

  if(yellowCenterP.x == -1 || yellowCenterP.y == -1) return 1520;


  float x_Difference = 160.0 - (float) yellowCenterP.x;
  float steer;
  int angle;

  if(x_Difference > 0.0){
    steer = 1520.0 + (3.4) * x_Difference;
  }
  else if(x_Difference < 0.0){
    steer = 1520.0 + 3.4 * x_Difference;
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

  return angle;



}

}


float data_transform(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void get_center_point(Mat& binaryImg, Point * p){

    int cnt_x = 0;
    int cnt_y = 0;
    int count = 0;

    for(int i = 0; i < binaryImg.cols; i++){
        for(int j = 0; j < binaryImg.rows; j++){
            if (binaryImg.at<uchar>(j, i) == 255){
                cnt_x += i;
                cnt_y += j;
                count++;
            }
        }
    }

    if(count > 1){
        p->x = cnt_x / count;
        p->y = cnt_y / count;
    }
 }
