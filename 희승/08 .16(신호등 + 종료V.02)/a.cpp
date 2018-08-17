#include <opencv2/opencv.hpp>
#include <iostream>
#include <utility>

#define IMGYE 100
#define IMGYE2 20
#define IMGYE3 500
using namespace std;
using namespace cv;

void get_center_point(const Mat& binaryImg, int & center_x, int & center_y);
int traffic(const Mat& oriImg)

const Vec3b HSV_RED_LOWER = Vec3b(0, 100, 100);
const Vec3b HSV_RED_UPPER = Vec3b(10, 255, 255);

const Vec3b HSV_RED_LOWER1 = Vec3b(160, 100, 100);
const Vec3b HSV_RED_UPPER1 = Vec3b(179, 255, 255);

const Vec3b HSV_YELLOW_LOWER = Vec3b(20, 40, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(50, 255, 255);

const Vec3b HSV_GREEN_LOWER = Vec3b(60, 100, 50);
const Vec3b HSV_GREEN_UPPER = Vec3b(110, 255, 255);

//0 빨강 1 노랑
//{ x, y,cnt }


vector<Vec3f> v;
bool check[2];
int main(){

    int cnt = 0;


    check[0] = false;
    check[1] = false;

    v.push_back({0, 0, 0});
    v.push_back({0, 0, 0});

    Point2f diff;
    Point2f leftgreen;
    Point2f green;


    return 0;
}

int traffic(const Mat& oriImg){


  Mat roiImg, hsv, redBinaryImg, redBinaryImg1, yellowBinaryImg, greenBinaryImg;
  Mat result;

  roiImg = srcRGB(Rect(0, 0, srcRGB.cols, srcRGB.rows/3));

  cvtColor(roiImg, hsv, COLOR_BGR2HSV);

  inRange(hsv, HSV_RED_LOWER, HSV_RED_UPPER, redBinaryImg);
  inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1, redBinaryImg1);

  redBinaryImg = redBinaryImg | redBinaryImg1;

  inRange(hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, yellowBinaryImg);

  inRange(hsv, HSV_GREEN_LOWER, HSV_GREEN_UPPER, greenBinaryImg);

  result = redBinaryImg | yellowBinaryImg | greenBinaryImg;

  cvtColor(result, result, CV_GRAY2BGR);
  resize(result, dstRGB, Size(nw, nh), 0, 0, CV_INTER_LINEAR);
  //
  Point2f rydiff;
  Point2f rgdiff;
  Point2f leftgreen;
  Point2f green;

  Point redCenter, yellowCenter, greenCenter;

  redCenter.x = 0, redCenter.y = 0;
  yellowCenter.x = 0, yellowCenter.y = 0;
  greenCenter.x = 0, greenCenter.y = 0;


  get_center_point(redBinaryImg, & redCenter);
  get_center_point(yellowBinaryImg, & yellowCenter);
  get_center_point(greenBinaryImg, & greenCenter);

  centerP[0] = redCenter.x;
  centerP[1] = redCenter.y;
  centerP[2] = yellowCenter.x;
  centerP[3] = yellowCenter.y;
  centerP[4] = greenCenter.x;
  centerP[5] = greenCenter.y;


    if(check[0] && check[1]){
        if(greenCenter.x == 0 || greenCenter.y == 0)
            return -5;
        rydiff = {v[1].x - v[0].x, v[1].y - v[0].y};
    }
    if(!check[0]){
        if(redCenter.x == 0 || redCenter.y == 0)
            return -3;

        v[0].x = redCenter.x;
        v[0].y = redCenter.y;
        check[0] = true;
    }
    else if(!check[1]){
        if(yellowCenter.x == 0 || yellowCenter.y == 0)
            return -4;

        v[1].x = yellowCenter.x;
        v[1].y = yellowCenter.y;

        check[1] = true;
    }
    else{
        rgdiff = {greenCenter.x - v[0].x, greenCenter.y - v[0].y};
        float ratio = rgdiff.x / rydiff.x;
        if(ratio > 2.5)
            return 2;
        else if(ratio > 1.5)
            return 1;
        else
            return 7;
    }
    return -6;
  }

}

void get_center_point(const Mat& binaryImg, int & center_x, int & center_y){

    int cnt_x = 0;
    int cnt_y = 0;
    int cnt = 0;

    for(int i = 0; i < binaryImg.cols; i++){
        for(int j = 0; j < binaryImg.rows; j++){
            if (binaryImg.at<uchar>(j, i) == 255){
                cnt_x += i;
                cnt_y += j;
                cnt++;
            }
        }
    }

    if(cnt != 0 && cnt > IMGYE){
        center_x = cnt_x / cnt;
        center_y = cnt_y / cnt;
    }
 }
