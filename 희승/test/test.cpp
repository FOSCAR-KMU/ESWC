
#include <iostream>
#include <fstream>
#include <cv.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

const CvScalar COLOR_BLUE = CvScalar(255, 0, 0);
const CvScalar COLOR_RED = CvScalar(0, 0, 255);
void Binarization(Mat& Img1, Mat& Img2);
void Region_Of_Interest(Mat& Img1, Mat& Img2);
bool GetIntersectPoint(const Point& AP1, const Point& AP2,
                       const Point& BP1, const Point& BP2, Point* IP);



int main(){

  VideoCapture capture1;

  capture1.open(1);

  Mat originImg1;
  Mat originImg2;

  Mat grayImg1;
  Mat grayImg2;

  Mat blured1;
  Mat blured2;

  Mat cannyImg1;
  Mat cannyImg2;

  Mat houghImg1;
  Mat houghImg2;

  Mat imageROI1;
  Mat imageROI2;

  while(1) {

    capture1 >> originImg1; // left_camera


    // 흑백화 하기
    cvtColor(originImg1, grayImg1, COLOR_BGR2GRAY);

    // ROI 설정


    imshow("gray", grayImg1);


    if(waitKey(10) == 0){
      break;
    }

  }




}


void Binarization(Mat& Img1, Mat& Img2){

  for(int i = 0; i < Img1.rows; i++){
    for(int j = 0; j < Img1.cols; j++){
      if(Img1.at<uchar>(i, j) < 130){
        Img1.at<uchar>(i, j) = 0;
      }
      else{
        Img1.at<uchar>(i, j) = 255;
      }

      if(Img2.at<uchar>(i, j) < 90){
        Img2.at<uchar>(i, j) = 0;
      }
      else{
        Img2.at<uchar>(i, j) = 255;
      }

    }
  }

}

void Region_Of_Interest(Mat& Img1, Mat& Img2){

  int right = 200;
  int left = Img1.cols - 320;

  for(int i = 0; i < Img1.cols; i++){
    for(int j = 0; j < Img1.rows/2; j++){
      Img1.at<uchar>(j, i) = 0;
      Img2.at<uchar>(j, i) = 0;

    }
  }

  for(int i = Img1.rows/2; i < Img1.rows; i++){
    for(int j = 0; j < 200; j++){
      Img2.at<uchar>(i, j) = 0;

    }
    for(int j = right; j < Img1.cols; j++){
      Img2.at<uchar>(i, j) = 0;

    }
    right += 2 ;
  }
  /////////////////////////////////

  for(int i = Img1.rows/2; i < Img1.rows; i++){
    for(int j = Img1.cols; j >= Img1.cols-320; j--){
      Img1.at<uchar>(i, j) = 0;

    }
    for(int j = left; j >=0; j--){
      Img1.at<uchar>(i, j) = 0;

    }
    left -= 1 ;
  }



}

bool GetIntersectPoint(const Point& AP1, const Point& AP2,
                       const Point& BP1, const Point& BP2, Point* IP)
{
    double t;
    double s;
    double under = (BP2.y-BP1.y)*(AP2.x-AP1.x)-(BP2.x-BP1.x)*(AP2.y-AP1.y);
    if(under==0) return false;

    double _t = (BP2.x-BP1.x)*(AP1.y-BP1.y) - (BP2.y-BP1.y)*(AP1.x-BP1.x);
    double _s = (AP2.x-AP1.x)*(AP1.y-BP1.y) - (AP2.y-AP1.y)*(AP1.x-BP1.x);

    t = _t/under;
    s = _s/under;

    if(t<0.0 || t>1.0 || s<0.0 || s>1.0) return false;
    if(_t==0 && _s==0) return false;

    IP->x = AP1.x + t * (double)(AP2.x-AP1.x);
    IP->y = AP1.y + t * (double)(AP2.y-AP1.y);

    return true;
}
