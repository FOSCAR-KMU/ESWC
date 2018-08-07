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
using namespace cv;

/*******************************************************************************
 *  INCLUDE FILES
 *******************************************************************************
 */
#include <stdio.h>
#include "car_lib.h"

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */
#define LIGHT_BEEP       // to test light and beep
#define POSITION_CONTROL  // to test postion control
#define SPEED_CONTROL     // to test speed control
#define SERVO_CONTROL     // to test servo control(steering & camera position)
#define LINE_TRACE              // to test line trace sensor
#define DISTANCE_SENSOR     // to test distance sensor

/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
unsigned char status;
short speed;
unsigned char gain;
int position, posInit, posDes, posRead;
short angle;
int channel;
int data;
char sensor;
int i, j;
int tol;
char byte = 0x80;


int data_transform(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool tunnelStart(Mat origin)
{
  int x, y, cnt = 0;
  Mat tmpImage; // 640 * 480
  cvtColor(origin, tmpImage,CV_BGR2YUV);
  imshow("yuv", tmpImage);
  inRange(tmpImage, Scalar(10, 110, 120), Scalar(70, 130, 140), tmpImage);
  imshow("inrange", tmpImage);

  for(y = 100; y <= 240; ++y)
    for(x = 450; x <= tmpImage.rows; ++x)
    {
      if(tmpImage.at<uchar>(y, x) == 255) //white
        cnt++;
      if(cnt > 500)
        return true;
    }
  return false;
}

bool tunnelEnd(Mat origin)
{
  int x, y, cnt = 0;
  Mat tmpImage; // 640 * 480
  cvtColor(origin, tmpImage,CV_BGR2YUV);
  imshow("yuv", tmpImage);
  inRange(tmpImage, Scalar(10, 110, 120), Scalar(70, 130, 140), tmpImage);
  imshow("inrange", tmpImage);

  for(y = 100; y <= 240; ++y)
    for(x = 450; x <= tmpImage.rows; ++x)
    {
      if(tmpImage.at<uchar>(y, x) == 0) // black
        cnt++;
      if(cnt > 500)
        return true;
    }
  return false;
}

void tunnelRun()
{
  int channel_left = 6, channel_right = 2;
  int data_left, data_right;
  data_left = DistanceSensor(channel_left);
  data_right = DistanceSensor(channel_right);

  data_left = data_transform(data_left, 0, 4005, 0, 5000);
  data_right = data_transform(data_right, 0, 4095, 0, 5000);

  // printf("channel = %d, distance = 0x%04X(%d) \n", channel, data, data);
  data_left = (27.61 / (data_left - 0.1696))*1000;
  data_right = (27.61 / (data_right - 0.1696))*1000;

  printf("left = %d , right = %d", data_left, data_right);

  angle = SteeringServoControl_Read();
  printf("SteeringServoControl_Read() = %d\n", angle);    //default = 1500, 0x5dc

  if(data_left < data_right) // right steering
  {
    angle -= 50;
  }

  else if(data_left > data_right) // left steering
  {
    angle += 50;
  }

  SteeringServoControl_Write(angle);
}

int main()
{
   int tunnel_flag = 0;
   VideoCapture cap(1);
   if(!cap.isOpened())
   {
     printf("error");
     exit(-1);
   }
   namedWindow("Video", 1);
   CarControlInit();
   for(;;)
   {
     Mat frame;
     cap.read(frame);
     if(tunnelStart(frame) && tunnel_flag == 0) // 터널 진입 전 -> 후
        tunnel_flag = 1;
     else if(tunnel_flag == 1)
     {
        tunnelRun();
        printf("running tunnel\n");
        if(tunnelEnd(frame))
        {
          tunnel_flag = -1; // 터널 종료
          printf("Stop!!\n");
        }
     }

     if(waitKey(1)==27)
     break;
   }
  return 0;
}
