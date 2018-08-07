
/*
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

bool isFinish(Mat origin)
{
  int x, y, cnt = 0;
  Mat tmpImage; // 640 * 480
  cvtColor(origin, tmpImage, CV_BGR2GRAY);
  imshow("gray", tmpImage);
  Mat cvCorner = origin.clone();
  std::vector<Point2f> corners;

  goodFeaturesToTrack(tmpImage, corners, 10, 0.5, 10);

  for(int i=0; i<corners.size() ; i++)
    circle(cvCorner, corners[i], 5, Scalar(0, 0, 0));

  imshow("corner", cvCorner);
}

int main()
{
   int rotary_flag = 0;
   VideoCapture cap(1);
   if(!cap.isOpened())
   {
     printf("error");
     exit(-1);
   }
   namedWindow("Video", 1);
   for(;;)
   {
     Mat frame;
     cap.read(frame);
     if(isFinish(frame))
      printf("isFinish!!! \n");
     else
      printf("No \n");
    // imshow("Video",frame);
     if(waitKey(1)==27)
     break;
   }
  return 0;
}

*/
