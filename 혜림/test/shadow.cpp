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

bool shadow(Mat origin)
{
  int checkX = -1,checkY = -1;
  int x, y;
  char filename[30];
  Mat tmpImage;
  cvtColor(origin, tmpImage,CV_BGR2YUV);
  imshow("yuv", tmpImage);
  inRange(tmpImage, Scalar(10, 110, 120), Scalar(70, 130, 140), tmpImage);
  imshow("inrange", tmpImage);
}

int main()
{
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
     if(shadow(frame))
        std::cout << "YES" << std::endl;
     //  else
     //    std::cout << "NO" << std::endl;
     // imshow("Video",frame);
     if(waitKey(1)==27)
     break;
   }
  return 0;
}
