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

int isTunnel(Mat origin)
{
  int x, y, cnt = 0;
  Mat tmpImage; // 640 * 480
  cvtColor(origin, tmpImage,CV_BGR2YUV);
  imshow("yuv", tmpImage);
  inRange(tmpImage, Scalar(10, 110, 120), Scalar(80, 130, 140), tmpImage);
  imshow("inrange", tmpImage);

  for(y = 480; y >= 400; --y)
    for(x = 450; x <= tmpImage.rows; ++x)
    {
      if(tmpImage.at<uchar>(y, x) == 255) //white (그림자 존재)
        cnt++;
      if(cnt > 1500)
        return cnt;
    }
  return cnt;
}
//
// bool tunnelEnd(Mat origin)
// {
//   int x, y, cnt = 0;
//   Mat tmpImage; // 640 * 480
//   cvtColor(origin, tmpImage,CV_BGR2YUV);
//   imshow("yuv", tmpImage);
//   inRange(tmpImage, Scalar(10, 110, 120), Scalar(70, 130, 140), tmpImage);
//   imshow("inrange", tmpImage);
//
//   for(y = 480; y >= 400; --y)
//     for(x = 450; x <= tmpImage.rows; ++x)
//     {
//       if(tmpImage.at<uchar>(y, x) == 0) // black (그림자 없음)
//         cnt++;
//       if(cnt > 1500)
//         return true;
//     }
//   return false;
// }

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
   //CarControlInit();
   for(;;)
   {
     Mat frame;
     cap.read(frame);
     if(isTunnel(frame) > 1500 && tunnel_flag == 0) // 터널 진입 전 -> 후
        tunnel_flag = 1;
     else if(tunnel_flag == 1)
     {
        //tunnelRun();
        printf("running tunnel\n");
        if(isTunnel(frame) < 100)
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
