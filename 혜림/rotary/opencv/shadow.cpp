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

int isShadow(Mat origin)
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
      if(cnt > 300)
        return cnt;
    }
  return cnt;
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
     if(isShadow(frame) > 300 && rotary_flag == 0) // 차가 지나가는 도중
        rotary_flag = 1;
     else if(isShadow(frame) < 40 && rotary_flag == 1) // 차가 완전히 지나갔을 경우
     {
        std::cout << "YES" << std::endl;
        //flag = -1;
     }
     else if(rotary_flag == 0) // 차가 지나가기 전
        std::cout << "NO" << std::endl;
     // imshow("Video",frame);
     if(waitKey(1)==27)
     break;
   }
  return 0;
}
