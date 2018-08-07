
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

extern "C" {

/**
  * @brief  To load image file to the buffer.
  * @param  file: pointer for load image file in local path
             outBuf: destination buffer pointer to load
             nw : width value of the destination buffer
             nh : height value of the destination buffer
  * @retval none
  */
void OpenCV_load_file(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB;
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    srcRGB = imread(file, CV_LOAD_IMAGE_COLOR); // rgb
    //cvtColor(srcRGB, srcRGB, CV_RGB2BGR);

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  To convert format from BGR to RGB.
  * @param  inBuf: buffer pointer of BGR image
             w: width value of the buffers
             h : height value of the buffers
             outBuf : buffer pointer of RGB image
  * @retval none
  */
void OpenCV_Bgr2RgbConvert(unsigned char* inBuf, int w, int h, unsigned char* outBuf)
{
    Mat srcRGB(h, w, CV_8UC3, inBuf);
    Mat dstRGB(h, w, CV_8UC3, outBuf);

    cvtColor(srcRGB, dstRGB, CV_BGR2RGB);
}

/**
  * @brief  Detect faces on loaded image and draw circles on the faces of the loaded image.
  * @param  file: pointer for load image file in local path
             outBuf: buffer pointer to draw circles on the detected faces
             nw : width value of the destination buffer
             nh : height value of the destination buffer
  * @retval none
  */
void OpenCV_face_detection(char* file, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    // Load Face cascade (.xml file)
    CascadeClassifier face_cascade;
    face_cascade.load( "haarcascade_frontalface_alt.xml" );

    // Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale( srcRGB, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

    // Draw circles on the detected faces
    for( int i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        ellipse( srcRGB, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

/**
  * @brief  To bind two images on destination buffer.
  * @param  file1: file path of first image to bind
             file2: file path of second image to bind
             outBuf : destination buffer pointer to bind
             nw : width value of the destination buffer
             nh : height value of the destination buffer
  * @retval none
  */
void OpenCV_binding_image(char* file1, char* file2, unsigned char* outBuf, int nw, int nh)
{
    Mat srcRGB = imread(file1, CV_LOAD_IMAGE_COLOR);
    Mat srcRGB2 = imread(file2, CV_LOAD_IMAGE_COLOR);
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    cv::resize(srcRGB2, srcRGB2, cv::Size(srcRGB2.cols/1.5, srcRGB2.rows/1.5));
    cv::Point location = cv::Point(280, 220);
    for (int y = std::max(location.y, 0); y < srcRGB.rows; ++y)
    {
        int fY = y - location.y;
        if (fY >= srcRGB2.rows)
            break;

        for (int x = std::max(location.x, 0); x < srcRGB.cols; ++x)
        {
            int fX = x - location.x;
            if (fX >= srcRGB2.cols)
            break;

            double opacity = ((double)srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + 3]) / 255.;
            for (int c = 0; opacity > 0 && c < srcRGB.channels(); ++c)
            {
                unsigned char overlayPx = srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + c];
                unsigned char srcPx = srcRGB.data[y * srcRGB.step + x * srcRGB.channels() + c];
                srcRGB.data[y * srcRGB.step + srcRGB.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

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
    cv::Mat contours;
    cv::Canny(srcGRAY, // �׷��̷��� ����
        contours, // ���� �ܰ���
        125,  // ���� ���谪
        350);  // ���� ���谪

    // ������ ȭ�ҷ� �ܰ����� ǥ���ϹǷ� ���� ���� ����
    //cv::Mat contoursInv; // ���� ����
    //cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
    // ���� ���� 128���� ������ 255�� �ǵ��� ����

    cvtColor(contours, contours, CV_GRAY2BGR);

    cv::resize(contours, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
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
    Scalar lineColor = cv::Scalar(255,255,255);

    Mat dstRGB(nh, nw, CV_8UC3, outBuf);

    Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
    Mat resRGB(ih, iw, CV_8UC3);
    //cvtColor(srcRGB, srcRGB, CV_BGR2BGRA);

    // ĳ�� �˰����� ����
    cv::Mat contours;
    cv::Canny(srcRGB, contours, 125, 350);

    // �� ���� ���� ���� ��ȯ
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(contours, lines, 1, PI/180, // �ܰ躰 ũ�� (1�� ��/180���� �ܰ躰�� ������ ���� ������ �������� ���� ã��)
        80);  // ��ǥ(vote) �ִ� ����

    // �� �׸���
    cv::Mat result(contours.rows, contours.cols, CV_8UC3, lineColor);
    //printf("Lines detected: %d\n", lines.size());

    // �� ���͸� �ݺ��� �� �׸���
    std::vector<cv::Vec2f>::const_iterator it= lines.begin();
    while (it!=lines.end())
    {
        float rho = (*it)[0];   // ù ��° ���Ҵ� rho �Ÿ�
        float theta = (*it)[1]; // �� ��° ���Ҵ� ��Ÿ ����

        if (theta < PI/4. || theta > 3.*PI/4.) // ���� ��
        {
            cv::Point pt1(rho/cos(theta), 0); // ù �࿡�� �ش� ���� ������
            cv::Point pt2((rho-result.rows*sin(theta))/cos(theta), result.rows);
            // ������ �࿡�� �ش� ���� ������
            cv::line(srcRGB, pt1, pt2, lineColor, 1); // �Ͼ� ������ �׸���

        }
        else // ���� ��
        {
            cv::Point pt1(0,rho/sin(theta)); // ù ��° ������ �ش� ���� ������
            cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
            // ������ ������ �ش� ���� ������
            cv::line(srcRGB, pt1, pt2, lineColor, 1); // �Ͼ� ������ �׸���
        }
        //printf("line: rho=%f, theta=%f\n", rho, theta);
        ++it;
    }

    cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
}

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

void OpenCV_shadow()
{
  int checkX = -1, checkY = -1; // 그림자 시작좌표
  IplImage* tmpImage; // x : 320, y : 180
  char filename[30];
  tmpImage = cvCreateImage(cvGetSize(origin),IPL_DEPTH_8U,1);

  cvCvtColor(origin, tmpImage, CV_RGB2GRAY);
  cvNot(tmpImage, tmpImage);
  cvThreshold(tmpImage, tmpImage, 220, 255, CV_THRESH_BINARY); // 이진화

  for(int y = 180; y >= 0; y--)
  {
    for(int x = 0; x <  tmpImage->widthStep; x++)
      if(tmpImage->imageData[x + y*tmpImage->widthStep] == 255)
      {
        checkX = x;
        checkY = y;
        break;
      }
  }

  //
  sprintf(filename, "image5/testb%d.png",num);
  cvSaveImage(filename, tmpImage, 0);
  sprintf(filename, "image5/testo%d.png",num);
  cvSaveImage(filename, origin, 0);
  // //ll++;


}

}
