#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const Vec3b HSV_YELLOW_LOWER = Vec3b(20, 40, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(50, 255, 255);


int main()
{
    VideoCapture capture(1);
    Mat frame;

    while(1)
    {
        capture >> frame;

        Mat hsvImg;
        
        cvtColor(frame, hsvImg, CV_BGR2HSV);

        Mat binImg;
        inRange(hsvImg, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, binImg);
        // threshold(grayImg, binImg, 150, 255, THRESH_BINARY);
        
        int blockSize = 2;
        int apertureSize = 3;
        double k = 0.06;
        float thresh = 250;

        Mat harrisImg, harrisImg_norm, harrisImg_norm_scaled;

        cornerHarris(binImg, harrisImg, blockSize, apertureSize, k, BORDER_DEFAULT);

        normalize(harrisImg, harrisImg_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );

        convertScaleAbs(harrisImg_norm, harrisImg_norm_scaled);

        for( int j = 0; j < harrisImg_norm.rows ; j++ ) {
            for( int i = 0; i < harrisImg_norm.cols; i++ ) {
                if( (int) harrisImg_norm.at<float>(j,i) > thresh ) {
                    circle( harrisImg_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
                }
            }
        }

        imshow("binary", binImg);
        imshow("harris_norm", harrisImg_norm);
        imshow("harris_norm_scaled", harrisImg_norm_scaled);
        if(waitKey(33) == 0);
    }
}
