#include <opencv2/opencv.hpp>
#include <iostream>
// #include <utility>

#define HEIGHT 750
#define WIDTH 1334
//#define HEIGHT 480
//#define WIDTH 640

#define CENTER 400

using namespace cv;
using namespace std;

const Vec3b YUV_BLACK_LOWER = Vec3b(10, 110, 120);
const Vec3b YUV_BLACK_UPPER = Vec3b(70, 130, 140);

//1이 위쪽
Point2f L[2];
Point2f R[2];

int main(){
    VideoCapture capture(0);
    Mat src;
    while(1)
    {
        capture >> src;
        resize(src, src, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_LINEAR);
        
        cout << src.rows << ' ' << src.cols << endl;
    
        Point2f src_vertices[4];
        L[0] = Point(525, 316);
        L[1] = Point(80, 540);
        R[0] = Point(780, 305);
        R[1] = Point(1330, 540);
    
        float diff0 = R[0].x - L[0].x;
        float diff1 = R[1].x - L[1].x;
    
        src_vertices[0] = Point(L[0].x - diff0, R[0].y);
        src_vertices[1] = Point(R[0].x + diff0, R[0].y);
        src_vertices[2] = Point(R[1].x + diff1, R[1].y);
        src_vertices[3] = Point(L[1].x - diff1, R[1].y);
    
        Point2f dst_vertices[4];
        dst_vertices[0] = Point(0, 0);
        dst_vertices[1] = Point(WIDTH, 0);
        dst_vertices[2] = Point(WIDTH, HEIGHT);
        dst_vertices[3] = Point(0, HEIGHT);
    
        Mat dst(HEIGHT, WIDTH, CV_8UC3);
    
        warpPerspective(src, dst, getPerspectiveTransform(src_vertices, dst_vertices), dst.size(), INTER_LINEAR, BORDER_CONSTANT);
    
        Mat yuv_img, bin_img;
        cvtColor(dst, yuv_img, CV_BGR2YUV);

        inRange(yuv_img, YUV_BLACK_LOWER, YUV_BLACK_UPPER, bin_img);

        int count[3] = {0, };

        for(int i = 0 ; i < bin_img.rows ; i++)
        {
            int j;
            for(j = 0 ; j < bin_img.cols / 3 ; j++)
                if(bin_img.at<uchar>(i, j) == 255) count[0]++;
            for(; j < bin_img.cols * 2 / 3 ; j++)
                if(bin_img.at<uchar>(i, j) == 255) count[1]++;
            for(; j < bin_img.cols ; j++)
                if(bin_img.at<uchar>(i, j) == 255) count[2]++;
        }    
    
        for(int i = 0 ; i < 3 ; i++)
        {
            cout << i << ' ';
            if(count[i] >= 3000) cout << "NONO" << endl;
            else cout << "YEAH" << endl;
        }
    
        //imshow("src", src);
        imshow("dst", dst);
        imshow("asdf", bin_img);
        if(cvWaitKey(33) == 27) break;

    }
    
}
