#include <opencv2/opencv.hpp>
#include <iostream>
#include <utility>

#define HEIGHT 750
#define WIDTH 1334

#define CENTER 400

using namespace cv;
using namespace std;

//1이 위쪽
Point2f L[2];
Point2f R[2];

int main(){
    Mat src = imread("차있는사진.JPG");
    resize(src, src, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_LINEAR);

    cout << src.rows << ' ' << src.cols << endl;

    Point2f src_vertices[4];
    L[0] = Point(660, 430);
    L[1] = Point(0, 742);
    R[0] = Point(870, 444);
    R[1] = Point(1400, 740);

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

    imshow("src", src);
    imshow("dst", dst);
    waitKey();
}