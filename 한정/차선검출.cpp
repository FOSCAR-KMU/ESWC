#include <opencv2/opencv.hpp>
#include <iostream>
#include <utility>

#define PI 3.1415
#define HEIGHT 750
#define WIDTH 1334

#define CENTER 400

using namespace cv;
using namespace std;


//1이 위쪽
Point2f L[2];
Point2f R[2];

void TopView(Mat &src, Mat &dst){
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

    warpPerspective(src, dst, getPerspectiveTransform(src_vertices, dst_vertices), dst.size(), INTER_LINEAR, BORDER_CONSTANT);
}

int main(){
    Mat img2 = imread("차있는사진.JPG");
    resize(img2, img2, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_LINEAR);

    Mat img(HEIGHT, WIDTH, CV_8UC3);
    TopView(img2, img);
    imshow("Original",img);
    Mat gray;
    cvtColor(img,gray, COLOR_BGR2GRAY);

    Mat edges;
    Canny(gray, edges, 25, 50);
    imshow("Edges",edges);
    vector<cv::Vec2f> lines;
    HoughLines(edges, lines, 1, PI/180,170);
    
    // 선 그리기
    cv::Mat result(edges.rows, edges.cols, CV_8U, cv::Scalar(255));
    std::cout << "Lines detected: " << lines.size() << std::endl;

    // 선 벡터를 반복해 선 그리기
    std::vector<cv::Vec2f>::const_iterator it= lines.begin();
    while (it!=lines.end()) {
        float rho = (*it)[0];   // 첫 번째 요소는 rho 거리
        float theta = (*it)[1]; // 두 번째 요소는 델타 각도
        if (theta < PI/4. || theta > 3.*PI/4.) { // 수직 행
            cv::Point pt1(rho/cos(theta), 0); // 첫 행에서 해당 선의 교차점   
            cv::Point pt2((rho-result.rows*sin(theta))/cos(theta), result.rows);
            // 마지막 행에서 해당 선의 교차점
            cv::line(img, pt1, pt2, cv::Scalar(255), 1); // 하얀 선으로 그리기
        } else { // 수평 행
            cv::Point pt1(0,rho/sin(theta)); // 첫 번째 열에서 해당 선의 교차점  
            cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
            // 마지막 열에서 해당 선의 교차점
            cv::line(img, pt1, pt2, cv::Scalar(255), 1); // 하얀 선으로 그리기
        }
        std::cout << "line: (" << rho << "," << theta << ")\n"; 
        ++it;
    }

    imshow("Lines",img);
    
    int k = waitKey(0);
    if (k == 27) {       // wait for ESC key to exit
        destroyAllWindows();
    }

    return 0;
}