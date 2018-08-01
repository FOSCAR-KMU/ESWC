#include <opencv2/opencv.hpp>
#include <iostream>
#include <utility>

#define PI 3.1415
#define HEIGHT 750
#define WIDTH 1334

#define CENTER 400

using namespace cv;
using namespace std;

int main(){
    Mat img = imread("장애물유.JPG");
    resize(img, img, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_LINEAR);

    imshow("Original",img);
    Mat gray;
    cvtColor(img,gray, COLOR_BGR2GRAY);

    Mat edges;
    Canny(gray, edges, 250, 500);
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