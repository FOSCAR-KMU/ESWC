#include <opencv2/opencv.hpp>
#include <iostream>
#include <utility>
#include <cmath>
//#include "car_lib.h"
#include <stdio.h>

#define IMGYE 100
#define IMGYE2 20
#define IMGYE3 1000

#define LEFT 1
#define RIGHT 2

#define PI 3.141592
using namespace std;
using namespace cv;

void get_center_point(const Mat& binaryImg, int & center_x, int & center_y);

//0 빨강 1 노랑
//{ x, y,cnt }
vector<Vec3f> v; 
bool check[2];
int main(){
    
    VideoCapture cap(1);
    // if(!cap.isOpened())
    //     return -1;

    // Vec3b vl1(0, 100, 50);
    // Vec3b vu1(100, 255, 200);

    // Vec3b vl2(165, 100, 50);
    // Vec3b vu2(179, 255, 200);
    int cnt = 0;
    const Vec3b HSV_RED_LOWER = Vec3b(0, 100, 100);
    const Vec3b HSV_RED_UPPER = Vec3b(10, 255, 255);

    const Vec3b HSV_RED_LOWER1 = Vec3b(160, 100, 100);
    const Vec3b HSV_RED_UPPER1 = Vec3b(179, 255, 255);

    const Vec3b HSV_YELLOW_LOWER = Vec3b(20, 40, 130);
    const Vec3b HSV_YELLOW_UPPER = Vec3b(50, 255, 255);

    const Vec3b HSV_GREEN_LOWER = Vec3b(60, 100, 50);
    const Vec3b HSV_GREEN_UPPER = Vec3b(110, 255, 255);
    
    check[0] = false;
    check[1] = false;
    
    v.push_back({0, 0, 0});
    v.push_back({0, 0, 0});

    Point2f diff;
    Point2f leftgreen;
    Point2f green;

    int steering = 0;
    
    for(;;){
        if(waitKey(30) >= 0) break;
        Mat frame, hsv, redBinaryImg, redBinaryImg1, yellowBinaryImg, greenBinaryImg, result, result1, result2;
        // cap >> frame;

        cap >> frame;// = imread("선택 영역_001.png");
        
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        inRange(hsv, HSV_RED_LOWER, HSV_RED_UPPER, redBinaryImg);
        inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1, redBinaryImg1);

        redBinaryImg = redBinaryImg | redBinaryImg1;

        inRange(hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, yellowBinaryImg);

        inRange(hsv, HSV_GREEN_LOWER, HSV_GREEN_UPPER, greenBinaryImg);
        imshow("frame",frame);
        //imshow("redBinaryImg", redBinaryImg);
        //imshow("yellowBinaryImg", yellowBinaryImg);
        //imshow("greenBinaryImg", greenBinaryImg);
        
        Point2i redCenter = {-1, -1};
        Point2i yellowCenter = {-1, -1};
        Point2i greenCenter = {-1, -1};
        
        get_center_point(redBinaryImg, redCenter.x,redCenter.y);
        get_center_point(yellowBinaryImg, yellowCenter.x, yellowCenter.y);
        get_center_point(greenBinaryImg, greenCenter.x, greenCenter.y);

        cout << redCenter.x << " " << redCenter.y << endl;
        cout << yellowCenter.x << " " << yellowCenter.y << endl;
        cout << greenCenter.x << " " << greenCenter.y << endl;

        if(check[0] && check[1]){
            cout << "초록불을 기다림" << '\n';
        }else if(check[0]){
            cout << "노란불을 기다림" << '\n';
        }else{
            cout << "빨강불을 기다림" << '\n';
        }
 
        if(check[0] && check[1]){
            diff = {v[1][0] - v[0][0], v[1][1] - v[0][1]};
            leftgreen = {v[1][0] + diff.x, v[1][1] + diff.y};
            green = {v[1][0] + 2 * diff.x, v[1][1] + 2 * diff.y};
        }
        if(!check[0]){
            if(redCenter.x == -1 || redCenter.y == -1)
                continue;
            cout << "red print dist : ";
            int distSquare = (v[0][0] - redCenter.x) * (v[0][0] - redCenter.x) +
            (v[0][1] - redCenter.y) * (v[0][1] - redCenter.y);
            cout << distSquare << '\n';
            if(distSquare < IMGYE){
                cnt++;
            }else{
                cnt = 0;
                v[0][0] = redCenter.x;
                v[0][1] = redCenter.y;
            }
            if(cnt > IMGYE2)
                check[0] = true;
        }else if(!check[1]){
            if(yellowCenter.x == -1 || yellowCenter.y == -1)
                continue;
            int dist = (v[0][0] - redCenter.x) * (v[0][0] - redCenter.x) +
            (v[0][1] - redCenter.y) * (v[0][1] - redCenter.y);
            int distSquare = (v[1][0] - yellowCenter.x) * (v[1][0] - yellowCenter.x) +
            (v[1][1] - yellowCenter.y) * (v[1][1] - yellowCenter.y);
            if(distSquare < IMGYE){
                cnt++;
            }else{
                cnt = 0;
                v[1][0] = yellowCenter.x;
                v[1][1] = yellowCenter.y;
            }
            if(cnt > IMGYE2)
                check[1] = true;
        }else{
            int differ = (leftgreen.x - greenCenter.x) * (leftgreen.x - greenCenter.x) + (leftgreen.y - greenCenter.y) * (leftgreen.y - greenCenter.y);
            cout << "differ : " << differ << ' ';
            cout << "leftgreen : " << leftgreen << '\n';
            
            if(differ < IMGYE3){
                steering = LEFT;//좌
                break;
            }
            
            differ = (green.x - greenCenter.x) * (green.x - greenCenter.x) + (green.y - greenCenter.y) * (green.y - greenCenter.y);
            cout << "differ : " << differ << ' ';
            cout << "green : " << differ << '\n';
            
            if(differ < IMGYE3){
                steering = RIGHT;
                break;
            }
        }
        
        //imshow("frame", frame);
        // imshow("binary", result);
    }
    
   /*
    for(;;){
        if(waitKey(30) >= 0) break;
        Mat yellowBinaryImg, hsv, frame;
        
        cap >> frame;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, yellowBinaryImg);
        steering = LEFT;
        if(steering == LEFT)
            yellowBinaryImg = yellowBinaryImg(Rect(yellowBinaryImg.cols / 2, 0, yellowBinaryImg.cols / 2, yellowBinaryImg.rows));
        else
            yellowBinaryImg = yellowBinaryImg(Rect(0, 0, yellowBinaryImg.cols / 2, yellowBinaryImg.rows));
        
        vector<cv::Vec2f> lines;
        HoughLines(yellowBinaryImg, lines, 1, PI/180, 130);
        if(lines.empty())
            continue;
        
        float theta = lines[0][1];
        float slope = -1 / tan(theta);
        cout << "slope : " << slope << '\n';
        imshow("pandan", yellowBinaryImg);
    }
    */
    return 0;
}

void get_center_point(const Mat& binaryImg, int & center_x, int & center_y){

    int cnt_x = 0;
    int cnt_y = 0;
    int cnt = 0;

    for(int i = 0; i < binaryImg.cols; i++){
        for(int j = 0; j < binaryImg.rows; j++){
            if (binaryImg.at<uchar>(j, i) == 255){
                cnt_x += i;
                cnt_y += j;
                cnt++;
            }
        }
    }

    if(cnt != 0 && cnt > IMGYE){
        center_x = cnt_x / cnt;
        center_y = cnt_y / cnt;
    }
 }
