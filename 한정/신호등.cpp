#include <opencv2/opencv.hpp>
#include <iostream>
#include <utility>

#define IMGYE 30
#define IMGYE2 5
using namespace std;
using namespace cv;

//0 빨강 1 노랑
//{ x, y,cnt }
vector<Vec3f> v; 
bool check[2];
int main(){
    
    VideoCapture cap(1);
    if(!cap.isOpened())
        return -1;

    Vec3b vl1(0, 100, 50);
    Vec3b vu1(100, 255, 200);

    Vec3b vl2(165, 100, 50);
    Vec3b vu2(179, 255, 200);
    
    check[0] = false;
    check[1] = false;
    
    v.push_back({0, 0, 0});
    v.push_back({0, 0, 0});
    int cnt = 0;
    for(;;){
        Mat frame, hsv, result1, result2, result;
        cap >> frame;
        
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, vl1, vu1, result1);
        inRange(hsv, vl2 ,vu2, result2);
        addWeighted(result1, 1.0, result2, 1.0, 0.0, result);

        Mat edge;
        Canny(result, edge, 230, 280);
        //GaussianBlur(result, result, Size(3, 3), 2, 2);
        
        vector<Vec3f> circles;
        HoughCircles(edge, circles, cv::HOUGH_GRADIENT, 1, result.rows/8, 120, 35, 0, 0);
        if(check[0] && check[1]){
            cout << "초록불을 기다림" << '\n';
        }else if(check[0]){
            cout << "노란불을 기다림" << '\n';
        }else{
            cout << "빨강불을 기다림" << '\n';
        }
        if(check[0] && check[1]){
            Point2f diff = {v[1][0] - v[0][0], v[1][1] - v[0][1]};
            Point2f leftgreen = {v[1][0] + diff.x, v[1][1] + diff.y};
            Point2f green = {v[1][0] + 2 * diff.x, v[1][1] + 2 * diff.y};
        }
 	    for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
 	    	cv::Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
 	    	int radius = round(circles[current_circle][2]);
            if(!check[0]){
                int distSquare = (v[0][0] - circles[current_circle][0]) * (v[0][0] - circles[current_circle][0]) +
                (v[0][1] - circles[current_circle][1]) * (v[0][1] - circles[current_circle][1]);
                if(distSquare < IMGYE){
                    cnt++;
                }else{
                    cnt = 0;
                    v[0][0] = circles[current_circle][0];
                    v[0][1] = circles[current_circle][1];
                }
                if(cnt > IMGYE2)
                    check[0] = true;
            }else if(!check[1]){
                int dist = (v[0][0] - circles[current_circle][0]) * (v[0][0] - circles[current_circle][0]) +
                (v[0][1] - circles[current_circle][1]) * (v[0][1] - circles[current_circle][1]);
                int distSquare = (v[1][0] - circles[current_circle][0]) * (v[1][0] - circles[current_circle][0]) +
                (v[1][1] - circles[current_circle][1]) * (v[1][1] - circles[current_circle][1]);
                if(dist > IMGYE && distSquare < IMGYE){
                    cnt++;
                }else{
                    cnt = 0;
                    v[1][0] = circles[current_circle][0];
                    v[1][1] = circles[current_circle][1];
                }
                if(cnt > IMGYE2)
                    check[1] = true;
            }else{
                int distSquare1 = (v[0][0] - circles[current_circle][0]) * (v[0][0] - circles[current_circle][0]) +
                (v[0][1] - circles[current_circle][1]) * (v[0][1] - circles[current_circle][1]);
                if(distSquare1 < IMGYE){
                    cout << "left 시발" <<'\n';
                }
                int distSquare2 = (v[1][0] - circles[current_circle][0]) * (v[1][0] - circles[current_circle][0]) +
                (v[1][1] - circles[current_circle][1]) * (v[1][1] - circles[current_circle][1]);
                if(distSquare2 < IMGYE){
                    cout << "right 시발" <<'\n';
                }
            }
 	    	cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 5);
 	    }
        imshow("frame", frame);
        imshow("result", edge);

        if(waitKey(30) >= 0) break;
    }
    
    return 0;
}