#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <sys/wait.h>
#include <vector>
#include <utility>

#define HEIGHT 750
#define WIDTH 1334

using namespace cv;
using namespace std;

//1이 위쪽
Point2f L[2];
Point2f R[2];

//void 차선검출(){
//    img = cv2.imread('road.jpg',1)
//    cv2.imshow('Original',img)
//    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
//    cv2.imshow('Gray',gray)
//    edges = cv2.Canny(gray,250,500,apertureSize = 3)
//    cv2.imshow('Edges',edges)
//    lines = cv2.HoughLines(edges,1,np.pi/180,170)
//
//    for rho,theta in lines[0]:
//        a = np.cos(theta)
//        b = np.sin(theta)
//        x0 = a*rho
//        y0 = b*rho
//        x1 = int(x0 + 1000*(-b))
//        y1 = int(y0 + 1000*(a))
//        x2 = int(x0 - 1000*(-b))
//        y2 = int(y0 - 1000*(a))
//
//        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
//
//    cv2.imshow('Lines',img)
//
//    k = cv2.waitKey(0)
//    if k == 27:         // wait for ESC key to exit
//        cv2.destroyAllWindows()
//    }
//
//}

//void 흰선으로중앙맞추기(){
//    점1,2 = 차선검출();
    //중앙맞추자.
//}

const Vec3b YUV_LOWER = Vec3b(10, 110, 120);
const Vec3b YUV_UPPER = Vec3b(70, 130, 140);


bool hough_left(Mat& img, Mat& srcRGB, Point* p1, Point* p2) {

	vector<Vec2f> linesL;
  vector<Vec2f> newLinesL;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 40;

  for (int i = 10; i > 0; i--){

    HoughLines(img, linesL, 1, CV_PI / 180, threshold);

    for(size_t i = 0; i < linesL.size(); i++){

      Vec2f temp;

      float rho = linesL[i][0];
      float theta = linesL[i][1];

      if(CV_PI / 18 >= theta || theta >= CV_PI / 18 * 8) continue;

      temp[0] = rho;
      temp[1] = theta;

      newLinesL.push_back(temp);

    }


    int clusterCount = 2;
  		Mat h_points = Mat(newLinesL.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (newLinesL.size() > 1) {
  			for (size_t i = 0; i < newLinesL.size(); i++) {
  				count++;
  				float rho = newLinesL[i][0];
  				float theta = newLinesL[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			// cout << "pt : " << mypt1.x << ' ' << mypt1.y << endl;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			// cout << "pt : " << mypt2.x << ' ' << mypt2.y << endl;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;


  		return false;
  	}
  	return true;
}

bool hough_right(Mat& img, Mat& srcRGB, Point* p1, Point* p2) {

	vector<Vec2f> linesR;
  vector<Vec2f> newLinesR;

  Point point1;
  Point point2;

  int count = 0, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  int threshold = 40;

  for (int i = 10; i > 0; i--){
    HoughLines(img, linesR, 1, CV_PI / 180, threshold);



    for(size_t i = 0; i < linesR.size(); i++){

      Vec2f temp;

      float rho = linesR[i][0];
      float theta = linesR[i][1];

      if(CV_PI / 18 * 10 >= theta || theta >= CV_PI / 18 * 17) continue;

      temp[0] = rho;
      temp[1] = theta;

      newLinesR.push_back(temp);

    }


    int clusterCount = 2;
  		Mat h_points = Mat(newLinesR.size(), 1, CV_32FC2);
  		Mat labels, centers;
  		if (newLinesR.size() > 1) {
  			for (size_t i = 0; i < newLinesR.size(); i++) {
  				count++;
  				float rho = newLinesR[i][0];
  				float theta = newLinesR[i][1];


  				double a = cos(theta), b = sin(theta);
  				double x0 = a * rho, y0 = b * rho;
  				// cout << "x0, y0 : " << rho << ' ' << theta << endl;
  				h_points.at<Point2f>(i, 0) = Point2f(rho, (float)(theta * 100));
  			}
  			kmeans(h_points, clusterCount, labels,
  				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  				3, KMEANS_RANDOM_CENTERS, centers);

  			Point mypt1 = centers.at<Point2f>(0, 0);

  			float rho = mypt1.x;
  			float theta = (float)mypt1.y / 100;
  			double a = cos(theta), b = sin(theta);
  			double x0 = a * rho, y0 = b * rho;

  			int _x1 = int(x0 + 1000 * (-b));
  			int _y1 = int(y0 + 1000 * (a));
  			int _x2 = int(x0 - 1000 * (-b));
  			int _y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			Point mypt2 = centers.at<Point2f>(1, 0);

  			rho = mypt2.x;
  			theta = (float)mypt2.y / 100;
  			a = cos(theta), b = sin(theta);
  			x0 = a * rho, y0 = b * rho;

  			// cout << "pt : " << mypt2.x << ' ' << mypt2.y << endl;

  			_x1 = int(x0 + 1000 * (-b));
  			_y1 = int(y0 + 1000 * (a));
  			_x2 = int(x0 - 1000 * (-b));
  			_y2 = int(y0 - 1000 * (a));

  			x1 += _x1;
  			y1 += _y1;

  			x2 += _x2;
  			y2 += _y2;

  			break;
  		};
  	}
  	if (count != 0) {
  		p1->x = x1 / 2; p1->y = y1 / 2;
  		p2->x = x2 / 2; p2->y = y2 / 2;

  		return false;
  	}
  	return true;
}


bool get_intersectpoint(const Point& AP1, const Point& AP2,
	const Point& BP1, const Point& BP2, Point* IP)
{
	double t;
	double s;
	double under = (BP2.y - BP1.y)*(AP2.x - AP1.x) - (BP2.x - BP1.x)*(AP2.y - AP1.y);
	if (under == 0) return false;

	double _t = (BP2.x - BP1.x)*(AP1.y - BP1.y) - (BP2.y - BP1.y)*(AP1.x - BP1.x);
	double _s = (AP2.x - AP1.x)*(AP1.y - BP1.y) - (AP2.y - AP1.y)*(AP1.x - BP1.x);

	t = _t / under;
	s = _s / under;

	if (t<0.0 || t>1.0 || s<0.0 || s>1.0) return false;
	if (_t == 0 && _s == 0) return false;

	IP->x = AP1.x + t * (double)(AP2.x - AP1.x);
	IP->y = AP1.y + t * (double)(AP2.y - AP1.y);

	return true;
}


pair<bool, vector<Point> > find_point_4_top_view(Mat srcImg){
    vector<Point> result;
  
    Point p1, p2, p3, p4;
    Point i_p1, i_p2, i_p3, i_p4;

    bool left_error = true;
    bool right_error = true;

  // ĳ�� �˰����� ����
    Mat leftROI, rightROI;
    Mat yuvImg1, yuvImg2;
    Mat binaryImg1, binaryImg2;
    Mat cannyImg1, cannyImg2;

    leftROI  = srcImg(Rect(0, srcImg.rows/2, srcImg.cols/2, srcImg.rows/2));
    rightROI = srcImg(Rect(srcImg.cols/2, srcImg.rows/2, srcImg.cols/2, srcImg.rows/2));

    cvtColor(leftROI, yuvImg1, CV_BGR2YUV);
    cvtColor(rightROI, yuvImg2, CV_BGR2YUV);

    inRange(yuvImg1, YUV_LOWER, YUV_UPPER, binaryImg1);
    inRange(yuvImg2, YUV_LOWER, YUV_UPPER, binaryImg2);


    Canny(binaryImg1, cannyImg1, 150, 250);
    Canny(binaryImg2, cannyImg2, 150, 250);

    left_error = hough_left(cannyImg1, leftROI, &p1, &p2);
    right_error = hough_right(cannyImg2, rightROI, &p3, &p4);

    if(!left_error && !right_error)
    {
        get_intersectpoint(p1, p2, Point(0, 0), Point(srcImg.cols, 0), &i_p1);
        get_intersectpoint(p1, p2, Point(0, srcImg.rows), Point(srcImg.cols, srcImg.rows), &i_p2);
        get_intersectpoint(Point(p3.x + 160, p3.y), Point(p4.x + 160, p4.y), Point(0, 0), Point(srcImg.cols, 0), &i_p3);
        get_intersectpoint(Point(p3.x + 160, p3.y), Point(p4.x + 160, p4.y), Point(0, srcImg.rows), Point(srcImg.cols, srcImg.rows), &i_p4);

        result.push_back(i_p1);
        result.push_back(i_p2);
        result.push_back(i_p3);
        result.push_back(i_p4);
    
        return make_pair(true, result);
    }
    else return make_pair(false, result);
}

int LaneCheck(){
    //./LaneCheck.cpp "L[0] L[1] R[0] R[1]"
    
    int status = system("./LaneCheck 660 468 0 742 870 468 1400 740");
    return WEXITSTATUS(status);
}

//void 다크프로그래머의장애물위치구하기(){
//
//}
//
//void 곡선주행하기(){
//
//}
//void 없는차선의장애물y값까지이동(){
//    다크프로그래머의장애물위치구하기();
//    곡선주행하기();
//}
//void 앞에노란선을검출해서흰선정지선까지움지경(){
//
//}
//void 멈춰(){
//    
//}
int main(){
    //흰선으로중앙맞추기();
    int go = LaneCheck();
    
    cout << LaneCheck() << endl;
    //없는차선의장애물y값까지이동();
    //앞에노란선을검출해서흰선정지선까지움지경();
    //멈춰();
    //execve("신호등");
    return 0;
}
