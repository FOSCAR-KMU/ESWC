#include <opencv2/opencv.hpp>
#include <iostream>
#include <utility>

#define PI 3.1415
#define HEIGHT 750
#define WIDTH 1334

#define CENTER 400

using namespace cv;
using namespace std;


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
    Mat img = imread("차있는사진.JPG");
    resize(img, img, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_LINEAR);

    //Mat img(HEIGHT, WIDTH, CV_8UC3);
    //TopView(img2, img);
    imshow("Original",img);
    Rect rect(0, HEIGHT * 2 / 3, WIDTH, HEIGHT/3);
    Mat roi = img(rect);

    imshow("ROI", roi);

    Mat gray;
    cvtColor(roi, gray, COLOR_BGR2GRAY);

    Mat bin;
    threshold(gray, bin, 150, 255, THRESH_BINARY);    

    imshow("Bin", bin);

    Mat edges;
    Canny(gray, edges, 250, 500);
    imshow("Edges",edges);
    vector<cv::Vec2f> lines;
    HoughLines(edges, lines, 1, PI/180, 130);
    
    // 선 그리기
    cv::Mat result(edges.rows, edges.cols, CV_8U, cv::Scalar(255));
    std::cout << "Lines detected: " << lines.size() << std::endl;

    vector<pair<Point2f, Point2f> > line_point;

    // 선 벡터를 반복해 선 그리기
    std::vector<cv::Vec2f>::const_iterator it= lines.begin();
    while (it!=lines.end()) {
        float rho = (*it)[0];   // 첫 번째 요소는 rho 거리
        float theta = (*it)[1]; // 두 번째 요소는 델타 각도
        Point pt1;
        Point pt2;
        if (theta < PI/4. || theta > 3.*PI/4.) { // 수직 행
            pt1 = Point(rho/cos(theta), 0); // 첫 행에서 해당 선의 교차점   
            pt2 = Point((rho-result.rows*sin(theta))/cos(theta), result.rows);
            // 마지막 행에서 해당 선의 교차점
            cv::line(img, pt1, pt2, cv::Scalar(255), 1); // 하얀 선으로 그리기
        } else { // 수평 행
            pt1 = Point(0,rho/sin(theta)); // 첫 번째 열에서 해당 선의 교차점  
            pt2 = Point(result.cols,(rho-result.cols*cos(theta))/sin(theta));
            // 마지막 열에서 해당 선의 교차점
            cv::line(roi, pt1, pt2, cv::Scalar(255), 1); // 하얀 선으로 그리기
        }
        std::cout << "line: (" << pt1 << "," << pt2 << ")\n";
        line_point.push_back(make_pair(pt1, pt2));
        ++it;
    }

    vector<Point2f> result_point;

    vector<pair<Point2f, Point2f> >::iterator iter1;
    for(iter1 = line_point.begin() ; iter1 != line_point.end() ; ++iter1)
    {
        Point temp_intersect_point1, temp_intersect_point2;
        get_intersectpoint(Point(0, 0), Point(0, 1334), (*iter1).first, (*iter1).second, &temp_intersect_point1);
        get_intersectpoint(Point(750, 0), Point(750, 1334), (*iter1).first, (*iter1).second, &temp_intersect_point2);
        result_point.push_back(temp_intersect_point1);
        result_point.push_back(temp_intersect_point2);
    }

    vector<Point2f>::iterator iter2;
    for(iter2 = result_point.begin() ; iter2 != result_point.end() ; ++iter2)
    {
        cout << (*iter2).x << ' ' << (*iter2).y << endl;
    }

    // Mat labels, centers;
    // kmeans(result_point, 2, labels,
  	// 			TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  	// 			3, KMEANS_RANDOM_CENTERS, centers);

    imshow("Lines",img);
    
    int k = waitKey(0);
    if (k == 27) {       // wait for ESC key to exit
        destroyAllWindows();
    }

    return 0;
}