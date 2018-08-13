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
    // imshow("Original",img);
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

    vector<Vec4i> lines;

    int threshold = 40;

    HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 30, 3);

//     HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
//     for( size_t i = 0; i < lines.size(); i++ )
//     {
//       Vec4i l = lines[i];
//       line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
//     }

    int carXpos = 500;
    int carYpos;

    Mat result;
    Mat h_points = Mat(lines.size() * 2, 1, CV_32FC2);
    for(size_t i = 0 ; i < lines.size() ; i++)
    {
        Vec4f l = lines[i];
        Point temp_intersect_point1;
        Point temp_intersect_point2;
        cout << l[0] << ' ' << l[1] << endl;
        cout << l[2] << ' ' << l[3] << endl;

        if(l[0] - l[2] == 0 || l[1] - l[3] == 0) continue;
        float a = (float)(l[1] - l[3])/(float)(l[0] - l[2]);

        get_intersectpoint(Point(l[0]-1000, l[1] - 1000*a), Point(l[2] + 1000, l[3] + 1000*a), Point(carXpos, 0), Point(carXpos, 1334), &temp_intersect_point1);
        get_intersectpoint(Point(l[0]-1000, l[1] - 1000*a), Point(l[2] + 1000, l[3] + 1000*a), Point(750, 0), Point(750, 1334), &temp_intersect_point2);

        cout << "I_P : " << temp_intersect_point1.x << ' ' << temp_intersect_point1.y << endl;
        cout << "I_P : " << temp_intersect_point2.x << ' ' << temp_intersect_point2.y << endl;

        h_points.at<Point2f>((i*2)  , 0) = temp_intersect_point1;
        h_points.at<Point2f>((i*2)+1, 0) = temp_intersect_point2;
        // line(roi, temp_intersect_point1, temp_intersect_point2, Scalar(0,0,255), 3, CV_AA);
    }

    int clusterCount = 4;
    Mat labels, centers;
    kmeans(h_points, clusterCount, labels,
  		   TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 10, 1.0),
  		   3, KMEANS_RANDOM_CENTERS, centers);

    for(int i = 0 ; i < 4 ; i++)
    {
        cout << centers.at<Point2f>(i, 0).x << ' ' << centers.at<Point2f>(i, 0).y << endl;
    }
    line(roi, Point(centers.at<Point2f>(0,0)), Point(centers.at<Point2f>(1,0)), Scalar(0,0,255), 3, CV_AA);
    line(roi, Point(centers.at<Point2f>(2,0)), Point(centers.at<Point2f>(1,0)), Scalar(0,0,255), 3, CV_AA);

    imshow("result", roi);


    int k = waitKey(0);
    if (k == 27) {       // wait for ESC key to exit
        destroyAllWindows();
    }

}