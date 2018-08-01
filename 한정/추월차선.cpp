#include <opencv2/opencv.hpp>
#include <iostream>


#define HEIGHT 750
#define WIDTH 1334

using namespace cv;
using namespace std;

//1이 위쪽
Point2f L[2];
Point2f R[2];

void 차선검출(){
    img = cv2.imread('road.jpg',1)
    cv2.imshow('Original',img)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    cv2.imshow('Gray',gray)
    edges = cv2.Canny(gray,250,500,apertureSize = 3)
    cv2.imshow('Edges',edges)
    lines = cv2.HoughLines(edges,1,np.pi/180,170)

    for rho,theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

    cv2.imshow('Lines',img)

    k = cv2.waitKey(0)
    if k == 27:         // wait for ESC key to exit
        cv2.destroyAllWindows()
    }
}

void 흰선으로중앙맞추기(){
    점1,2 = 차선검출();
    //중앙맞추자.
}
void 버드뷰()
    Mat src = imread("장애물유.JPG");
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

    Mat dst2(HEIGHT, WIDTH, CV_8UC3);

    warpPerspective(src_vertices, dst_vertices, getPerspectiveTransform(src_vertices, dst_vertices), dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    imshow("src", src);
    imshow("dst2", dst2);
    waitKey();
);

void 장애물없는차선검출(){

}

void 다크프로그래머의장애물위치구하기(){

}

void 곡선주행하기(){

}
void 없는차선의장애물y값까지이동(){
    다크프로그래머의장애물위치구하기();
    곡선주행하기();
}
void 앞에노란선을검출해서흰선정지선까지움지경(){

}
void 멈춰(){
    
}
int main(){
    흰선으로중앙맞추기();
    버드뷰();
    장애물없는차선검출();
    없는차선의장애물y값까지이동();
    앞에노란선을검출해서흰선정지선까지움지경();
    멈춰();
    execve("신호등");
}
