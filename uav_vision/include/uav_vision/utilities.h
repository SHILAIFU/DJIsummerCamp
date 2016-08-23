#ifndef UTILITIES_H
#define UTILITIES_H

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
using namespace cv;
#define HsvType int
#define HSV   0
#define HSV_H 1
#define HSV_S 2
#define HSV_V 3

#define BGR   4
#define BGR_B 5
#define BGR_G 6
#define BGR_R 7

class Utilities{
public:
    Utilities();
    Mat colorConversion(Mat img, HsvType hsvtype);
    Mat decectMarkerContour(Mat img);
    Mat getPosition(Mat &img, Point2f* pointsIn);

    //void argParams();
};
#endif
