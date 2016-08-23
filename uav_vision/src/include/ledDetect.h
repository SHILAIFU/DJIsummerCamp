#ifndef LEDDETECT_H
#define LEDDETECT_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class LedDetect{
public:
    Mat orbMatching(Mat img_mode,Mat img);
    vector<vector<Point>> drawDoll(Mat img);
    void dollDetect(Mat img, vector<Point> modeContour, int sign);
};
#endif
