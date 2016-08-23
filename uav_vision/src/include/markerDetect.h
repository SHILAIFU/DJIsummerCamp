/**
  * Author   : Yanan Guo
  * Data     : 2016-8-7
  * Discribe : Detect the marker
  * Version  : 2.0
  *
  */

#ifndef MARKERDETECT_H
#define MARKERDETECT_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include <ctype.h>
#include <map>
#include <utility>
#include <string>
#define ColorType int
#define HSV   0
#define HSV_H 1
#define HSV_S 2
#define HSV_V 3
#define BGR   4
#define BGR_B 5
#define BGR_G 6
#define BGR_R 7

using namespace cv;
using namespace std;

class Utilities{
public:
    Utilities();
    void  readCameraParam(Mat cameraMatrix, vector<double> distCoeffsMat);
    void  lkTracking(Mat &frame);
    void  camshiftTracking(Mat img, Rect selection);
    Mat   colorConversion(Mat img, ColorType hsvtype);
    Mat   imgProc(Mat img);
    Mat   findMarkerCandidate(Mat img, int flyDirection, int _markerArea = 10000);
    bool  findMiniBox(Mat imgROI, float markerArea);
    Mat   setROI(Mat threshold, Point2f vertex[]);
    vector<double> getPosition(Mat &img, Point2f* pointsIn);
    bool  orbMatching(Mat img_mode,Mat img);
    bool  contourFilter(float valY);
    vector<vector<Point>>   drawDoll(Mat img);
    void dollDetect(Mat img, vector<Point> modeContours, int sign);
private:
    Mat imgSrc, gray_prev, gray, imgROI, imgMode, imgROI_3_changel;
    vector<Mat> img_mode;

    //vector<float> vertexPoint;
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> flowPointsIn;
    vector<Point2f> flowPointsOut;
    bool ifFindMarker = true;
    //void argParams();

    Rect camshiftBox;
    int vmin = 10;
    int vmax = 256;
    int smin = 30;
    int hsize = 16;

    Rect trackWindow;
};
#endif //MARKERDETECT_H

