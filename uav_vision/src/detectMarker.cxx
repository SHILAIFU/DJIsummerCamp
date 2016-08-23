/** @file  Find the color apperent object
 * @author DreamTale
 * @date   Jul 30, 2016
  */

#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <ctype.h>
#include <map>
#include <utility>
#include <cmath>
// Include ros and data transport headers
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
// Include image transport & bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

// Include self defined msg header
#include <uav_vision/DetectInfo.h>

#define HSV         0x00
#define HSV_H       0x01
#define HSV_S       0x02
#define HSV_V       0x03
#define BGR         0x04
#define BGR_B       0x05
#define BGR_G       0x06
#define BGR_R       0x07

typedef int ColorType;
typedef int HsvType;

using namespace std;
using namespace cv;

// Global parameters
ros::Publisher pubMarkerPosi;
Mat imgSrc, grayPrev, gray, imgROI, imgMode, imgROI3Changel;
vector<Mat> vecImgMode;
vector<uchar> status;
vector<float> err;
vector<Point2f> flowPointsIn;
vector<Point2f> flowPointsOut;
bool ifFindMarker = true;
//void argParams();

Rect camshiftBox;
int globalVmin  = 10;
int globalVmax  = 256;
int globalSMin  = 30;
int globalHSize = 16;

Rect trackWindow;

// +--------+--------+--------++--------+--------+--------+
// | rvec_x | rvec_y | rvec_z || tvec_x | tvec_y | tvec_z |
// +--------+--------+--------++--------+--------+--------+

Mat   colorConversion(Mat img, ColorType hsvtype);
Mat   imgProc(Mat img);
Mat   findMarkerCandidate(Mat img, int flyDirection, int _markerArea = 10000);
Mat   setROI(Mat threshold, Point2f vertex[]);
void  lkTracking(Mat &frame);
void  camshiftTracking(Mat img, Rect selection);
void  getPosition(Mat &img, Point2f* pointsIn);
void  dollDetect(Mat img, vector<Point> modeContours, int sign);
bool  findMiniBox(Mat imgROI, float markerArea);
bool  contourFilter(float valY);
vector<vector<Point> > drawDoll(Mat img);

void callBackMarkerTrack(const cv_bridge::CvImage::ConstPtr& msg)
{
    //Matrix to store each frame of the webcam feed
    Mat src = msg->image;

    Mat dst = findMarkerCandidate(src, 0, 10000);

    imshow("testWindow", dst);

    waitKey(1);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "find_marker");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Subscriber subImg;

    pubMarkerPosi = nh.advertise<uav_vision::DetectInfo>("/uav_vision/detectMarker", 1000);

    subImg = nh.subscribe("/uav_cam/image", 5, callBackMarkerTrack);

    while(nh.ok())
    {ros::spinOnce();}

    return 0;
}

Mat colorConversion(Mat img, ColorType imgtype){

    GaussianBlur(img, img, Size(5,5),0,0);

    if(imgtype==HSV||imgtype==HSV_H||imgtype==HSV_S||imgtype==HSV_V){
        Mat img_hsv, img_h, img_s, img_v;
        cvtColor(img, img_hsv, CV_BGR2HSV);
        vector<Mat> hsv_channels;
        split(img_hsv, hsv_channels);
        img_h = hsv_channels[0];
        img_s = hsv_channels[1];
        img_v = hsv_channels[2];
        switch (imgtype){
        case 0:
            return img_hsv;
        case 1:
            return img_h;
        case 2:
            return img_s;
        case 3:
            return img_v;
        }
    }

    if(imgtype==BGR||imgtype==BGR_B||imgtype==BGR_G||imgtype==BGR_R){
        Mat b_img, g_img, r_img;
        vector<Mat> bgr_channels;
        split(img,bgr_channels);
        b_img=bgr_channels[0];
        g_img=bgr_channels[1];
        r_img=bgr_channels[2];

        switch(imgtype){
        case 4:
            return img;
        case 5:
            return b_img;
        case 6:
            return g_img;
        case 7:
            return r_img;
        }
        return img;
    }
}

bool findMiniBox(Mat imgROI, float markerArea)
{
    Mat imgThreshold;
    //mini rectangle
    vector<vector<Point> > contours;
    vector<Point> approxCurve;
    approxCurve.resize(contours.size());
    vector<Vec4i> hierarchy;
    if(imgROI.channels() != 1)
        imgThreshold = imgProc(imgROI);
    else
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
        imgROI.copyTo(imgThreshold);
    //vector<Rect> box(contours.size());
    findContours(imgThreshold,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
    for(size_t i=0;i<contours.size();i++)
    {
        if(markerArea > contourArea(contours[i]) && contourArea(contours[i]) > 0.03*markerArea)
            return true;
        //drawContours(imgROI, contours, i, Scalar(0,0,255),5);
    }

    return false;
}

Mat setROI(Mat img, Point2f vertex[])
{

    Mat mask(img.rows,img.cols,img.type(),Scalar::all(0));
    vector<vector<Point> > maskContours;
    vector<Point> ptr;
    for(int i=0;i<4;i++){
        ptr.push_back(Point(vertex[i].x,vertex[i].y));
    }
    maskContours.push_back(ptr);
    drawContours(mask,maskContours,0,Scalar::all(255),-1);
    Mat imgROI;
    img.copyTo(imgROI,mask);
    return imgROI;
}

Mat imgProc(Mat img)
{
    Mat imgSingle, imgThreshold;

    //split the single channel
    imgSingle=colorConversion(img, BGR_R);
    threshold(imgSingle,imgThreshold,0,255,CV_THRESH_BINARY + CV_THRESH_OTSU );
    //image process
    Mat element = getStructuringElement(MORPH_RECT,Size(3,3));
    morphologyEx(imgThreshold,imgThreshold,MORPH_ERODE,element);

    //    imshow("threshold image",imgThreshold);

    Canny(imgThreshold,imgThreshold,30,90);

    return imgThreshold;
}

/**
 * @brief findMarkerCandidate
 * @param img
 * @param flyDirection
 * @param _markerArea
 * @return The Image to debug
 */
Mat findMarkerCandidate(Mat img, int flyDirection, int _markerArea) // _markerArea default is 10000
{
    imgSrc = img.clone();
    Mat imgThreshold = imgProc(img);
    //Define the vertex of marker candidate
    Point2f pointsIn[4]={Point2f(0,0)};
    Mat imgShow(imgThreshold.rows, imgThreshold.cols,CV_8UC3,Scalar(0,0,0));
    vector<vector<Point> > contours;
    vector<Point> approxCurve;
    vector<Vec4i> hierarchy;
    float markerCandidateArea = 0;

    findContours(imgThreshold,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
    approxCurve.resize(contours.size());

    Point2f minVal = Point(INT_MAX,INT_MAX);
    int mini = 0;
    for(size_t i=0; i<contours.size();i++)
    {
        float contArea=contourArea(contours[i]);
        //remove the noise by limit the area of the contour
        if(contArea > _markerArea)
        {
            markerCandidateArea = contArea;
            //fiting polygon
            approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.05, true);
            if (isContourConvex(Mat(approxCurve)) && approxCurve.size() == 4)
            {
                int minX = INT_MAX;
                int minIndex = 0;
                for(int j=0;j<4;j++)
                {
                    if(sqrt(approxCurve[j].x*approxCurve[j].x+approxCurve[j].y*approxCurve[j].y) < minX){
                        minX = sqrt(approxCurve[j].x*approxCurve[j].x+approxCurve[j].y*approxCurve[j].y);
                        minIndex = j;
                    }
                }
                if(minIndex == 0 )
                {
                    for(int j=0;j<4 ;j++){
                        pointsIn[j] = approxCurve[j];
                        circle(imgShow,Point(pointsIn[j].x,pointsIn[j].y),8,Scalar(0,0,255));
                    }
                }
                else
                {
                    vector<int> tmp;//{1,2,3,0,1,2,3};
                    tmp.push_back(1);tmp.push_back(2);tmp.push_back(3);
                    tmp.push_back(0);
                    tmp.push_back(1);tmp.push_back(2);tmp.push_back(3);

                    vector<int>::iterator tmp_itr = tmp.begin();
                    int k=0;
                    tmp_itr = find(tmp.begin(),tmp.end(),minIndex);

                    for(; minIndex!=0 && tmp_itr!=tmp.end() && k!=4; tmp_itr++)
                    {
                        pointsIn[k++] = approxCurve[*tmp_itr];
                    }
                }
                imgROI = setROI(imgThreshold, pointsIn);
                bool ifFindMiniBox = findMiniBox(imgROI, markerCandidateArea);
                if(ifFindMiniBox)
                {
                    bool ifMatching = 1;
                    float cmpVal   = flyDirection ? pointsIn[0].x:pointsIn[0].y;
                    float banchVal = flyDirection ? minVal.x:minVal.y;
                    if(ifMatching == true && cmpVal < banchVal)
                    {
                        minVal = pointsIn[0];
                        mini = i;
                    }
                }
                else continue;
                //rectangle(src_frame, boundRect[i], Scalar(0,0,255), 2,8,0);
            }

        }

    }
    float noiseVal = flyDirection ? pointsIn[0].y : pointsIn[0].x;
    bool ifNoise = contourFilter(noiseVal);
    if(!ifNoise && mini !=0)
    {
        drawContours(imgShow,contours, mini,Scalar(0,255,0),2,8);
        getPosition(img, pointsIn);
    }

    return imgShow;
}

bool contourFilter(float noiseVal)
{
    static int oldNoiseVal = noiseVal;
    if(abs(noiseVal - oldNoiseVal) > 200)
        return true;
    else
    {
        oldNoiseVal = noiseVal;
        return false;
    }
}

/**
 * @brief Utilities::getPosition
 * @param img
 * @param pointsIn
 * @return The main position we'll publish it
 */
void getPosition(Mat &img, Point2f* pointsIn)
{
    vector<Point3f> objectPoints;               //Size of the Marker
    objectPoints.push_back(Point3f(0,0,0));
    objectPoints.push_back(Point3f(410,0,0));
    objectPoints.push_back(Point3f(410,410,0));
    objectPoints.push_back(Point3f(0,410,0));

    vector<Point2f> imagePoints;
    for(int i=0;i<4;i++)
    {
        imagePoints.push_back(Point2f(pointsIn[i].x,pointsIn[i].y));
    }


    //    // The camera 0.0 pameraters
    //    float tmp[3][3] = {{720.0694,0,319.5}
    //                       ,{0, 720.0694, 239.5}
    //                       ,{0,0,1}};
    //    float tmp2[5] = {-0.175335,0.360192,0,0,-1.464377};

    // The camera 2.2 pameraters
    float tmp[3][3] = {{482.3176, 0,       324.5216},
                       {0,        482.9905, 219.4612},
                       {0,        0,        1      }};
    float tmp2[5] = {-0.409179,0.1566728,-0.00132855,-0.00103737, 0};

    Mat cameMatrix(3,3,CV_32FC1, tmp);
    Mat distCoeffs(1,5,CV_32F,tmp2);

    Mat rotat_vec(3,3,CV_32F), trans_vec(3,3,CV_32F);

    solvePnP(objectPoints,imagePoints,cameMatrix,distCoeffs,rotat_vec,trans_vec);
    //    cout<<"rotat_vec:"<<rotat_vec<<endl;
    //    cout<<"trans_vec:"<<trans_vec<<endl;
    //    cout<<"center   :"<<trans_vec.at<double>(0,0)+205<<", "<<trans_vec.at<double>(0,1)+205<<endl;
    //    cout<<"*****************************"<<endl;

    // Publish the main infomaiton
    uav_vision::DetectInfo detectInfo;

    detectInfo.header.stamp = ros::Time::now();

    detectInfo.data.data.push_back(rotat_vec.at<double>(0, 0));
    detectInfo.data.data.push_back(rotat_vec.at<double>(0, 1));
    detectInfo.data.data.push_back(rotat_vec.at<double>(0, 2));

    detectInfo.data.data.push_back(trans_vec.at<double>(0, 0));
    detectInfo.data.data.push_back(trans_vec.at<double>(0, 1));
    detectInfo.data.data.push_back(trans_vec.at<double>(0, 2));

    pubMarkerPosi.publish(detectInfo);
}

