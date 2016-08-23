#include <ros/ros.h>

//include message libraries
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <uav_vision/DetectInfo.h>

//include opencv libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

//include ros transport&bridge libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

vector<vector<Point> > dollContours;
ros::Publisher pubLedPos;
ros::Publisher pubLedImgeInfo;

vector<vector<Point> > drawDoll(Mat img)
{
    Mat imgThreshold, imgGray;
    cvtColor(img,imgGray,CV_BGR2GRAY);
    threshold(imgGray,imgThreshold,200,255,CV_THRESH_BINARY);
    Mat element = getStructuringElement(0,Size(3,3));
    morphologyEx(imgThreshold, imgThreshold, CV_MOP_ERODE, element, Point(-1,-1),2);
    morphologyEx(imgThreshold, imgThreshold, CV_MOP_DILATE, element, Point(-1,-1),2);
    blur(imgThreshold,imgThreshold,Size(3,3));
    //imshow("threshold",imgThreshold);
    vector<vector<Point> > contours;
    findContours(imgThreshold,contours,RETR_TREE,CHAIN_APPROX_NONE);
    return contours;
}

Mat dollDetect(Mat img, vector<vector<Point> > dollContours, float &dollType)
{
    vector<vector<Point> > srcContours  = drawDoll(img);
    for(int dollInd = 0; dollInd<dollContours.size();dollInd++)
    {
        for(int contInd = 0; contInd<srcContours.size(); contInd++)
        {
            if(contourArea(srcContours[contInd])>300)
            {
                double similarityParam = matchShapes(dollContours[dollInd], srcContours[contInd],CV_CONTOURS_MATCH_I1,0);
                //cout<<similarityParam<<endl;
                if(similarityParam < 0.01)
                {
                    drawContours(img, srcContours, contInd, Scalar(0,0,255),3);
                    if(dollInd == 0)
                        putText(img,"hippo", srcContours[contInd][0], 5, 1, Scalar(255,0,0), 2);
                    if(dollInd ==1)
                        putText(img,"octopus", srcContours[contInd][0], 5, 1, Scalar(255,0,0), 2);
                    dollType = (float)(dollInd + 1);
                }
            }
        }
    }
    return img;
}

void basketDetect(Mat& img, float &dollType, uav_vision::DetectInfo region4Info)
{
    Mat imgGray;
    cvtColor(img, imgGray, CV_BGR2GRAY);
    GaussianBlur(imgGray, imgGray, Size(3,3), 1);
    Mat element = getStructuringElement(0, Size(3,3));
    morphologyEx(imgGray, imgGray, CV_MOP_OPEN,element,Point(-1,-1),2);
    Mat imgCanny;
    Canny(imgGray, imgCanny, 30, 90);

    vector<vector<Point> > contours;
    vector<Point>         contourDP;
    contourDP.resize(contours.size());
    findContours(imgCanny, contours, RETR_TREE,CHAIN_APPROX_NONE);
    for(int i=0;i<contours.size();i++)
    {
        float contArea = contourArea(contours[i]);
        approxPolyDP(contours[i], contourDP, double(contours[i].size()) * 0.05, true);
        if(contArea > 2000 &&isContourConvex(Mat(contourDP)) && contourDP.size() == 4)
        {
            drawContours(img, contours, i, Scalar(0,0,255),2);
            float centerX, centerY;
            for(int i=0;i<4;i++)
            {
                centerX += contourDP[i].x;
                centerY += contourDP[i].y;
            }
            region4Info.data.data.push_back(centerX/4);
            region4Info.data.data.push_back(centerY/4);
        }
    }
}

void callbackRegion4Detect(const cv_bridge::CvImage::ConstPtr& msg)
{
    Mat src_frame = msg->image;

//    float timeNow = getTickCount();
    float dollType;
    uav_vision::DetectInfo region4Info;//The kind and position of doll
    Mat imgShow = dollDetect(src_frame, dollContours, dollType);
    region4Info.data.data.push_back(dollType);
    //basketDetect(imgShow, dollType, region4Info);

    pubLedPos.publish(region4Info);
//    timeNow = ((double)getTickCount()-timeNow)/getTickFrequency();
//    cout << 1000*timeNow << endl;
    imshow("show",imgShow);
    waitKey(1);
    // Draw the ""Cross line" at the image center
    int lineLenth = 5;
    line(imgShow,
         Point(imgShow.cols / 2, imgShow.rows / 2 - lineLenth),
         Point(imgShow.cols / 2, imgShow.rows / 2 + lineLenth),
         Scalar(0, 0, 255));
    line(imgShow,
         Point(imgShow.cols / 2 - lineLenth, imgShow.rows / 2),
         Point(imgShow.cols / 2 + lineLenth, imgShow.rows / 2),
         Scalar(0, 0, 255), 2);

    cv_bridge ::CvImage outImg;
    outImg.image = imgShow;
    outImg.header.stamp = ros::Time::now();
    pubLedImgeInfo.publish(outImg);
}

int main(int argc, char** argv)
{
    Mat octopusMode = imread("/home/yf/hippoMode.png");
    Mat hippoMode   = imread("/home/yf/octopusMode.png");
    vector<vector<Point> > octopusModeContours = drawDoll(octopusMode);
    vector<vector<Point> > hippoModeContours = drawDoll(hippoMode);
    vector<Point> octopusModeContour = octopusModeContours[0];
    vector<Point> hippoModeContour = hippoModeContours[0];
    dollContours.push_back(octopusModeContour);
    dollContours.push_back(hippoModeContour);

    ros::init(argc, argv, "uav_led_detect");
    ros::NodeHandle nh;
    ros::Subscriber subImg;

    cout << "The led detection node is setup!" << endl;

    pubLedPos = nh.advertise<uav_vision::DetectInfo>("/uav_vision/detectLed", 1000);
    pubLedImgeInfo = nh.advertise<cv_bridge::CvImage>("/uav_vision/infoLed", 2);
    subImg = nh.subscribe<cv_bridge::CvImage>("/uav_cam/image", 5, callbackRegion4Detect);


    ros::spin();
    return 0;
}
