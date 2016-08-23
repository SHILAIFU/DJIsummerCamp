/** @file  Find the dolls and publish the position object
 * @author DreamTale
 * @date   Jul 30, 2016
  */

#include <sstream>
#include <string>
#include <iostream>
#include <vector>
// Include ros and data transport headers
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <uav_vision/DetectInfo.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
// Include image transport & bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "squareDetect.h"

ros::Publisher pubDollPosi;
using namespace std;

// +----------+----------++----------+----------++----------+----------+
// | posi_r_x | posi_r_y || posi_y_x | posi_y_y || posi_b_x | posi_b_y |
// +----------+----------++----------+----------++----------+----------+

RotatedRect detectSquare(Mat img)
{
    //split the channel
    vector<Mat> img_split;
    Mat img_single;
    split(img,img_split);
    img_single = img_split[2].clone();
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    float maxContourArea = 0;
    int maxContourArea_index;

    Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(img_single,img_single,MORPH_CLOSE, element);
    threshold(img_single, img_single, 0, 255, CV_THRESH_BINARY_INV + CV_THRESH_OTSU);// Use the OTSU algorithm
    //imshow("img_single",img_single);
    findContours(img_single, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    // Find the max area of contours
    for (int i=0;i<contours.size();i++)
    {
        int contourSize = contourArea(contours[i]);
        if(contourSize > maxContourArea)
        {
            maxContourArea = contourSize;
            maxContourArea_index = i;
        }
    }
    RotatedRect box = minAreaRect(contours[maxContourArea_index]);
    return box;
}

Mat detectDoll(Mat img, RotatedRect box)
{
    // Set ROI
    Mat mask(img.rows,img.cols,CV_8UC3,Scalar::all(0));
    Point2f vertex[4];
    box.points(vertex);
    vector<vector<Point> > maskContours;
    vector<Point> ptr;
    for(int i=0;i<4;i++)
    {
        ptr.push_back(Point(vertex[i].x,vertex[i].y));
    }
    maskContours.push_back(ptr);
    drawContours(mask,maskContours,0,Scalar::all(255),-1);
    Mat imgROI;
    img.copyTo(imgROI,mask);

    // convert to HSV
    Mat img_hsv_blue, img_hsv_red1,img_hsv_red2,img_hsv_yellow;
    cvtColor(imgROI, img_hsv_blue, CV_BGR2HSV);
    Mat img_threshold_blue, img_threshold_red, img_threshold_red1, img_threshold_red2, img_threshold_yellow;

    Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(img_hsv_blue,img_hsv_blue ,MORPH_OPEN,element,Point(-1,-1),3);
    morphologyEx(img_hsv_blue,img_hsv_blue ,MORPH_CLOSE,element,Point(-1,-1),3);

    img_hsv_red1   = img_hsv_blue.clone();
    img_hsv_red2   = img_hsv_blue.clone();
    img_hsv_yellow = img_hsv_blue.clone();

    Mat blue_low(Scalar(60,43,46));
    Mat blue_higher(Scalar(140,255,255));

    Mat red1_low(Scalar(0,43,46));
    Mat red1_higher(Scalar(3,255,255));

    Mat red2_low(Scalar(170,43,46));
    Mat red2_higher(Scalar(180,255,255));

    Mat yellow_low(Scalar(20,43,46));
    Mat yellow_higher(Scalar(34,255,255));

    inRange(img_hsv_blue,   blue_low,   blue_higher,   img_threshold_blue);
    inRange(img_hsv_red1,   red1_low,   red1_higher,   img_threshold_red1);
    inRange(img_hsv_red2,   red2_low,   red2_higher,   img_threshold_red2);
    inRange(img_hsv_yellow, yellow_low, yellow_higher, img_threshold_yellow);
    img_threshold_red = img_threshold_red1 | img_threshold_red2;

    //imshow("123",img_threshold_red1);

    vector<vector<Point> > contours_blue, contours_red, contours_yellow;
    vector<Vec4i> hierarchy_blue, hierarchy_red, hierarchy_yellow;
    findContours(img_threshold_blue,   contours_blue,hierarchy_blue,     CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    findContours(img_threshold_red,    contours_red,hierarchy_red,       CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    findContours(img_threshold_yellow, contours_yellow,hierarchy_yellow, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    // Find the max area of contours

    double rgbDollPosi[6] = { 0.0 };

    for (int i=0;i<contours_blue.size();i++)
    {
        int contourSize = contourArea(contours_blue[i]);
        if(contourSize>6000)    // TODO: need to be changed
        {
            Rect box = boundingRect(contours_blue[i]);
            rectangle(img,box,Scalar(255,0,0),5);

            //            // Method 1, out of date
            //            // Publish BLUE the doll position
            //            std_msgs::Int32MultiArray outPubMsg;
            //            outPubMsg.data.push_back(2);
            //            outPubMsg.data.push_back(box.x + box.width / 2);
            //            outPubMsg.data.push_back(box.y + box.height / 2);
            //            pubDollPosi.publish(outPubMsg);
            rgbDollPosi[4] = rgbDollPosi[4] + (float)box.x + (float)box.width / 2;
            rgbDollPosi[5] = rgbDollPosi[5] + (float)box.y + (float)box.height / 2;
        }
    }

    if(contours_blue.size() > 0)
    {
        rgbDollPosi[4] = rgbDollPosi[4] / contours_blue.size();
        rgbDollPosi[5] = rgbDollPosi[5] / contours_blue.size();
    }

    for (int i=0;i<contours_red.size();i++){
        int contourSize = contourArea(contours_red[i]);
        if(contourSize>10000)       // TODO: need to be changed
        {
            Rect box = boundingRect(contours_red[i]);
            rectangle(img,box,Scalar(0,0,255),5);
            //            // Publish BLUE the doll position
            //            std_msgs::Int32MultiArray outPubMsg;
            //            outPubMsg.data.push_back(0);
            //            outPubMsg.data.push_back(box.x + box.width / 2);
            //            outPubMsg.data.push_back(box.y + box.height / 2);
            //            pubDollPosi.publish(outPubMsg);
            rgbDollPosi[0] = rgbDollPosi[0] + (float)box.x + (float)box.width / 2;
            rgbDollPosi[1] = rgbDollPosi[1] + (float)box.y + (float)box.height / 2;
        }
    }
    if(contours_red.size() > 0)
    {
        rgbDollPosi[0] = rgbDollPosi[0] / contours_blue.size();
        rgbDollPosi[1] = rgbDollPosi[1] / contours_blue.size();
    }

    for (int i=0;i<contours_yellow.size();i++)
    {
        int contourSize = contourArea(contours_yellow[i]);
        if(contourSize>10000)       // TODO: need to be changed
        {
            Rect box = boundingRect(contours_yellow[i]);
            rectangle(img,box,Scalar(0,255,255),5);
            //            // Publish BLUE the doll position
            //            std_msgs::Int32MultiArray outPubMsg;
            //            outPubMsg.data.push_back(1);
            //            outPubMsg.data.push_back(box.x + box.width / 2);
            //            outPubMsg.data.push_back(box.y + box.height / 2);
            //            pubDollPosi.publish(outPubMsg);

            rgbDollPosi[2] = rgbDollPosi[2] + (float)box.x + (float)box.width / 2;
            rgbDollPosi[3] = rgbDollPosi[3] + (float)box.y + (float)box.height / 2;
        }
    }

    if(contours_yellow.size() > 0)
    {
        rgbDollPosi[2] = rgbDollPosi[2] / contours_blue.size();
        rgbDollPosi[3] = rgbDollPosi[3] / contours_blue.size();
    }

    // Now, publish the data we got
    uav_vision::DetectInfo dollDetectInfo;
    dollDetectInfo.header.stamp = ros::Time::now();
    double checkSum = 0;
    for(int i = 0; i < 6; i++)
    {
        dollDetectInfo.data.data.push_back(rgbDollPosi[i]);
        if(!(rgbDollPosi < 0))
            checkSum = checkSum + rgbDollPosi[i];
        cout << ", " << rgbDollPosi[i];
    }

    cout << endl;
    if(checkSum > 1.0)
        pubDollPosi.publish(dollDetectInfo);

    imshow("dst",img);
    return img;
}


void callBackMarkerTrack(const cv_bridge::CvImage::ConstPtr& msg)
{
    Mat src = msg->image;

    RotatedRect box = detectSquare(src);
    Point2f vertex[4];
    box.points(vertex);
    for(int i=0;i<4;i++)
    {
        line(src,vertex[i],vertex[(i+1)%4],Scalar(255,0,0),5);
    }
    //rectangle(img_src,box,Scalar(0,255,0),5);

    Mat imgROI = detectDoll(src, box);
    //time_start = getTickCount() - time_start;
    //cout << "run time:" << time_start / getTickFrequency() << "s" << endl;

    waitKey(10);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "find_doll");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate loopRate(100);
    ros::Subscriber subImg;

    pubDollPosi = nh.advertise<uav_vision::DetectInfo>("/uav_vision/detectDoll", 100);

    subImg = nh.subscribe("/uav_cam/image", 5, callBackMarkerTrack);

    while(nh.ok())
    {
        loopRate.sleep();
        ros::spinOnce();
    }

    return 0;
}
