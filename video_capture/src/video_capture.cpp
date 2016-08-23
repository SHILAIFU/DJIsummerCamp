#include <ros/ros.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/time.h>
#include <string>

using namespace std;
using namespace cv;

#define WIDTH 640
#define HEIGHT 480

long Get_time(void)
{
   struct timeval cur_time;
   gettimeofday(&cur_time,NULL);
   return cur_time.tv_sec;
}

int main( int argc,char** argv )
{
    ros::init(argc,argv,"video_capture");
    ros::NodeHandle nh; 
    string path="/home/ubuntu/capture_video/";
    long time=Get_time();
    stringstream ss;
    ss<<time;
    VideoWriter raw_writer;
    raw_writer.open((path+ss.str()+".avi"),CV_FOURCC('D', 'I', 'V', 'X'),20.0,cvSize(WIDTH,HEIGHT),true);
    VideoCapture capture;
    capture.open(0);
    Mat image;
    while(ros::ok())
    {
       capture >> image;
       if(!image.empty())
       {
          raw_writer.write(image);
          imshow("Image", image);
       }
       waitKey(1);
    }
    capture.release();
    return 0;
}
