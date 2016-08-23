/** @file Find the square up position and publish it
 * @author Dremtale
 * @date Aug 5, 2016
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

#include "detectSquareUp.h"

ros::Publisher pubSquareUpPosi;
ros::Publisher pubImageInfo;
using namespace std;

/**
   +-----------+-----------+----+---++-----------+-----------+----+---++-----------+-----------+----+---+
   | posi_r0_x | posi_r0_y |... | x || posi_y0_x | posi_y0_y |... | x || posi_b0_x | posi_b0_y |... | x |
   +-----------+-----------+----+---++-----------+-----------+----+---++-----------+-----------+----+---+
----- x = - 1
*/
/*  Just get the bule object
   +-----------+-----------+
   | posi_b0_x | posi_b0_y |
   +-----------+-----------+
*/
void callBackSquareUpTrack(const cv_bridge::CvImage::ConstPtr& msg)
{
    Mat src = msg->image;
    if(!src.empty())
    {
      SquareUp mySquare;
      mySquare.init();
      // Get the color squares position information
      Mat dst;
      vector< vector <Point> > squarePosiInfo = mySquare.detect(src, dst);
      // Make the position info into package to publish

      cv_bridge::CvImage outInfo;
      outInfo.encoding = "bgr8";
      outInfo.header.stamp = ros::Time::now();

      // Draw the ""Cross line" at the image center
      int lineLenth = 5;
      line(dst,
           Point(dst.cols / 2, dst.rows / 2 - lineLenth),
           Point(dst.cols / 2, dst.rows / 2 + lineLenth),
           Scalar(255, 0, 255));
      line(dst,
           Point(dst.cols / 2 - lineLenth, dst.rows / 2),
           Point(dst.cols / 2 + lineLenth, dst.rows / 2),
           Scalar(255, 0, 255), 2);

      outInfo.image = dst;

      //imshow("testWin", dst);
      outInfo.header.stamp = ros::Time::now();
      pubImageInfo.publish(outInfo.toImageMsg());

      for(int i = 0; i < squarePosiInfo.size(); i++)
      {
        cout << i << " :  ";
        for(int j = 0; j < squarePosiInfo[i].size(); j++)
          cout << squarePosiInfo[i][j];
        cout << endl;
      }

      uav_vision::DetectInfo myPubInfo;
//      for(int i = 0; i < squarePosiInfo.size(); i++)
//      {
//        for(int j = 0; j < squarePosiInfo[i].size(); j++)
//        {
//          myPubInfo.data.data.push_back(squarePosiInfo[i][j].x);
//          myPubInfo.data.data.push_back(squarePosiInfo[i][j].y);
//        }
//        // -1 is the split symbol
//        myPubInfo.data.data.push_back(-1);
//      }

      // Storaged as bgr, we select the bule square and throw

      if(squarePosiInfo[2].size() > 0)
      {
        myPubInfo.data.data.push_back((float)squarePosiInfo[2][0].x);
        myPubInfo.data.data.push_back((float)squarePosiInfo[2][0].y);
      }
      else
      {
        myPubInfo.data.data.push_back(-1);
        myPubInfo.data.data.push_back(-1);
      }
      pubSquareUpPosi.publish(myPubInfo);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "detect_square_up");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);
    ros::Subscriber subImg;

    pubSquareUpPosi = nh.advertise<uav_vision::DetectInfo>("/uav_vision/detectSquareUp", 100);

    pubImageInfo = nh.advertise<cv_bridge::CvImage>("/uav_vision/infoSquareUp", 5);

    cout << "The square-up detection node is setup!" << endl;

    subImg = nh.subscribe("/uav_cam/image", 5, callBackSquareUpTrack);

    while(nh.ok())
    {
        loopRate.sleep();
        ros::spinOnce();
    }

    return 0;
}
