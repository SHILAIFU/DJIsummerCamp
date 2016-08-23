/**
 * @file CONTROL_CODE
 * @name Shan.5
 * @date Aug 3,2016
 *
 **/
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <math.h>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "PID_lib/pid.h"
#include "flight_logic.h"
#include "control_node.h"
//#include "guidance/open_cv.h"
#include "dji_sdk_control/pid_ctrl_data.h"

#define DEBUG

using namespace DJI::onboardSDK;
using namespace std;
using namespace Eigen;
using namespace cv;

unsigned char ctrl_flag;
ros::Subscriber core_ctrl_sub;
ros::Subscriber core_cmd_sub;
Ctrl_data ctrl_data;

bool pose_update;
bool land_flag = false;

MISSION_STATUS mission_status = m100_INIT;

volatile double markerInfo[6] = {0.0}; //mark rectify

const float missioHeight = 1.700;
const float flightHeight = 0.800;

bool MarkerFlag = false;
int command;

DJIDrone* drone;

//ros::Publisher pid_ctrl;

ros::Subscriber ctrl_vel_sub;

// ros::Subscriber odometry_sub;

float ctrl_x;
float error[3];

Matrix3d cMb;

//ros::Subscriber odometry_sub;

//void pid_callback(const dji_sdk_control::pid_ctrl_data &pid_data);

void obtain_control_callback(const ros::TimerEvent&)
{
        ROS_INFO("obtain control callback");
        drone->request_sdk_permission_control();
}


//void core_ctrl_callback()
//{

//}

void core_cmd_callback(const std_msgs::UInt8& msg)
{
    switch (msg.data)
    {
    case m100_INIT:
    {
        if(drone->request_sdk_permission_control())                      //car message
                {
                    mission_status = m100_TAKEOFF;
                    cout<<"obtain controll command has been send!"<<endl;
                }
    }
        break;
    case m100_TAKEOFF:
    {
        if( drone->takeoff() )
        {
            mission_status =  m100_STAND_BY;
            cout<<"takeoff command has been send!"<<endl;
        }
    }
        break;



    case m100_LANDING:
    {
        if( drone->landing() )
        {
            mission_status = m100_LANDING;
            cout<<"landing command has been send!"<<endl;
        }
    }
        break;
    default:
        break;
    }
}


void ctrl_vel_callback(const std_msgs::Float32MultiArray::ConstPtr& ctrl_vel)
{
    float vx = ctrl_vel->data[0];
    float vy = ctrl_vel->data[1];
    float vz = ctrl_vel->data[2];
    float yaw = ctrl_vel->data[3];
    float land = ctrl_vel->data[4];
    if(land == 1.0)
        land_flag = true;
    else
        land_flag = false;

    cout<<"ctrl_vel_callback"<<endl;
    cout<<"vx="<<vx<<endl;
    cout<<"vy="<<vy<<endl;
    cout<<"vz="<<vz<<endl;
    cout<<"yaw"<<yaw<<endl;
    cout<<"land_state"<<land<<endl;

    if(land_flag)
        {
         drone->landing();       
         cout<<"landing"<<land<<endl;
        }
    else
        {
//       cout<<"attitude control"<<endl;
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                Flight::VerticalLogic::VERTICAL_VELOCITY |
                Flight::YawLogic::YAW_ANGLE |
                Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                Flight::SmoothMode::SMOOTH_ENABLE,
                vx, vy, vz, yaw);
        }
   usleep(20000);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dji_sdk_control");
    ROS_INFO("dji_sdk_control_node");
    ros::NodeHandle nh;
    ros::Timer obtain_control_timer=nh.createTimer(ros::Duration(1.0),obtain_control_callback);  //Duration(1.0)
    drone = new DJIDrone(nh);

    ctrl_vel_sub = nh.subscribe("/control_velocity", 50, ctrl_vel_callback);

//    core_ctrl_sub = nh.subscribe("/core_ctrl", 10 ,core_ctrl_callback);
    core_cmd_sub  = nh.subscribe("/core_cmd", 10, core_cmd_callback);
//   debug core_cmd_sub
//    drone->request_sdk_permission_control();
//    sleep(2);
//    ROS_INFO("ready to go!");
//     drone -> takeoff();
/* debug*/
    sleep(1);

    ros::Rate rate(50);    //控制频率50Hz
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
