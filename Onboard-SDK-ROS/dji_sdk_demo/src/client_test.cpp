/*@file client
 *@author shan.5
 * 2016.08.08
 * */
#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <dji_sdk/dji_drone.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <vector>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iostream>
#include <sys/select.h>
using namespace std;
using namespace DJI::onboardSDK;

// The sort of uav control signals
enum m100_CtrlSignal
{
    m100_takeOff = 0x0001,
    m100_landing = 0x0002,
    m100_hangSlient = 0x0004,
    m100_goFlightHeigh = 0x0008,
    m100_goMissionHeigh = 0x0010,
    m100_movVelocity = 0x0020,
    m100_forceMannul = 0x0040,
    m100_getCtrlAbility = 0x0080,
    //m100_controlSum
};
// missions
enum m100_state
{
    m100_standby,
    m100_throwDoll,
};


volatile double droneVoInfo[9] = {0.0};
volatile double markerInfo[6] = {0.0};

const float missioHeight = 1.700;
const float flightHeight = 0.800;

bool MarkerFlag = false;

int command;

// Define a global DIJ drone
DJIDrone* drone;

bool inputAvilable()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_set(STDIN_FILENO);

}

bool position_control()
{
    float delta_x = 0.05;
    float delta_y = 0.05;
    float delta_z = 0.02;



    drone ->attitude_control(Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                             Flight::VerticalLogic::VERTICAL_VELOCITY |
                             Flight::YawLogic::YAW_ANGLE |
                             Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                             Flight::SmoothMode::SMOOTH_ENABLE,
                             vx,vy,vz,yaw);

}
// MARKER RECTIFY
void rectify()
{
    static int turn = 0;
    while (MarkerFlag)
    {
        ros::spinOnce();
    }

}

void menu()
{
    cout << "Welcome !"<< endl;
}


// void coreLogicFunc(vector<int> &_cmd)
void callBackMarkerCtrl(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
}
/*
 *   add later for Marker
void callBackMarkerCtrl(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    vector<int> cmd;
    if(msg->data[3] > 50.0)
    {
        cmd.push_back(uavMovLeft);
        coreLogicFunc(cmd);
    }
    if(msg->data[3] < -50.0)
    {
        cmd.push_back(uavMovRight);
        coreLogicFunc(cmd);
    }
    if(msg->data[4] > 50.0)
    {
        cmd.push_back(uavMovBack);
        coreLogicFunc(cmd);
    }
    if(msg->data[4] < -50.0)
    {
        cmd.push_back(uavMovFront);
        coreLogicFunc(cmd);
    }
}
*/
void callBackGuidanceinfo(const nav_msgs::Odometry& guidance_odometry )
{

}



int main(int argc, char *argv[])
{
    float x_coordinate,y_coordinate;

    ros::init(argc, argv, "onboard_sdk");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);

    ros::Subscriber subGuidanceInfo;
    subGuidanceInfo = nh.subscribe("/guidance/odometry",100,callbackguidanceinfo);

    //ros::Subscriber subMarkTrack;
    //ros::Subscriber subCoreLogic;

    ros::Subscriber subDetectMarker;
    subDetectMarker = nh.subscribe("/uav_guider/velPoseByVo",100,callBackMarkerCtrl);// add later********

    //subMarkTrack = nh.subscribe("/uav_vision/findMarker", 100, callBackMarkerCtrl);
    //subCoreLogic = nh.subscribe("/uav_ctrl/onboard", 100, callBackMarkerCtrl);
    //drone->request_sdk_permission_control();
    cout<<"ready to take off"<<endl;

    drone = new DJIDrone(nh);

    drone->takeoff();   // fly to 1.2m
    sleep(10);

    while (nh.ok())
    {
        ros::spinOnce();
        meum();

        switch (command)
        {
         case'':

          break;
         case'':

          break;
        default:
            break;
        }
        usleep(20000);
    }

    while(nh.ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
