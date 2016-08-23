#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <vector>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iostream>
using namespace std;
using namespace DJI::onboardSDK;

// The sort of uav control signals
enum uavCtrlSignal
{
    uavTakeOff = 0,
    uavLanding,
    uavHangSlient,
    uavGoFlightHeigh,
    uavGoMissionHeigh,
    uavMovVelocity,
    uavForceMannul,
    uavGetCtrlAbility,
    uavControlSum
};

// Define a global DIJ drone
DJIDrone* drone;
const float missioHeight = 1.700;
const float flightHeight = 0.800;

// void coreLogicFunc(vector<int> &_cmd)
void callBackMarkerCtrl(const std_msgs::Int32MultiArray::ConstPtr& msg)
{

    int cmd = msg->data[0];
    cout<< "I got command: "<< cmd << endl;
    double correntX, correntY, correntZ;
    switch (cmd) {
    case uavGetCtrlAbility:
        drone->request_sdk_permission_control();
        break;

    case uavMovVelocity:
//        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
//                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
//                                 Flight::YawLogic::YAW_ANGLE |
//                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
//                                 Flight::SmoothMode::SMOOTH_ENABLE,
//                                 (float)msg->data[1] / 100,                 // x
//                                 (float)msg->data[2] / 100,                 // y
//                                 (float)msg->data[3] / 100,                 // z
//                                 (float)msg->data[4] / 100);                // yaw


        cout << "+++" << (float)msg->data[1] / 100 << endl;
        usleep(20000);
        break;

    case uavTakeOff:
        drone->takeoff();   // fly to 1.2m

        break;
    case uavGoFlightHeigh:



        break;
    case uavGoMissionHeigh:

        break;
    case uavLanding:

        break;
    case uavForceMannul:
        // If wanna switch to mannual control, release the control ability
        drone->release_sdk_permission_control();

        break;

    case uavHangSlient:     // Default status is hang slient
    default:
        break;
    }
}

//void callBackMarkerCtrl(const std_msgs::Float32MultiArray::ConstPtr& msg)
//{
//    vector<int> cmd;
//    if(msg->data[3] > 50.0)
//    {
//        cmd.push_back(uavMovLeft);
//        coreLogicFunc(cmd);
//    }
//    if(msg->data[3] < -50.0)
//    {
//        cmd.push_back(uavMovRight);
//        coreLogicFunc(cmd);
//    }
//    if(msg->data[4] > 50.0)
//    {
//        cmd.push_back(uavMovBack);
//        coreLogicFunc(cmd);
//    }
//    if(msg->data[4] < -50.0)
//    {
//        cmd.push_back(uavMovFront);
//        coreLogicFunc(cmd);
//    }
//}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "onboard_sdk");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);

    drone = new DJIDrone(nh);

    //ros::Subscriber subMarkTrack;
    ros::Subscriber subCoreLogic;

    //subMarkTrack = nh.subscribe("/uav_vision/findMarker", 100, callBackMarkerCtrl);
    subCoreLogic = nh.subscribe("/uav_ctrl/onboard", 100, callBackMarkerCtrl);
    drone->request_sdk_permission_control();
    cout<<"ready to take off"<<endl;

    drone->takeoff();   // fly to 1.2m
    sleep(10);
    cout<<"take off"<<endl;


    float x = drone->local_position.x;

    double KP = 1;
    double KD = 0.1;

    while (x != 1.2)
    {
        float Control_x;

        cout << "local_position.x = " << drone->local_position.x << endl;
        cout << "local_position.y = " << drone->local_position.y << endl;
        cout << "local_position.z = " << drone->local_position.z << endl;

        float x = drone->local_position.x;

        Control_x = KP * (1.2 - x) ;

        cout << "The Control_x : " << Control_x << endl;

        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                                     Flight::VerticalLogic::VERTICAL_VELOCITY |
                                     Flight::YawLogic::YAW_ANGLE |
                                     Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                     Flight::SmoothMode::SMOOTH_ENABLE,
                                     Control_x,                 // x
                                     0,                  // y
                                     0,                 // z
                                     0);               // yaw
        usleep(20000);

    }

    while(nh.ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
