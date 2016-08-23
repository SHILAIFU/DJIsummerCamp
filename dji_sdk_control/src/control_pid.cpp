/*@file   control_pid
 *@author shan.5
 *@date   2016.08.03
*/

#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include "PID_lib/pid.h"
#include <Eigen/Dense>
#include "dji_sdk_control/pid_ctrl_data.h"
#include "guidance/open_cv.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace ros;
using namespace cv;
using namespace DJI::onboardSDK;
using namespace Eigen;

ros::Publisher ctrl_vel_pub;      //m100_control
ros::Publisher servo_pub;         //servo_control(add later)
ros::Publisher guidance_bias_pub; //set orignal when get pid control
ros::Publisher real_pos_pub;

ros::Subscriber odometry_sub;     //get the infomation from guidance
ros::Subscriber target_pos_sub;     //get the infomation from corelogic
ros::Subscriber landing_active_sub; //sub

ros::Subscriber car_state_sub;    //permit land

DJIDrone* drone;

PID *ctrl_x;
PID *ctrl_y;
PID *ctrl_z;
PID *ctrl_yaw;

/*+++++++++++++++++++++++++Position PID_control+++++++++++++++++++++++++++*/
bool pose_update = true;             //lock the yaw_flag
bool vision_active = false;
bool land_flag = false;
string car_state;

double Kp_pos = 0.5;
double Ki_pos = 0.0;
double Kd_pos = 0.1;

double Kp_height = 0.2;
double Ki_height = 0.0;
double Kd_height = 0.0;

double Kp_yaw = 0.2;
double Ki_yaw = 0.0;
double Kd_yaw = 0.0;

float ctrl_data[4] = {0,0,0,0};
float landing_data[4] = {0,0,0,0};
float target_position[3] = {0,0,1};
float target_yaw = 0.0;
double body_yaw = 0.0;
double alpha = 0.0;
double yaw_ori = 0.0;

float dt = 0.01;
float first_time = 0.0;

double pid_ctrl_limit = 0.2;
double pid_ctrl_limit_vert = 0.2;
double pid_yaw_limit = 5;

Vector3d bodyCoordinate;
/*+++++++++++++++++++add by 35 ++++++++++++++++++++++++++++++++++++++++++++++++*/
/*------------------------read PID Param---------------------------------------*/
void readParam()
{
    FileStorage fs("/home/ubuntu/control_node.yml", FileStorage::READ);

    fs["Kp_pos"] >> Kp_pos;
    fs["Ki_pos"] >> Ki_pos;
    fs["Kd_pos"] >> Kd_pos;

     fs["Kp_height"] >> Kp_height;
     fs["Ki_height"] >> Ki_height;
     fs["Kd_height"] >> Kd_height;

     fs["Kp_yaw"] >> Kp_yaw;
     fs["Ki_yaw"] >> Ki_yaw;
     fs["Kd_yaw"] >> Kd_yaw;
}

/*--------------------get car message---------------------------------------------*/
void carStateCallback(const std_msgs::String& msg)
{
    car_state = msg.data; //"carStart"   "forLand"   "pushDone"
    if(car_state == "forLand")
    {
        land_flag = true;
    }
    else
    {
        land_flag = false;
    }

}


/*+++++++++++++++++++++++++++guidence guide position++++++++++++++++++++++++*/
void target_pos_callback(const geometry_msgs::Vector3 target_pos)
{
    target_position[0] = target_pos.x;
    target_position[1] = target_pos.y;
    target_position[2] = target_pos.z;

    ctrl_x->set_point(target_position[0]);
    ctrl_y->set_point(target_position[1]);
    ctrl_z->set_point(target_position[2]);
    ctrl_yaw->set_point(0);

}
/*+++++++++++++++aruco _control callback+++++++++++++++++++++++++++++++++++++*/
void landing_active_callback(const guidance::open_cv& msg)
{
//      vision_active = msg.landing_mark;
    std_msgs::Float32MultiArray velocity;
//    float set_high = 30;

    if((msg.landing_mark == 1) && land_flag)                      //get aruco  message ready land
    {
        vision_active = true;
        landing_data[0] = msg.landing_x_out;
        landing_data[1] = msg.landing_y_out;
        landing_data[2] = msg.landing_z_out;
        landing_data[3] = body_yaw*180.0/M_PI + msg.landing_yaw_out;

        if(msg.Landing_Flag)
            landing_data[4] = 1.0;
        else
            landing_data[4] = 0.0;

        velocity.data.push_back(landing_data[0]);
        velocity.data.push_back(landing_data[1]);
        velocity.data.push_back(landing_data[2]);
        velocity.data.push_back(landing_data[3]);
        velocity.data.push_back(landing_data[4]);

        ctrl_vel_pub.publish(velocity);

        ROS_INFO("laout_x=%f",landing_data[0]);
        ROS_INFO("laout_y=%f",landing_data[1]);
        ROS_INFO("laout_z=%f",landing_data[2]);
    }
    else if(msg.landing_mark == 0)
    {
        vision_active = false;
        landing_data[0] = 0;
        landing_data[1] = 0;
        landing_data[2] = 0;
        landing_data[3] = 0;
        landing_data[4] = 0;
    }
}

// odometry_cllback  pub VX VY VZ value

/*+++++++++++++++++++++++++++++++guidance_odometry_callback++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void guidance_odometry_callback(const nav_msgs::Odometry& guidance_odometry) {
    //float q0 = drone->attitude_quaternion.q0, q1 = drone->attitude_quaternion.q1, q2 = drone->attitude_quaternion.q2,q3 = drone->attitude_quaternion.q3;

    float q0 = guidance_odometry.pose.pose.orientation.w, q1 = guidance_odometry.pose.pose.orientation.x, q2 = guidance_odometry.pose.pose.orientation.y,q3 = guidance_odometry.pose.pose.orientation.z;
    body_yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));        //real time yaw value

    if(pose_update)
    {
     //float q0 = guidance_odometry.pose.pose.orientation.w, q1 = guidance_odometry.pose.pose.orientation.x, q2 = guidance_odometry.pose.pose.orientation.y,q3 = guidance_odometry.pose.pose.orientation.z;
     //body_yaw = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
        yaw_ori = body_yaw;
//       ang_cov = yaw_ori -  3*M_PI/4.0;
        alpha = yaw_ori*180.0/M_PI;
        pose_update = false;
    }
        ROS_INFO("alpha=%f",alpha);
    bodyCoordinate(0) = guidance_odometry.pose.pose.position.x;
    bodyCoordinate(1) = guidance_odometry.pose.pose.position.y;
    bodyCoordinate(2) = guidance_odometry.pose.pose.position.z;

        Matrix3d cMb; 
        double sina=sin(yaw_ori) ,cosa=cos(yaw_ori);
        cMb(0,0) = cosa;       //cosa;
        cMb(0,1) = sina;
        cMb(0,2) = 0;
        cMb(1,0) = -sina;
        cMb(1,1) = cosa;
        cMb(1,2) = 0;
        cMb(2,0) = 0;
        cMb(2,1) = 0;
        cMb(2,2) = 1;
        bodyCoordinate = cMb*bodyCoordinate;

    geometry_msgs::Vector3 real_pos;

    real_pos.x = bodyCoordinate(0);
    real_pos.y = bodyCoordinate(1);
    real_pos.z = bodyCoordinate(2);

    real_pos_pub.publish(real_pos);

    cout <<"Tx="<< bodyCoordinate(0) <<endl;
    cout <<"Ty="<< bodyCoordinate(1) <<endl;
    cout <<"Tz="<< bodyCoordinate(2) <<endl;

    /* Publish the output control velocity from PID controller */
    dt = guidance_odometry.header.stamp.toSec() - first_time;
    first_time = guidance_odometry.header.stamp.toSec();
    //guidance_odometry.twist.twist.linear.x =
    //ROS_INFO("GU_Z=%f",guidance_odometry.pose.pose.position.z);
    cout<<"GU_Z=%f"<<guidance_odometry.pose.pose.position.z<<endl;
    ctrl_data[0] = ctrl_x -> update(bodyCoordinate(0), dt);
    ctrl_data[1] = ctrl_y -> update(bodyCoordinate(1), dt);
    ctrl_data[2] = ctrl_z -> update(bodyCoordinate(2), dt); //target_position[2]; // position control logic for z-axis
    ctrl_data[3] = alpha;     //yaw
    //ROS_INFO("ctrl_x=%f",ctrl_x);

    if (ctrl_data[0] > pid_ctrl_limit)
        ctrl_data[0] = pid_ctrl_limit;
    if (ctrl_data[0] < -pid_ctrl_limit)
        ctrl_data[0] = -pid_ctrl_limit;
    //ctrl_data[0] = ctrl_data[0] > pid_ctrl_limit ?

    if (ctrl_data[1] > pid_ctrl_limit)
        ctrl_data[1] = pid_ctrl_limit;
    if (ctrl_data[1] < -pid_ctrl_limit)
        ctrl_data[1] = -pid_ctrl_limit;

    if (ctrl_data[2] > pid_ctrl_limit)
        ctrl_data[2] = pid_ctrl_limit;
    if (ctrl_data[2] < -pid_ctrl_limit)
        ctrl_data[2] = -pid_ctrl_limit;
    //ROS_INFO("ctrl_yaw=%f",ctrl_data[3]);
    //geometry_msgs::Vector3 velocity;
   // geometry_msgs::Vector3;
    if((!vision_active )||(!land_flag))                      //No aruco message
    {
    std_msgs::Float32MultiArray velocity;
    ROS_INFO("ctrl_x=%f",ctrl_data[0]);
    ROS_INFO("ctrl_y=%f",ctrl_data[1]);
    ROS_INFO("ctrl_z=%f",ctrl_data[2]);

    velocity.data.push_back(ctrl_data[0]);
    velocity.data.push_back(ctrl_data[1]);
    velocity.data.push_back(ctrl_data[2]);
    velocity.data.push_back(ctrl_data[3]);
    velocity.data.push_back(0.0);

//    if(guidance_odometry.pose.pose.position.z > 1.5)
//    {
//        velocity.z = 0;
//    }
        cout<<"control_pid"<<endl;
        ctrl_vel_pub.publish(velocity);
    }

}

void position_control()
{

}

/*+++++++++++++++++++++++++++marker position get control+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
//void marker_position_callback(const geometry_msgs::Vector3& marker_pos)
//{
//    float error_x,error_y,error_z;
//    error_x = marker_pos.x;
//    error_y = marker_pos.y;
//    error_z = marker_pos.z;

//    ctrl_x->set_point(target_position[0]);
//    ctrl_y->set_point(target_position[1]);
//    ctrl_z->set_point(target_position[2]);

//    ctrl_data[0] = ctrl_x -> update(marker_pos.x, dt);
//    ctrl_data[1] = ctrl_y -> update(marker_pos.y, dt);
//    ctrl_data[2] = ctrl_z -> update(marker_pos.z, dt); //target_position[2]; // position control logic for z-axis
//    ctrl_data[3] = alpha;     //yaw

//    if (ctrl_data[0] > pid_ctrl_limit)
//        ctrl_data[0] = pid_ctrl_limit;
//    if (ctrl_data[0] < -pid_ctrl_limit)
//        ctrl_data[0] = -pid_ctrl_limit;
//    //ctrl_data[0] = ctrl_data[0] > pid_ctrl_limit ?

//    if (ctrl_data[1] > pid_ctrl_limit)
//        ctrl_data[1] = pid_ctrl_limit;
//    if (ctrl_data[1] < -pid_ctrl_limit)
//        ctrl_data[1] = -pid_ctrl_limit;

//    if (ctrl_data[2] > pid_ctrl_limit)
//        ctrl_data[2] = pid_ctrl_limit;
//    if (ctrl_data[2] < -pid_ctrl_limit)
//        ctrl_data[2] = -pid_ctrl_limit;

//    std_msgs::Float32MultiArray velocity;

//    if(vision_active)
//    {
//    velocity.data.push_back(ctrl_data[0]);
//    velocity.data.push_back(ctrl_data[1]);
//    velocity.data.push_back(ctrl_data[2]);
//    velocity.data.push_back(ctrl_data[3]);

//       ctrl_vel_pub.publish(velocity);
//    }
//}
/*+++++++++++++++++++++++++++marker position get control+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*y jiaozheng */


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_pid");
    ros::NodeHandle nh;
    std_msgs::UInt8 bias_correct_msg;

    readParam();
    drone = new DJIDrone(nh);
//    nh.param("Kp_pos",Kp_pos,0.2);
//    nh.param("Ki_pos",Ki_pos,0.0);
//    nh.param("Kd_pos",Kd_pos,0.1);

//    nh.param("Kp_height",Kp_pos,0.4);
//    nh.param("Ki_height",Ki_pos,0.0);
//    nh.param("Kd_height",Kd_pos,0.0);
    cout << "kp  "<< Kp_pos <<endl;
    cout << "Kp_height   "<<Ki_pos <<endl;
    cout << "Kp_yaw  "<< Kd_pos <<endl;

    ctrl_x   = new PID( Kp_pos, Ki_pos, Kd_pos, -5, 5, -pid_ctrl_limit, pid_ctrl_limit, false);
    ctrl_y   = new PID( Kp_pos, Ki_pos, Kd_pos, -5, 5, -pid_ctrl_limit, pid_ctrl_limit, false);
    ctrl_z   = new PID( Kp_height, Ki_height, Kd_height, -5, 5, -pid_ctrl_limit_vert, pid_ctrl_limit_vert, false);
    ctrl_yaw = new PID( Kp_yaw, Ki_yaw, Kd_yaw, -5, 5, -pid_yaw_limit, pid_yaw_limit, false);

    ctrl_x   ->set_point(target_position[0]);
    ctrl_y   ->set_point(target_position[1]);
    ctrl_z   ->set_point(target_position[2]);
    ctrl_yaw ->set_point(target_yaw);

    ros::Rate loop_rate(48);

    target_pos_sub = nh.subscribe("/target_position",10,target_pos_callback);
    odometry_sub = nh.subscribe("/guidance/odometry", 10, guidance_odometry_callback);
//  marker_position_sub   = nh.subscribe("/marker_position", 1, marker_position_callback);//  add later
    landing_active_sub = nh.subscribe("/landing_car",10,landing_active_callback);
    car_state_sub = nh.subscribe("/car_state" , 10 , carStateCallback);

    real_pos_pub = nh.advertise<geometry_msgs::Vector3>("/drone_pos",20);
    ctrl_vel_pub = nh.advertise<std_msgs::Float32MultiArray>("/control_velocity", 20);
    guidance_bias_pub=nh.advertise<std_msgs::UInt8>("/guidance/bias",20);

    cout << "init success!"<<endl;
    bias_correct_msg.data = 1;

    while(ros::ok())
    {
        guidance_bias_pub.publish(bias_correct_msg);
//     cout << "init success!"<<endl;
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

