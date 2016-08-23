/**********DJIsummerCamp*****************
 * @file: core_logic
 * @Note:
 * @Date: 2016.08.05
 * @Author: shan.5_W
 ***************************************/
#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include <dji_sdk/dji_drone.h>
#include "flight_logic.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
//#include <visualization_msgs/Marker.h>
#include "guidance/open_cv.h"
#include <opencv2/opencv.hpp>

using namespace DJI::onboardSDK;
using namespace std;
using namespace ros;

MISSION_STATUS last_mission;
MISSION_STATUS current_mission;
MISSION_STATUS next_mission;

bool vision_active;        //vison_control
std_msgs::UInt8 cmd_msg;
string car_state;

//bool carStart = false;
//bool carForland = false;
//bool carPushDone = false;
bool landing_car = false;
bool color = false;
bool release = true;
float pos[3] ={0.0,0.0,0.0};
int t = 0;
int dt = 0;

ros::Publisher target_pos_pub;     // Publish the target position to PID controller
ros::Publisher cmd_msg_pub;        //// Publish the control message (obtain control, takeoff, landing,etc to)
ros::Publisher stm32_cmd_pub;      //publish the command to stm32

//ros::Subscriber vison_activation_sub; //sub
ros::Subscriber vo_guidance_sub;      //pid control node
ros::Subscriber car_state_sub;
ros::Subscriber landing_sub;
//ros::Subscriber odometry_sub;
ros::Subscriber real_pos_sub;
ros::Subscriber sudoku_sub;


DJIDrone* drone;

void wait_key()
{
    cin.get();
    cin.ignore();
}

//  Delay for x seconds
void delay_s(int x)
{
    ros::Duration(x).sleep();
}

void target_point(double x, double y, double z)
{
    int counter = 0;
    geometry_msgs::Vector3 target_pos;

    target_pos.x = x;
    target_pos.y = y;
    target_pos.z = z;
    /* Publish 5 times */
    while(counter < 5)
    {
        target_pos_pub.publish(target_pos);
        counter++;
    }
}

void sudoku_callback(const guidance::open_cv& sudo_msg )
{
    if(sudo_msg.sudoku_color != 0)
    {
        color = true;
    }
    else
    {
        color = false;
    }
}

void real_position_callback(const geometry_msgs::Vector3& real_position)
{
   pos[0] = real_position.x;
   pos[1] = real_position.y;
   pos[2] = real_position.z;
}


void carStateCallback(const std_msgs::String& msg)
{
    car_state = msg.data; //"carStart"   "forLand"   "pushDone"

}

void landCarCallback(const guidance::open_cv& msg )
{
    if(msg.Landing_Flag)
    {
        landing_car = true;
    }
    else
        landing_car = false;
}


int mission_run()
{

    switch(current_mission)
    {
    /*------------------------------------------------------------------/
     *   INIT
    -------------------------------------------------------------------*/
        case m100_INIT:
        {
            ROS_INFO("Initializing...");
            delay_s(5); // Wait for 5 seconds
            t = t +1;
            cmd_msg.data = m100_INIT;

            int i = 0;
            while(i<5)
            {
            cmd_msg_pub.publish(cmd_msg);
            i++;
            }
            ROS_INFO("Ready for Takeoff");
         if(car_state == "carStart" || car_state == "pushDone" || (t > 36))
           {
             t = 0;
             current_mission = m100_TAKEOFF;
              ROS_INFO("Ready");
            }
            break;
        }
    /*------------------------------------------------------------------/
     *   TAKEOFF
    -------------------------------------------------------------------*/
        case m100_TAKEOFF:
        {
            cmd_msg.data = m100_TAKEOFF;
            ROS_INFO("Press any key to takeoff");

            int i = 0;
            while(i<10)
            {
                cmd_msg_pub.publish(cmd_msg);
                i++;
            }

            ROS_INFO("Taking off...");

            delay_s(6); // Taking off, wait for 6 seconds
            current_mission = GO2ZONE2;
            break;
        }

/*------------------------------------------------------------------/
 *   GO 2 ZONE2
-------------------------------------------------------------------*/
    case  GO2ZONE2:
    {
        ROS_INFO("Get Octopus! go to zone 2!");
        ROS_INFO("Mission starts!");
        ROS_INFO("Moving to ZONE 2!");
        target_point(0, 5.7, 1);
       if(color || (pos[1] > 5.0))            //y >3
            {
            current_mission = m100_STAND_BY;
            }
        break;
    }

/*------------------------------------------------------------------/
 *   stand by
-------------------------------------------------------------------*/
    case m100_STAND_BY:               //zone2
    {
//        cmd_msg.data = m100_STAND_BY;
//        cmd_msg_pub.publish(cmd_msg);
        ROS_INFO("m100_STAND_BY(0, 6.0, 0.5");
        target_point(0, 6.0, 0.5);
        sleep(1);
        if(( pos[0] < 1) && (pos[1] > 5.3) && color)
        {
            current_mission = release_OCTOPUS;
            break;
        }

        ROS_INFO("Next step: Move to (-0.3,6)");
        ROS_INFO("m100_STAND_BY(-0.3, 6, 0.7)");
        target_point(-0.3, 6.0, 0.5);
        sleep(1);
        if(( pos[0] < 0.7) && (pos[1] > 5.3) && color)
        {
            current_mission = release_OCTOPUS;
            break;
        }

        ROS_INFO("m100_STAND_BY(-0.5, 4, 0.5)");
        target_point(-0.3, 5.7, 0.5);
        sleep(1);
        if(( pos[0] < 0.7) && (pos[1] > 5.3) && color)
        {
            current_mission = release_OCTOPUS;
            break;
        }

        ROS_INFO("Next step: Move to (0,4)");
        ROS_INFO("m100_STAND_BY(-0.0, 5.5, 0.7)");
        target_point(0, 5.5, 0.5);                 //zhong jian point
        sleep(1);
        if(( pos[0] < 0.7) && (pos[1] > 5.3) && color)
        {
            current_mission = release_OCTOPUS;
            break;
        }

        target_point(0, 5.7, 0.5);                 //zhong jian point
        ROS_INFO("Mission accomplished!");
        ROS_INFO("release_OCTOPUS...");
        usleep(6000000); // Landing, wait for 6 seconds
       if(( pos[0] < 0.7) && (pos[1] > 5.3) || color)
       {
           current_mission = release_OCTOPUS;
       }

//       if(landing_car && release)
//        {
//            current_mission = CAR_LAND;
//        }
        break;
    }


/*------------------------------------------------------------------/
 *   release    OCTOPUS
-------------------------------------------------------------------*/
    case release_OCTOPUS:
    {
        std_msgs::String cmd;
        cmd.data = "Release";
        stm32_cmd_pub.publish(cmd);
        ROS_INFO("Octopus released");
        release = true;
//        release_time = ros::Time::now().sec;
        current_mission = searchCar;
        break;
    }

/*------------------------------------------------------------------/
 *   search Car
-------------------------------------------------------------------*/
    case searchCar:
    {

        target_point(0.5, 5.7, 1);            //car position
        sleep(3);
//        target_point(-2, 5.7, 1);
//        sleep(3);
        dt = dt + 1;
        if ((car_state == "forLand") || dt > 10)
        {
            //release = false;
            dt = 0;
            current_mission = CAR_LAND;
        }
        break;
    }


/*------------------------------------------------------------------/
 *   CAR_LAND
-------------------------------------------------------------------*/
    case  CAR_LAND:
    {
        ROS_INFO("Find aruco!Ready to land");


        target_point(-2.0, 5.7, 0.8);
        sleep(4);

        if( pos[0] < -1.5 )
        {
            target_point(0, 5.7, 0.8);
            sleep(4);
        }

 //       target_point(-1.0, 5.7, 0.6);
 //       sleep(1);

 //     target_point(-0.5, 5.7, 0.6);
 //       sleep(1);

 //       target_point(0.0, 5.7, 0.6);            //car position
 //       sleep(1);

        ROS_INFO("Mission starts!");
        ROS_INFO(" Land on car!");

        if(landing_car)
        {
            current_mission = m100_INIT;
        }
        break;
    }
/*------------------------------------------------------------------/
 *   serching
-------------------------------------------------------------------*/

/*------------------------------------------------------------------/
 *     LANDING
-------------------------------------------------------------------*/
    case m100_LANDING:
        {
            cmd_msg.data = m100_LANDING;
            int i;
            while(i<10)
            {
               cmd_msg_pub.publish(cmd_msg);
                i++;
            }
               cmd_msg.data = m100_RELEASE_CONTROL;
               cmd_msg_pub.publish(cmd_msg);
            current_mission = m100_INIT;
        break;
        }
    }
}

/*+++++++++++++++++++++++++++  Main_Menu  ++++++++++++++++++++++++++++++++++++++++++++++*/

static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
    printf("| [w] Press 'w' to[0, 4,1]!     | [r] Press 'r' to release!       |\n");
    printf("| [a] Press 'a' to[0, 3,1]!     | [t] press 't' to takeoff!       |\n");
    printf("| [s] Press 's' to[-1,3,1]!     |                                 |\n");
    printf("| [d] Press 'd' to[-1,4,1]!     |                                 |\n");
    printf("| [e] Landing                   |                                 |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input a/b/c etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
    printf("input: ");
}


int main(int argc, char **argv)
{
    int temp32;
    ros::init(argc, argv, "flight_logic");
    
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);

    vision_active = false;
    stm32_cmd_pub = nh.advertise<std_msgs::String>("/stm32_cmd", 1);
    target_pos_pub = nh.advertise<geometry_msgs::Vector3>("/target_position", 1);
    cmd_msg_pub = nh.advertise<std_msgs::UInt8>("/core_cmd", 10);


//    vision_active_sub = nh.subscribe("/vision_active",10,vison_active_callback);
    landing_sub = nh.subscribe("/landing_car",10,landCarCallback);
    car_state_sub = nh.subscribe("/car_state" , 10 , carStateCallback);
//    odometry_sub = nh.subscribe("/guidance/odometry", 10, guidance_odometry_callback);
    real_pos_sub = nh.subscribe("/drone_pos", 10, real_position_callback);
    sudoku_sub = nh.subscribe("sudoku",10,sudoku_callback);

    current_mission = m100_INIT;
    ros::Rate loop_rate(50);
    Display_Main_Menu();

    std_msgs::UInt8 cmd;

    while( ros::ok() )
    {
/*-------------------------- menu--------------------------*/
//        temp32 = getchar();
//        switch(temp32)
//        {
//        case 'I':
//            cmd.data = m100_INIT;
//            cmd_msg_pub.publish(cmd);
//            break;
//        case 't':
//            cmd.data = m100_TAKEOFF;
//            cmd_msg_pub.publish(cmd);
//            break;
//        case 'e':
//            /* landing*/
//            drone->landing();
//            break;
//        case 'w':
//            target_point(0.0,4.0,1);
//            ROS_INFO("Traget pos is [0, 4,1]!");
//            break;
//        case 'a':
//            target_point(0.0,3.0,1);
//            ROS_INFO("Traget pos is [0.0,3.0,1]!");
//            break;
//        case 's':
//            target_point(-1.0,3.0,1);
//            ROS_INFO("Traget pos is [-1.0,3.0,1]!");
//            break;
//        case 'd':
//            target_point(-1.0,4.0,1);
//            ROS_INFO("Traget pos is [-1.0,4.0,1]!");
//            break;
//        case 'r':
//            std_msgs::String cmd;
//            cmd.data = "Release";
//            stm32_cmd_pub.publish(cmd);
//            break;
//        }
/*-------------------------- menu--------------------------*/
        mission_run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
