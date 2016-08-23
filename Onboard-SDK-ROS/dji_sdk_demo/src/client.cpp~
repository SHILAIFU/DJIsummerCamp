#include <dji_sdk/dji_drone.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <stdio.h>       /*标准输入输出定义*/
#include <cstdlib>       /*标准函数库定义*/
#include <std_msgs/String.h>

//chuankou touwenjian
#include <unistd.h>      /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>       /*文件控制定义*/
#include <termios.h>     /*POSIX 终端控制定义*/
#include <errno.h>       /*错误号定义*/
#include <string.h>      /*字符串功能函数*/
//#include "guidance/open_cv.h"         add later
//#define FIFO_SERVER "/tmp/myfifo"


using namespace DJI::onboardSDK;
using namespace std;
bool claw_flag;
int fd = 0; /*函数返回值：成功返回文件描述符，如果失败返回-1*/
int serials_test = 0;
unsigned char buftemp[60];
struct termios options_old, options;
void serials_init(int serials_fd);
void serials_release(int serials_fd);
int serials_data_read(int serials_fd,int *switch_data,uint16_t *_roll_data,
                         uint16_t *_pitch_data,uint16_t *_throttle_data,uint16_t *_yaw_data);

ros::Subscriber  stm32_cmd_sub;
ros::Publisher   car_state_pub;

void stm32_callback(const std_msgs::String& msg)
{
//    temp32 = msg.data;
    //temp32 = getchar();
    if(msg.data == "Release")
    {
          msg.data == "close";
          serials_test = write(fd, "re", 2);     //open
          sleep(3);
          serials_test = write(fd, "ge", 2);
          sleep(1);
        cout<<serials_test<< "open the claw"<<endl;
          serials_test = write(fd, "oe", 2);     //call car to leave

    }
    else if(msg.data == "close")
    {
        serials_test = write(fd, "ge", 2);              //close
        cout<< serials_test<<"close the claw"<<endl;
    }

    if(msg.data == "land_ok")
    {
        serials_test = write(fd, "le", 2);
        cout<< serials_test<<" land_ok"<<endl;
    }

}


//void Hover_callback1(const guidance::open_cv& msg)
//{
//    float Hover_P = 0.1;
//    float Hover_Vx ;
//    float Hover_Vy ;
//    if(msg.rev6 == 1)
//    {
//        Hover_Vx = msg.rev4/1000*Hover_P;
//        Hover_Vy = msg.rev5/1000*Hover_P;
//        cout<<claw_flag<<endl;
//        if(msg.rev4<=0.1&&msg.rev5<=0.1&&claw_flag==0)
//        {
//          serials_test = write(fd, "re", 2);
//          cout<< "close the claw:"<<"Hover_Vx="<<msg.rev4<<endl;
//          claw_flag = 1;
//          usleep(5000000);
//        }
//    }
//    else
//    {
//      Hover_Vx = 0;
//      Hover_Vy = 0;
//    }
//}

/*              UART1 -- /dev/ttyTHS0
 *              UART2 -- /dev/ttyTHS1
 *              UART3 -- /dev/ttyTHS2
 *              UART4 -- /dev/ttyS0       */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle n;
    ros::Subscriber stm32_cmd_sub = n.subscribe("/stm32_cmd", 5, stm32_callback);
    car_state_pub = n.advertise<std_msgs::String>("/car_state",10);   // car message for drone
    //ros::NodeHandle nh;
    //DJIDrone* drone = new DJIDrone(nh);
    int temp32;
    std_msgs::String car_cmd;
    int receive_test = -1;
    //char receive_data;
    unsigned char receive_data[1];
    //    int iRet = read(serials_fd, buf, 60);   //串口读操作（接收端）


    fd = open("/dev/ttyTHS2", O_RDWR | O_NOCTTY | O_NDELAY);        //打开串口3,作为妙算和飞机的通讯串口
    serials_init(fd);       //串口3初始化
    cout<<serials_test<< "please press J(open)/k(close)"<<endl;

    while(ros::ok())
    {
//        temp32 = getchar();
// //       temp32 = msg.data;
//        if(temp32  == 'j')
//        {
//              serials_test = write(fd, "re", 2);     //open
//              sleep(3);
//              serials_test = write(fd, "ge", 2);
//              sleep(1);
//            cout<<serials_test<< "close the claw"<<endl;
//        }
//        else if(temp32  == 'k')
//        {
//            serials_test = write(fd, "ge", 2);              //close
//            cout<< serials_test<<"open the claw"<<endl;
//        }
/*------------------- car  to  drone -----------------------------------------*/

//        car_cmd.data = "carStart";
//        car_state_pub.publish(car_cmd);

        receive_test = read(fd,receive_data, 1);
        if ( receive_test != -1)
        {

            if ('d' == receive_data[0])						// push the doll;
            {
                 car_cmd.data = "carStart";
                 int i = 0;
                 while(i < 10)
                 {
                 car_state_pub.publish(car_cmd);
                 i++;
                 }
                cout << receive_test << '\t' << receive_data[0] << endl;
                // start;
                receive_data[0] = 'n';
            }
            if ('f' == receive_data[0])						// car is ready;
            {
                car_cmd.data = "forLand";
                int j = 0;
                while(j<10)
                {
                car_state_pub.publish(car_cmd);
                j++;
                }
                cout << receive_test << '\t' << receive_data[0] << endl;
                // landing;
                receive_data[0] = 'n';
            }
            else if ('p' == receive_data[0])					// catch the toy;
            {
                car_cmd.data = "pushDone";
                int k = 0;
                while(k<10)
                {
                car_state_pub.publish(car_cmd);
                k++;
                }
                cout << receive_test << '\t' << receive_data[0] << endl;
                // takeoff;
                receive_data[0] = 'n';
            }
        }
/*------------------- car  to  drone -----------------------------------------*/


        ros::spinOnce();
    }
    serials_release(fd);
    return 0;
}
void serials_init(int serials_fd)
{
    //if(serials_fd < 0)
        //printf("[%s-%d] open error!!\n",__FILE__,__LINE__);     //串口打开失败
    tcgetattr(serials_fd,&options);     //获取终端控制属性,储存目前的序列埠设定
    options_old = options;
    cfsetispeed(&options, B115200);     //指定输入波特率，115200bps
    cfsetospeed(&options, B115200);     //指定输出波特率，115200bps

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(serials_fd, TCSANOW, &options);   //设置终端控制属性
}

void serials_release(int serials_fd)
{
    tcsetattr(serials_fd, TCSANOW, &options_old);
    close(serials_fd);
}

//// Manifold和stm32板通讯 读取32板数据
//int serials_data_read(int serials_fd,int *switch_data,uint16_t *_roll_data,
//                         uint16_t *_pitch_data,uint16_t *_throttle_data,uint16_t *_yaw_data)
//{
//    unsigned char buf[60];
//    int iRet = read(serials_fd, buf, 60);   //串口读操作（接收端）
//    int begin_char;
//    stringstream ss;
//    string buf_str;
//    if(iRet != -1)
//    {\
//        ss << buf;
//        ss >> buf_str;
//        begin_char = buf_str.find('#');
//        ss.clear();
//        ss << buf_str.substr(begin_char + 1,4);
//        ss >> *_roll_data;
//        ss.clear();
//        ss << buf_str.substr(begin_char + 6,4);
//        ss >> *_pitch_data;
//        ss.clear();
//        ss << buf_str.substr(begin_char + 11,4);
//        ss >> *_yaw_data;
//        ss.clear();
//        ss << buf_str.substr(begin_char + 16,4);
//        ss >> *_throttle_data;
//        ss.clear();
//        ss << buf_str.substr(begin_char + 21,4);
//        ss >> *switch_data;\
//        ss.clear();
//        tcflush(serials_fd, TCIFLUSH);
//        iRet = -1;
//        memset(buf,0,60);
//        return 0;
//    }
//    else
//    {
//        //cout << "can't read serials" << endl;
//        return 1;
//    }
//}

