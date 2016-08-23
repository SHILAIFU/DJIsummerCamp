#include <ros/ros.h>
#include "aruco/levmarq.h"
#include <aruco/aruco.h>
#include <aruco/posetracker.h>
#include "aruco/ippe.h"

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
using namespace aruco;


ros::Publisher pubBoxPos;
ros::Publisher pubBoxPosImg;

vector<Point> modeConstruct(Mat img);
void boxDetect(Mat src, Mat &dst, vector<Point> boxModeContour, Mat &tVec, Mat &rVec);
void solvePNP(vector<Point> &pointsIn, Mat &tVec, Mat &rVec);
template<typename T>
double __aruco_solve_pnp(const std::vector<cv::Point3f> & p3d,const std::vector<cv::Point2f> & p2d,const cv::Mat &cam_matrix,const cv::Mat &dist,cv::Mat &r_io,cv::Mat &t_io);

Mat boxMode  = imread("/home/yf/01.png");
Mat boxMode2 = imread("/home/yf/02.png");
//Mat boxMode3 = imread("/home/lu/catkin_ws2/src/Guidance-SDK-ROS/led_picture/3.png");

vector<Point> boxModeContour  = modeConstruct(boxMode);
vector<Point> boxModeContour2 = modeConstruct(boxMode2);
//vector<Point> boxModeContour3 = modeConstruct(boxMode3);

vector<Point> modeConstruct(Mat img)
{
  Mat imgGray;
  cvtColor(img, imgGray, CV_BGR2GRAY);
  GaussianBlur(imgGray, imgGray, Size(3,3), 1);
  Mat element = getStructuringElement(0, Size(3,3));
  morphologyEx(imgGray, imgGray, CV_MOP_OPEN,element,Point(-1,-1),2);
  Mat imgCanny;
  Canny(imgGray, imgCanny, 30, 90);

  vector<vector<Point>> contours;
  vector<Point>         contourDP;
  contourDP.resize(contours.size());
  findContours(imgCanny, contours, RETR_TREE,CHAIN_APPROX_NONE);
  for(int i=0;i<contours.size();i++)
  {
    float contArea = contourArea(contours[i]);
    approxPolyDP(contours[i], contourDP, double(contours[i].size()) * 0.05, true);
    if(contArea > 3000 && isContourConvex(Mat(contourDP)) && contourDP.size() == 4)
    {
      drawContours(img, contours, i, Scalar(0,0,255),2);
      return contours[i];
    }
  }
}

/**
 * @brief boxDetect
 * @param src
 * @param dst
 * @param boxModeContour
 * @param tVec
 * @param rVec
 * @return
 */
void boxDetect(Mat src, Mat &dst, vector<Point> boxModeContour, Mat &tVec, Mat &rVec)
{
  if(dst.empty())
    dst = src.clone();
  Mat imgGray;
  cvtColor(src, imgGray, CV_BGR2GRAY);
  GaussianBlur(imgGray, imgGray, Size(5,5), 0);
  Mat element = getStructuringElement(0, Size(3,3));
  morphologyEx(imgGray, imgGray, CV_MOP_OPEN,element,Point(-1,-1),1);
  Mat imgCanny;

  threshold(imgGray,imgCanny,0,255,CV_THRESH_BINARY + CV_THRESH_OTSU);

  Canny(imgGray, imgCanny, 30, 90);

  vector<vector<Point>> contours;
  vector<Point>         contourDP;
  contourDP.resize(contours.size());
  findContours(imgCanny, contours, RETR_TREE,CHAIN_APPROX_NONE);

  for(int i=0;i<contours.size();i++)
  {
    float contArea = contourArea(contours[i]);
    approxPolyDP(contours[i], contourDP, double(contours[i].size()) * 0.05, true);
    if(contArea > 3000 && isContourConvex(Mat(contourDP)) && contourDP.size() == 4)
    {
      double similarityParam = matchShapes(boxModeContour, contours[i],CV_CONTOURS_MATCH_I1,0);
      //if(similarityParam < 0.01)
      if(similarityParam<0.01)
      {
        drawContours(dst, contours, i, Scalar(0,0,255),2);
        /*Order the points!*/
        vector <Point> pointsIn;
        int min_x = INT_MAX;
        int min_index = 0;
        for(int j=0;j<4;j++)
        {
          if(sqrt(contourDP[j].x*contourDP[j].x+contourDP[j].y*contourDP[j].y) < min_x)
          {
            min_x = sqrt(contourDP[j].x*contourDP[j].x+contourDP[j].y*contourDP[j].y);
            min_index = j;
          }
        }

        if(min_index == 0 )
        {
          for(int j=0;j<4 ;j++)
          {
            pointsIn.push_back(contourDP[j]);
            //circle(img_show,Point(pointsIn[j].x,pointsIn[j].y),8,Scalar(0,0,255));
          }
        }
        else
        {
          vector<int> tmp={1,2,3,0,1,2,3};
          vector<int>::iterator tmp_itr = tmp.begin();
          int k=0;
          tmp_itr = find(tmp.begin(),tmp.end(),min_index);


          for(;min_index!=0 && tmp_itr!=tmp.end() && k!=4; tmp_itr++){
            pointsIn[k++] = contourDP[*tmp_itr];
            //circle(img_show,Point(pointsIn[k-1].x,pointsIn[k-1].y),8,Scalar(0,0,255));
          }
        }
        solvePNP(pointsIn, tVec, rVec);
      }
    }
  }
}

void solvePNP(vector<Point> &pointsIn, Mat &tVec, Mat &rVec)
{
  vector<Point3f> objectPoints;//Size of the Marker
  objectPoints.push_back(Point3f(0,0,0));

  vector<Point2f> imagePoints;
  for(int i=0;i<4;i++)
  {
    imagePoints.push_back(Point2f(pointsIn[i].x,pointsIn[i].y));
  }

  float tmp[3][3] = {{431.7,0,319.5}
                     ,{0, 431.7, 239.5}
                     ,{0,0,1}};
  Mat   cameMatrix(3,3,CV_32FC1, tmp);
  float tmp2[5] = {-0.37704,-0.20618,0,0,-0.074069};
  Mat   distCoeffs(1,5,CV_32F,tmp2);
  Mat   rotat_vecr1(1,3,CV_32F), trans_vecr1(1,3,CV_32F);

  //solvePnP(objectPoints,imagePoints,cameMatrix,distCoeffs,rotat_vecr1,trans_vecr1);
  __aruco_solve_pnp<float>(objectPoints,imagePoints,cameMatrix,distCoeffs,rotat_vecr1,trans_vecr1);

  tVec = trans_vecr1.clone();
  rVec = rotat_vecr1.clone();
}

template<typename T>
double __aruco_solve_pnp(const std::vector<cv::Point3f> & p3d,const std::vector<cv::Point2f> & p2d,const cv::Mat &cam_matrix,const cv::Mat &dist,cv::Mat &r_io,cv::Mat &t_io)
{

  assert(r_io.type()==CV_32F);
  assert(t_io.type()==CV_32F);
  assert(t_io.total()==r_io.total());
  assert(t_io.total()==3);
  auto toSol=[](const cv::Mat &r,const cv::Mat &t)
  {
    typename LevMarq<T>::eVector sol(6);
    for(int i=0;i<3;i++){
      sol(i)=r.ptr<float>(0)[i];
      sol(i+3)=t.ptr<float>(0)[i];
    }
    return sol;
  };
  auto fromSol=[](const typename LevMarq<T>::eVector &sol,cv::Mat &r,cv::Mat &t){
    r.create(1,3,CV_32F);
    t.create(1,3,CV_32F);
    for(int i=0;i<3;i++){
      r.ptr<float>(0)[i]=sol(i);
      t.ptr<float>(0)[i]=sol(i+3);
    }
  };

  cv::Mat Jacb;
  auto err_f= [&](const  typename LevMarq<T>::eVector &sol,typename LevMarq<T>::eVector &err){
    std::vector<cv::Point2f> p2d_rej;
    cv::Mat r,t;
    fromSol(sol,r,t);
    cv::projectPoints(p3d,r,t,cam_matrix,dist,p2d_rej,Jacb);
    err.resize(p3d.size()*2);
    int err_idx=0;
    for(size_t i=0;i<p3d.size();i++){
      err(err_idx++)=p2d_rej[i].x-p2d[i].x;
      err(err_idx++)=p2d_rej[i].y-p2d[i].y;
    }
  };
  auto jac_f=[&](const typename LevMarq<T>::eVector &sol,Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> &J){
    (void)(sol);
    J.resize(p3d.size()*2,6);
    for(size_t i=0;i<p3d.size()*2;i++){
      double *jacb=Jacb.ptr<double>(i);
      for(int j=0;j<6;j++) J(i,j)=jacb[j];
    }
  };

  LevMarq<T> solver;
  solver.setParams(100,0.01,0.01);
  typename LevMarq<T>::eVector sol=toSol(r_io,t_io);
  auto err=solver.solve(sol,err_f,jac_f);

  fromSol(sol,r_io,t_io);
  return err;

}

void boxDetectCallback(const cv_bridge::CvImage::ConstPtr &msg)
{
  Mat img = msg -> image, tVec1, rVec1, tVec2, rVec2, imgShow;
  if(!img.empty())
  {
    boxDetect(img, imgShow, boxModeContour,  tVec1, rVec1);
    boxDetect(img, imgShow, boxModeContour2, tVec2, rVec2);

    if(!tVec1.empty() && !tVec2.empty() &&
       !rVec1.empty() && !rVec2.empty())
    {
      // Get the average value to count
      Mat transVec = (tVec1 + tVec2) / 2;
      Mat rotatVec = (tVec1 + tVec2) / 2;

      Mat R;
      Rodrigues(rotatVec, R);
      R = R.t();
      // Changed by dreamtale, minus 90 breause of the cordinate
      float ledBoxDiffYaw = (atan2(-R.at<float>(1,0),R.at<float>(0,0))) * 180.0 /3.141592 - 90.0;

      cout << "********************************" << endl;
      cout << "  yawAngle is :" << ledBoxDiffYaw << endl;
      cout << "  x is:        " << transVec.at<float>(0,0) << endl
           << "  y is:        " << transVec.at<float>(0,1) << endl
           << "  z is:        " << transVec.at<float>(0,2) << endl;

      //    Draw the ""Cross line" at the image center
      int lineLenth = 5;
      line(imgShow,
           Point(imgShow.cols / 2, imgShow.rows / 2 - lineLenth),
           Point(imgShow.cols / 2, imgShow.rows / 2 + lineLenth),
           Scalar(0, 0, 255));
      line(imgShow,
           Point(imgShow.cols / 2 - lineLenth, imgShow.rows / 2),
           Point(imgShow.cols / 2 + lineLenth, imgShow.rows / 2),
           Scalar(0, 0, 255), 2);

      imshow("led box detect", imgShow);

      // Publish the key data
      /*
       +-------+-------+-------+---------+
       | dif_x | dif_y | dif_z | dif_yaw |
       +-------+-------+-------+---------+
      */ // TODO:
      // Unit is:
      uav_vision::DetectInfo boxInfo;
      std_msgs::Float32MultiArray boxData;
      boxData.data.push_back(transVec.at<double>(0,0));
      boxData.data.push_back(transVec.at<double>(0,1));
      boxData.data.push_back(transVec.at<double>(0,2));
      boxData.data.push_back(ledBoxDiffYaw);

      boxInfo.header.stamp = ros::Time::now();
      boxInfo.data = boxData;
      pubBoxPos.publish(boxInfo);
      waitKey(1);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_led_box_detect");
  ros::NodeHandle nh;
  ros::Subscriber subImg;

  cout << "The led box detection node is setup!" << endl;
  pubBoxPos = nh.advertise<uav_vision::DetectInfo>("/uv_vision/detectLedBox", 50);
  subImg    = nh.subscribe<cv_bridge::CvImage>("/uav_cam/image", 5, boxDetectCallback);

  ros::spin();
  return 0;
}
