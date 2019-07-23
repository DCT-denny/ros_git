#include <cstdio>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cv_bridge/cv_bridge.h>
using namespace cv;

#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#define  pi 3.1415926
#include <iostream>
using namespace std;
using namespace cv;
float laser_msg[683]={};
int i,j,cnt=0,i_cnt=0;
float theta,err,err_back=0,dist,dist_arr[5];
float fi;
int pose_pub[2];
float imu_init=0,imu_back,imu_diff;
float a_tan ;
void test(float[],float,float,float);
void sort(float[]);
void scanCallback(const sensor_msgs::LaserScan msg)
{
  float x_11=0,x_12=0,x_21=0,x_22=0,temp,x_arr[19],y_arr[19],err,x_1_deg,y_1_deg;
  float det=0,a_arr[19],b_arr[19],a=0,b=0;
  
  for(i=1;i<683;i++){
   laser_msg[i] = msg.ranges[i];
  }

  
  cout << "x331:" << laser_msg[331]*cos((-9)*0.3515*pi/180) << endl;
  cout << "y331:" << laser_msg[331]*sin((-9)*0.3515*pi/180) << endl;
  cout << "x349:" << laser_msg[349]*cos(9*0.3515*pi/180) << endl;
  cout << "y349:" << laser_msg[349]*sin(9*0.3515*pi/180) << endl;

  a = (laser_msg[349]*sin(9*0.3515*pi/180)-laser_msg[331]*sin((-9)*0.3515*pi/180))/(laser_msg[349]*cos(9*0.3515*pi/180)-laser_msg[331]*cos((-9)*0.3515*pi/180));

  cout << "a:" << a << endl;
  Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
  line(img, Point(laser_msg[331]*cos((-9)*0.3515*pi/180)*1000+200,laser_msg[331]*sin((-9)*0.3515*pi/180)*1000+200), Point(laser_msg[349]*cos(9*0.3515*pi/180)*1000+200,laser_msg[349]*sin(9*0.3515*pi/180)*1000+200), Scalar(255,0,0), 3);
  
  
    
  cv::imshow("window", img);
  cv::waitKey(2);
  
}





int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "wallcorner2");

  
  ros::NodeHandle n;
  
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  //ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, chatterCallback);
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
  
  //test(laser_msg);
  



  ros::spin();

  return 0;
}



