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
  for(i=331;i<=349;i++){
   
   x_arr[i-331] = laser_msg[i]*cos(((i-331)-9)*0.3515*pi/180);
   y_arr[i-331] = laser_msg[i]*sin(((i-331)-9)*0.3515*pi/180);
   cout << "x[" << i-331 << "]:" << x_arr[i-331] << endl;
   cout << "y[" << i-331 << "]:" << y_arr[i-331] << endl;
   //cout << "sin1:" << sin(2*pi/180) <<endl;
   
  }
  
  
  for(i=331;i<=349;i++){
   x_11+=x_arr[i-331]*x_arr[i-331];
   x_12+=x_arr[i-331];
   x_21+=x_arr[i-331];
   
  }
  x_22 = 19;
  det = x_11*x_22-x_12*x_21;
  temp = x_22;
  x_22 = x_11;
  x_11 = temp;
  x_11 = x_11/det;
  x_22 = x_22/det;
  x_12=-x_12/det;
  x_21=-x_21/det;
  cout << "x_11:" << x_11 << endl << "x_12:" << x_12 <<endl << "x_21:" << x_21 <<endl << "x_22:" << x_22 <<endl ;
  
  for(i=331;i<=349;i++){
   a_arr[i-331]=x_11*laser_msg[i]*cos(((i-331)-9)*0.3515*pi/180)+x_12*1;
   b_arr[i-331]=x_21*laser_msg[i]*cos(((i-331)-9)*0.3515*pi/180)+x_22*1;
   //cout << "a[" << i-331 << "]:" << a_arr[i-331] << endl;
   //cout << "b[" << i-331 << "]:" << b_arr[i-331] << endl;
   
  }
  
  for(i=331;i<=349;i++){
   a+=a_arr[i-331]*y_arr[i-331];
   b+=b_arr[i-331]*y_arr[i-331];
   //cout <<"laser" << i << ":" << laser_msg[i] << endl;
   
  }

  
  
  a_tan = atan(a);
  if(atan(a) > 0){
   theta = 90-atan(a)/pi*180;
  }
  else{
   theta = 90+atan(a)/pi*180;
  }
  cout << "a:" << a << endl << "b:" << b << endl << "a_tan:" << a_tan << endl<< "theta:" << theta << endl;
  fi =150;
  /*
  x_1_deg =  laser_msg[300]*cos((theta+fi)*0.3515*pi/180); 
  y_1_deg = a*x_1_deg+b;
  
  cout << "1_deg:"<< laser_msg[300] << endl;

  dist = laser_msg[300]*sin((theta+fi)*0.3515*pi/180);
  err = y_1_deg-dist;
  cout << "a:" << a << endl << "b:" << b << endl << "a_tan:" << a_tan << endl;
  cout << "y_1_deg:"<< y_1_deg << endl << "dist:"<< dist << endl << "err:"<< err << endl;
  */
  //cout << "x331:" << laser_msg[331]*cos((-9)*0.3515*pi/180) << endl;
  //cout << "y331:" << (a*laser_msg[331]*cos((-9)*0.3515*pi/180)+b) << endl;
  //cout << "x349:" << laser_msg[349]*cos(9*0.3515*pi/180) << endl;
  //cout << "y349:" << (a*laser_msg[349]*cos(9*0.3515*pi/180)+b) << endl;

  Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
  line(img, Point(laser_msg[331]*cos((-9)*0.3515*pi/180)*100+200,(a*laser_msg[331]*cos((-9)*0.3515*pi/180)+b)*100+200), Point(laser_msg[349]*cos(9*0.3515*pi/180)*100+200,(a*laser_msg[349]*cos(9*0.3515*pi/180)+b)*100+200), Scalar(255,0,0), 3);
    
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

