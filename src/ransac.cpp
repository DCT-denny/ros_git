#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#define  pi 3.14159265
#include <iostream>
using namespace std;
using namespace cv;
float laser_msg[683]={},err_big,err_small;
float ave_laser_right[10][10]={};
int i ,j,k,cnt,i_s,j_s,i_b,j_b;
float a_b,a_s,b_b,b_s;
float x1_s,x2_s,y1_s,y2_s,x1_b,x2_b,y1_b,y2_b;
void sort(float,float,float,float,float,int,int,float,float);
void scanCallback(const sensor_msgs::LaserScan msg)
{
  float err=0,x1,x2,y1,y2,a=0,b=0,x,y,y_ori;
  cnt=1;
  for(i=1;i<683;i++){
   laser_msg[i] = msg.ranges[i];
  }
  
	
  for(i=80;i<90;i++){
   for(j=i+1;j<=90;j++){
     x1=laser_msg[i]*cos(((i-80)-5)*0.3515*pi/180);
     y1=laser_msg[i]*sin(((i-80)-5)*0.3515*pi/180);
     x2=laser_msg[j]*cos(((j-80)-5)*0.3515*pi/180);
     y2=laser_msg[j]*sin(((j-80)-5)*0.3515*pi/180);
     a = (y2-y1)/(x2-x1);
     b = (y2*x1-y1*x2)/(x1-x2);
     for(k=80;k<=90;k++){
      x=laser_msg[k]*cos(((k-80)-5)*0.3515*pi/180);
      y_ori=laser_msg[k]*sin(((k-80)-5)*0.3515*pi/180);
      y =a*x+b;
      err += abs(y_ori-y);
      cout << "x:" << x << endl << "y:" << y_ori << endl;
     }
    cout << "err:" << err << endl;
    //cout << "ev_a:" << a << endl; 
    sort(err,x1,x2,y1,y2,i,j,a,b);
    err=0;
   }
  }
  cout << "a:" << a_s << "  " << "b:" << b_s << endl ;
  cout << "x1:" << x1_s << "  " << "y1:" << y1_s << endl;
  cout << "x2:" << x2_s << "  " << "y2:" << y2_s << endl;
  cout << "i:" << i_s << "  " << "j:" << j_s << endl;
  //cout << "x:" << x << endl << "y:" << y << endl;  
  
  

  Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
  line(img, Point(laser_msg[80]*cos((-5)*0.3515*pi/180)*100+200,(a_s*laser_msg[80]*cos((-5)*0.3515*pi/180)+b_s)*100+200), Point(laser_msg[90]*cos(5*0.3515*pi/180)*100+200,(a_s*laser_msg[90]*cos(5*0.3515*pi/180)+b_s)*100+200), Scalar(255,0,0), 3);
    
  cv::imshow("window", img);
  cv::waitKey(2);
}


void sort(float err,float x1,float x2,float y1,float y2,int i,int j,float a,float b){
 float t;
 
 if(cnt == 1){
  err_small = err ;
  x1_s = x1;
  x2_s = x2;
  y1_s = y1;
  y2_s = y2;
  i_s = i ;
  j_s = j ;
  a_s = a ;
  b_s = b ;
 }
 err_big = err;
 x1_b = x1;
 x2_b = x2;
 y1_b = y1;
 y2_b = y2;
 i_b = i;
 j_b = j;
 a_b = a ;
 b_b = b ;
 if(err_small > err_big){
  t = err_small;
  err_small = err_big;
  err_big = t ;
  t = x1_s ;
  x1_s = x1_b;
  x1_b = t ;
  t = x2_s ;
  x2_s = x2_b;
  x2_b = t ;
  t = y1_s ;
  y1_s = y1_b;
  y1_b = t ;
  t = y2_s ;
  y2_s = y2_b;
  y2_b = t ;
  t = i_s ;
  i_s = i_b;
  i_b = t ;
  t = j_s ;
  j_s = j_b;
  j_b = t ;
  t = a_s ;
  a_s = a_b;
  a_b = t ;
  t = b_s ;
  b_s = b_b;
  b_b = t ;
 }
 cnt++;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "ransac");

  
  ros::NodeHandle n;
  
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
  
 
  



  ros::spin();

  return 0;
}

