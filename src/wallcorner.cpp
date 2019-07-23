#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#define  pi 3.14159265
#include <iostream>
using namespace std;
float laser_msg[61]={};
int i,j,cnt=1;
float theta,err,dist;
float fi;
int pose_pub[2];
float imu_init=0,imu_back,imu_diff;
void test(float[]);
void scanCallback(const sensor_msgs::LaserScan msg)
{
  
  
  for(i=0;i<=30;i++){
   
   
   laser_msg[i] = msg.ranges[300-i];
   
   
  }
  test(laser_msg);
 
}



void test(float laser_msg[]){
 for(i=0;i<30;i++){
  
   err = laser_msg[i]-laser_msg[i+1];
   if(err > 0.12){
    ROS_INFO("corner");
    theta = 30-i-1 ;
    dist=laser_msg[i+1]*sin(theta*pi/180);
    cout << "distance:" << dist <<endl;
    break;
   } 
  
 }
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "wallcorner");

  
  ros::NodeHandle n;
  
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  //ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, chatterCallback);
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
  
  //test(laser_msg);
  



  ros::spin();

  return 0;
}

