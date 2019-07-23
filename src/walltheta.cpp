#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#define  pi 3.14159265
#include <iostream>
using namespace std;
float laser_msg[360]={};
void scanCallback(const sensor_msgs::LaserScan msg)
{
  float a=0,theta;
  for(int i=0;i<360;i++){
   laser_msg[i] = msg.ranges[i];
  }
 a = laser_msg[275]*sin(5*pi/180)/(laser_msg[275]*cos(5*pi/180)-laser_msg[270]);
 cout << "270x:" << laser_msg[270] <<endl;
 cout <<"275x:" << laser_msg[275]*cos(5*pi/180) << endl << "275y:" <<laser_msg[275]*sin(5*pi/180) <<endl;
 theta = 90-atan(a)*180/pi;
 cout << "a:" << a <<endl << "theta:" << theta << endl;
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "walltheta");

  
  ros::NodeHandle n;
  
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  //ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, chatterCallback);
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
  
  //test(laser_msg);
  



  ros::spin();

  return 0;
}

