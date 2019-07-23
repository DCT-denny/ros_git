#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <iostream>
using namespace std;
double lasterr=0.0;
int cnt=0;
float imu_init=0,imu_back,imu_diff;
float laser_msg0;
std_msgs::Int64MultiArray msgmove;
int pose_pub[2];
int i=0;
double lm=0.0,rm=0.0,err=0.0,der=0.0,lidarmsg=0.0,turn=0.0; 
int laser_point = 1;
void test(float,float);
void scanCallback(const sensor_msgs::LaserScan msg)
{

  laser_msg0 = msg.ranges[0];

  
 
}
void chatterCallback(const sensor_msgs::Imu imu_msg)
{
  ros::NodeHandle n;
  ros::ServiceClient client=n.serviceClient<std_srvs::Empty>("/imu/set_zero_orientation");
  
  if(cnt ==1){
   //imu_init=tf::getYaw(imu_msg.orientation)*180/3.1415926;
   std_srvs::Empty srv;
   client.call(srv);
  }
  //cout << imu_msg.orientation.z << endl;
  cout << "imu_init_angle:" << imu_init << endl;
  
  

  imu_back=tf::getYaw(imu_msg.orientation)*180/3.1415926;
  cout << "imu_angle:" << imu_back << endl;
  imu_diff = imu_back - imu_init ;
  cout << "imu_diff_ori:" << imu_diff << endl;
  cnt=0;
  test(laser_msg0,imu_diff);
  
}
void test(float laser_msg0,float imu_diff){
 ros::NodeHandle n;
 ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
 ros::Rate loop_rate(10);
 cout << "laser0:" << laser_msg0 << endl;
 if (laser_msg0 < 0.3 && laser_msg0!=0 || laser_point==0) {
		

		if (laser_point == 1) {
			cnt = 1;
		}

		laser_point = 0;

		if (abs(imu_diff) < 90) {
			pose_pub[0] = -130;
			pose_pub[1] = 130;

			msgmove.data.push_back(pose_pub[0]);
			msgmove.data.push_back(pose_pub[1]);
			move_pub.publish(msgmove);
			msgmove.data.clear();
			//debug
			
			//debug
			
		}
		else {
			laser_point = 1;
                        cout << "?" <<endl;
		}
  }
  else{
  
    pose_pub[0] = 130;
    pose_pub[1] = 130;
    

    msgmove.data.push_back(pose_pub[0]);
    msgmove.data.push_back(pose_pub[1]);
    move_pub.publish(msgmove);
    msgmove.data.clear();
  }
  //ros::spinOnce();
  //loop_rate.sleep();
}
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "wallandturn");

  
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, chatterCallback);
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
  //test(laser_msg0,imu_diff);
  ros::ServiceClient clinet=n.serviceClient<std_srvs::Empty>("/imu/set_zero_orientation");
  
  



  ros::spin();

  return 0;
}


