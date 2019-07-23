#include "ros/ros.h"
#include <std_msgs/Int64MultiArray.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Int16MultiArray.h>
#include <iostream>
using namespace std;
float laser_msg[683]={};
void err_READ(const sensor_msgs::LaserScan msg){
	int i ;
	ros::NodeHandle n;
	int errpoint=0;
	ros::Publisher err_pub = n.advertise<std_msgs::Int16MultiArray>("err", 100);
	
	
	for(i=1;i<683;i++){
		laser_msg[i] = msg.ranges[i];
	}
	
	for(i=1;i<683;i++){
		if(laser_msg[i] <0.2){
			errpoint=1;
			std_msgs::Int16MultiArray msgerr;
   		msgerr.data.push_back(errpoint);
   		err_pub.publish(msgerr);   
			ROS_INFO("errpoint=1");
			cout << "Positon " << i << " too close" << endl;
			break;
		}
		else if(isnan(laser_msg[i])){
			cout << "nan "  << endl;
		}
		
	}
	if(errpoint !=1){
		errpoint = 0 ;
		std_msgs::Int16MultiArray msgerr;
   	msgerr.data.push_back(errpoint);
   	err_pub.publish(msgerr);   
		ROS_INFO("errpoint=0");
		cout << "wall far" << endl;
	}

}
int main(int argc, char **argv){
	ros::init(argc,argv,"test_break");
	ros::NodeHandle n;
	
	ros::Subscriber lidar_err_sub = n.subscribe("/scan", 100, err_READ);
	ros::Publisher err_pub = n.advertise<std_msgs::Int16MultiArray>("err", 100);
	ros::spin();
	return 0;
}
