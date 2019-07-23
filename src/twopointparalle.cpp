#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <iostream>
using namespace std;
double lasterr = 0.0;
int cnt = 1;
std_msgs::Int64MultiArray msgmove;
float  lidarmsg2,lidarmsg358,lidarmsg0,lidarmsg5,lidarmsg355;
int pose_pub[2];
int i = 0;
double lm = 0.0, rm = 0.0,err=0.0,err2=0;
//double integ=0;
void scanCallback(const sensor_msgs::LaserScan msg)
{
	ros::NodeHandle n;
	ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move", 1000);
	
	lidarmsg0 = msg.ranges[0];
	lidarmsg2 = msg.ranges[2];
	lidarmsg358 = msg.ranges[358];

	lidarmsg5 = msg.ranges[5];
	lidarmsg355 = msg.ranges[355];
		

	//err = abs(lidarmsg2-lidarmsg358);
        err = lidarmsg5*cos(5*M_PI/180)-0.3;
        err2 = lidarmsg355*cos(5*M_PI/180)-0.3;
        
	if(err > 0.02 && err2 >0.02){
	 rm = 130;
	 lm = 130;
	}
       else if(err < -0.02 && err2 >0.02){
	rm = 130;
	lm = -130;
       }else if(err > 0.02 && err2 <-0.02){
	rm = -130;
	lm = 130;
       }else if(err < -0.02 && err2 <-0.02){
	rm = -130;
	lm = -130;
       }


	cout << "err" << err << endl;
	cout << "err2" << err2 << endl;
        
	cout << "rm" << int(rm) << endl;
	cout << "lm" << int(lm) << endl;
	pose_pub[0] = int(lm);
	pose_pub[1] = int(rm);
	msgmove.data.push_back(pose_pub[0]);
	msgmove.data.push_back(pose_pub[1]);
	move_pub.publish(msgmove);
	//msgmove.data.clear();
	
	

}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "testwallpd");


	ros::NodeHandle n;

	ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
	ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move", 1000);
	ros::Rate loop_rate(2);
	/*
	while(ros::ok()){
	 if(err > 0.02 && err2 >0.02){
	 rm = 130;
	 lm = 130;
	}
       else if(err < -0.02 && err2 >0.02){
	rm = 130;
	lm = -130;
       }else if(err > 0.02 && err2 <-0.02){
	rm = -130;
	lm = 130;
       }else if(err < -0.02 && err2 <-0.02){
	rm = -130;
	lm = -130;
       }


	cout << "err" << err << endl;
	cout << "err2" << err2 << endl;
        
	cout << "rm" << int(rm) << endl;
	cout << "lm" << int(lm) << endl;
	pose_pub[0] = int(lm);
	pose_pub[1] = int(rm);
	msgmove.data.push_back(pose_pub[0]);
	msgmove.data.push_back(pose_pub[1]);
	move_pub.publish(msgmove);
	msgmove.data.clear();
	ros::spinOnce();
	loop_rate.sleep();
	}
	*/
	ros::spin();

	return 0;
}

