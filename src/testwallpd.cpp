#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
using namespace std;
double lasterr = 0.0;
int cnt = 1;
//double integ=0;
void scanCallback(const sensor_msgs::LaserScan msg)
{
	ros::NodeHandle n;
	ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move", 1000);
	std_msgs::Int64MultiArray msgmove;
        float  lidarmsg267,lidarmsg268,lidarmsg269,lidarmsg270,lidarmsg271;
        float lidar_msg[5]={};
	int pose_pub[2];
	int i = 0;
	double lm = 0.0, rm = 0.0, err = 0.0, der = 0.0, lidarmsg = 0.0, turn = 0.0;

	for(i=266;i<=274;i++){
	 lidar_msg[i-266] = msg.ranges[i];
	 //lidar_msg[i-266] = lidar_msg[i-266]-0.3;
	}
        for(i=0;i<9;i++){
	 err +=lidar_msg[i];
	 
	}
	err=err/i-0.3;
	//err = (lidar_msg[4] - lidar_msg[0]) * 2 + (lidar_msg[3] - lidar_msg[1]) * 1 + lidar_msg[2];
        //integ+=err;
	der = err - lasterr ;
	turn = err * 700 + der * 250;
        //turn = err *700 ;
	rm = 180 - turn;
	lm = 180 + turn;
	cout << "err" << err << endl;
	cout << "lasterr" << lasterr << endl;
        if(rm > 255){
         rm = 255; 
        }else if(rm < -255){
	 rm = -255;
	}
        if(lm > 255){
         lm = 255; 
        }else if(lm < -255){
	 lm = -255;
	}
	cout << "rm" << int(rm) << endl;
	cout << "lm" << int(lm) << endl;
	pose_pub[0] = int(lm);
	pose_pub[1] = int(rm);
	msgmove.data.push_back(pose_pub[0]);
	msgmove.data.push_back(pose_pub[1]);
	move_pub.publish(msgmove);
	lasterr = err;

	

}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "testwallpd");


	ros::NodeHandle n;

	ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
	ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move", 1000);

	ros::spin();

	return 0;
}

