#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#define  pi 3.14159265
#include <iostream>
using namespace std;
using namespace cv;
float laser_msg[683]={},err_big,err_small;
float ave_laser_right[21][10]={},ave_laser_forward[21][10]={},ave_laser_left[21][10]={};
int cnt=0, i_cnt ,wall_cnt=0;
int x1_right,x2_right,y1_right,y2_right,x1_forward,x2_forward,y1_forward,y2_forward;
int x1_left,x2_left,y1_left,y2_left,x1_f,x2_f,y1_f,y2_f;
float x[44]={},y[44]={},r_save,_r[17]={};
int _x1[17]={},_x2[17]={},_y1[17]={},_y2[17]={},change=1;
void pointCallback(const std_msgs::Int16MultiArray msg)
{
 
  int i ;
  
  Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
  for(i=0;i<16;i++){
 		_x1[i] = msg.data[4*i];
 		_y1[i] = msg.data[4*i+1];
 		_x2[i] = msg.data[4*i+2];
 		_y2[i] = msg.data[4*i+3];
		cout << "_x1[" << i << "]:" << _x1[i] << endl;
		cout << "_x2[" << i << "]:" << _x2[i] << endl;
		cout << "_y1[" << i << "]:" << _y1[i] << endl;
		cout << "_y2[" << i << "]:" << _y2[i] << endl;
  }
  for(i=0;i<16;i++){
  	if(abs(_x2[i] - _x1[i]) < 20 && abs(_y2[i] - _y1[i]) < 20){
  	cout << "x_diff:" << _x2[i] - _x1[i] << endl;
  		line(img, Point(_x1[i]+200,-_y1[i]+200), Point(_x2[i]+200,-_y2[i]+200), Scalar(255,0+i*10,0), 3);
  	}
  }
  line(img, Point(200,200), Point(210,200), Scalar(0,255,0), 3);
  line(img, Point(200,200), Point(200,190), Scalar(0,0,255), 3);
  cv::imshow("window", img);
  cv::waitKey(2);
  
  
	
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "remote_paintwall");

  
  ros::NodeHandle n;
  
  ros::Subscriber point_sub = n.subscribe("/linesss", 1000, pointCallback);
  //ros::Subscriber point_sub = n.subscribe("/point", 1000, pointCallback);
  
  
  



  ros::spin();

  return 0;
}


