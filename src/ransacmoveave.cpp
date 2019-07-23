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
float ave_laser_right[11][10]={};
int i ,j,k,cnt,i_s,j_s,i_b,j_b , i_cnt ,wall_cnt=0;
float a_b,a_s,b_b,b_s;
float x1_s,x2_s,y1_s,y2_s,x1_b,x2_b,y1_b,y2_b;
void right_wall(float []);
void sort(float,float,float,float,float,int,int,float,float);
void scanCallback(const sensor_msgs::LaserScan msg)
{
  float err=0,x1,x2,y1,y2,a=0,b=0,x,y,y_ori;
  cnt=1;
  for(i=1;i<683;i++){
   laser_msg[i] = msg.ranges[i];
  }
  
	right_wall(laser_msg);
  for(i=80;i<90;i++){
   for(j=i+1;j<=90;j++){
     x1=ave_laser_right[i-80][9]*cos(((i-80)-5)*0.3515*pi/180);
     y1=ave_laser_right[i-80][9]*sin(((i-80)-5)*0.3515*pi/180);
     x2=ave_laser_right[j-80][9]*cos(((j-80)-5)*0.3515*pi/180);
     y2=ave_laser_right[j-80][9]*sin(((j-80)-5)*0.3515*pi/180);
     a = (y2-y1)/(x2-x1);
     b = (y2*x1-y1*x2)/(x1-x2);
     for(k=80;k<=90;k++){
      x=ave_laser_right[k-80][9]*cos(((k-80)-5)*0.3515*pi/180);
      y_ori=ave_laser_right[k-80][9]*sin(((k-80)-5)*0.3515*pi/180);
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
  line(img, Point(laser_msg[1]*cos((-84)*0.3515*pi/180)*100+200,(a_s*laser_msg[1]*cos((-84)*0.3515*pi/180)+b_s)*100+200), Point(laser_msg[170]*cos(85*0.3515*pi/180)*100+200,(a_s*laser_msg[170]*cos(85*0.3515*pi/180)+b_s)*100+200), Scalar(255,0,0), 3);
    
  cv::imshow("window", img);
  cv::waitKey(2);
  for(i=0;i<11;i++){
		ave_laser_right[i][9]=0;
	}
  wall_cnt++;
}

void right_wall(float laser_msg[]){
	i_cnt=0;
	
	if(wall_cnt <9 && wall_cnt !=0){
		for(i=80;i<=90;i++){
			
			ave_laser_right[i-80][cnt] = laser_msg[i];
			
			i_cnt++;
		}
		for(i=0;i<11;i++){
			for(j=0;j<cnt;j++){
					ave_laser_right[i][9] += ave_laser_right[i][j]/cnt;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	else if(wall_cnt == 0) {
		for(i=80;i<=90;i++){
			
			ave_laser_right[i-80][cnt] = laser_msg[i];
			
			i_cnt++;
		}
		
		for(i=0;i<11;i++){
			ave_laser_right[i][9] = ave_laser_right[i][0];
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
		}
	}
	else{
		//cout << "laser91:" << laser_msg[91] << endl;
		for(i=0;i<11;i++){
			for(j=0;j<9;j++){
				ave_laser_right[i][j] = ave_laser_right[i][j+1];
			} 
		}
		for(i=80;i<=90;i++){
			ave_laser_right[i-80][8] = laser_msg[i];
			//cout << "laser" << i << ":" << laser_msg[i] << endl;
			
			i_cnt++;
		}
		for(i=0;i<11;i++){
			for(j=0;j<9;j++){
					ave_laser_right[i][9] += ave_laser_right[i][j]/9;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	
	
	
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

