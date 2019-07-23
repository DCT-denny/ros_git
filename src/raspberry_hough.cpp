#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#define  pi 3.14159265
#include <iostream>
#include <sstream>
using namespace std;
using namespace cv;
float laser_msg[683]={},err_big,err_small;
int cnt=0, i_cnt ;
int x1_f,x2_f,y1_f,y2_f;
float x[44]={},y[44]={},r_save,_r[17]={};
int _x1[17]={},_x2[17]={},_y1[17]={},_y2[17]={},change=1;
void hough_transform(float ave_laser[],int&,int&,int&,int&,float &);
void scanCallback(const sensor_msgs::LaserScan msg)
{
  int i ;
  ros::NodeHandle n;
  ros::Publisher point_pub = n.advertise<std_msgs::String>("/point",1000);
  
  for(i=1;i<683;i++){
   laser_msg[i] = msg.ranges[i];
  }
	
	for(i=1;i<=16;i++){
		hough_transform(laser_msg,x1_f,x2_f,y1_f,y2_f,r_save);
		_x1[i]=x1_f;
		_x2[i]=x2_f;
		_y1[i]=y1_f;
		_y2[i]=y2_f;
		_r[i] = r_save;
		cout << "_x1[" << i << "]:" << _x1[i] << "  " << "_y1[" << i << "]:" << _y1[i] << endl;
		cout << "_x2[" << i << "]:" << _x2[i] << "  " << "_y2[" << i << "]:" << _y2[i] << endl;
		cout << "_r[" << i << "]:" << _r[i] << endl;
		cnt++;
  }
  std_msgs::String msgpoint;
  std::stringstream ss;
  for(i=1;i<=16;i++){
  	ss << _x1[i] ;
  	
  	ss << ",";
  	
		
  	ss << _x2[i] ;
  	
  	ss << ",";
  	
		
  	ss << _y1[i] ;
  	
  	ss << ",";
  	
		
  	ss << _y2[i] ;
  	
  	ss << ",";
  	
		
  }
  msgpoint.data = ss.str();
  /*
  for(i=1;i<=16;i++){
  	msgpoint.data.push_back(_x1[i]);
  	msgpoint.data.push_back(_y1[i]);
  	msgpoint.data.push_back(_x2[i]);
  	msgpoint.data.push_back(_y2[i]);
  }
  */
  point_pub.publish(msgpoint);
  
  
  change=1;
	cnt=0;
	
}


void hough_transform(float ave_laser[],int &x1,int &x2,int &y1,int &y2,float &r_save){
	float acc_arr[41][41]={},r,err[41],err_min=100,final_r,final_angle;
	int r_angle[2],err_num;
	float max_x=0,min_x=100;
	float acc_num[41];
	int ticket_max=0;
	float a=0,b=0,x0,y0;
	int i ,j,k,out=0;
	x1=0,x2=0,y1=0,y2=0;
	//float x1,x2,y1,y2;
	
	for(i=1+cnt*42;i<=43+cnt*42;i++){
		if(isnan(ave_laser[i])){
			cout << "out nan" << endl;
			out = 1;
			break;
		}
	}
	
	if(out == 0){
	for(i=1+cnt*42;i<=43+cnt*42;i++){
		x[i-cnt*42] = ave_laser[i]*cos((i-85)*0.3515*pi/180);
		y[i-cnt*42] = ave_laser[i]*sin((i-85)*0.3515*pi/180);
		if(max_x < ave_laser[i]){
			max_x = ave_laser[i];
		}
		
		//cout << "ave_laser[" << i << "]:" << ave_laser[i] << endl;
		//cout << "x" << i << ":" << x[i] << endl;
	}
	cout << "max_x:" << max_x << endl;
	for(i=0;i<41;i++){
		acc_num[i] = -max_x+(max_x*2/40)*i;
		//acc_num[i] = min_x + (max_x-min_x)/40*i;
		//acc_num[i] = (max_x*2/20)*i;
		//cout << "acc_num" << i << ":" << acc_num[i] << endl;
	}
	
	for(i=1;i<=43;i++){
		for(j=0;j<41;j++){
			r = x[i]*cos(j*(pi/40))+y[i]*sin(j*(pi/40));
			//if(r > 0){
			for(k=0;k<41;k++){
				err[k] = abs(acc_num[k]- r) ;
				if(err_min > err[k]){
					err_min = err[k];
					err_num = k;
					//cout << "acc_num" << k << ":" << acc_num[k] << endl;
				}
				//cout << "err" << k << ":" << err[k] << endl;
			}
			//}
			err_min=100;
			acc_arr[err_num][j]++;
			//cout << "r" << i << ":" << r << endl;
			
		}
	}
	
	for(i=0;i<41;i++){
		for(j=0;j<41;j++){
			if(ticket_max <acc_arr[j][i]){
				ticket_max=acc_arr[j][i];
				r_angle[0]=j;
				r_angle[1]=i;
			}
			//cout << "acc_arr" << i << j << ":" << acc_arr[i][j] << endl;
		}
	}
	final_r = acc_num[r_angle[0]];
	final_angle = r_angle[1]*(pi/40);
	cout << "final_r:" << final_r << endl << "final_angle:" << final_angle << endl;
	
	r_save = final_r;
	
	a =  cos(final_angle);
	b =  sin(final_angle);
	x0 = a*final_r;
	y0 = b*final_r;
	
	x1 = int(x0*100+x[1]*100);
	y1 = int(y0*100+y[1]*100);
	
	x2 = int(x0*100+x[43]*100);
	y2 = int(y0*100+y[43]*100);
	
	
	
	cout << "x[1]:" << x[1] << "  " << "y[1]:" << y[1] << endl;
	cout << "x[43]:" << x[43] << "  " << "y[43]:" << y[43] << endl;
	
	}
	else if(out == 1){
		cout << "out nan break" << endl;
		
	}

	
	//cout << "x1_right2:" << x1_right << " " << "y1_right2:" << y1_right << endl;
  //cout << "x2_right2:" << x2_right << " " << "y2_right2:" << y2_right << endl;
	
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "raspberry_hough");

  
  ros::NodeHandle n;
  
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  ros::Publisher point_pub = n.advertise<std_msgs::String>("/point",1000);
  
  



  ros::spin();

  return 0;
}


