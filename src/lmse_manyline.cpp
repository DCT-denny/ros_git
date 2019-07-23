#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#define  pi 3.1415926
#define  each_angle 0.3515625
#include <iostream>
using namespace std;
using namespace cv;
float laser_msg[683]={};
float _a[17]={},_b[17]={};
float ave_laser[44][10]={};
int cnt=0,line_cnt=0,i_cnt=0,ave_cnt=0;
float a_final,b_final;
void moveave(float []);
void calc(float ave_laser[],float &,float &);
void scanCallback(const sensor_msgs::LaserScan msg)
{
	
	float x1[17],x2[17],y1[17],y2[17];
	int i,j ;
	int change=1;
  for(i=1;i<683;i++){
  	laser_msg[i] = msg.ranges[i];
  }
	
	
	for(j=1;j<=16;j++){
		//moveave(laser_msg);
		calc(laser_msg,a_final,b_final);
		_a[j]=a_final;
		_b[j]=b_final;
		cnt++;
		cout << "_a[" << j << "]:" << _a[j] << "  " << "_b[" << j << "]:" << _b[j] << endl;
		for(i=1;i<=43;i++){
			ave_laser[i][9]=0;
		}
	}
	
	//cout << "a_final:" << a_final << endl << "b_final:" << b_final << endl;
	Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
	for(i=1;i<=16;i++){
		x1[i]=laser_msg[1+line_cnt*42]*cos(0*each_angle*pi/180);
		y1[i]=_a[i]*laser_msg[1+line_cnt*42]*cos(0*each_angle*pi/180)+_b[i];
		
		x2[i]=laser_msg[43+line_cnt*42]*cos(43*each_angle*pi/180);
		y2[i]=_a[i]*laser_msg[43+line_cnt*42]*cos(43*each_angle*pi/180)+_b[i];
		
		cout << "x1[" << i << "]:" << x1[i] << "  " << "y1[" << i << "]:" << y1[i] << endl;
		cout << "x2[" << i << "]:" << x2[i] << "  " << "y2[" << i << "]:" << y2[i] << endl;
		
		//line(img, Point(x1[i]*100+200,y1[i]*100+200), Point(x2[i]*100+200,y2[i]*100+200), Scalar(255,0,0), 3);
		
		if(change ==1){
			//if(i < 3){
				line(img, Point(x1[i]*100+200,y1[i]*100+200), Point(x2[i]*100+200,y2[i]*100+200), Scalar(255,0,0), 3);
				//cout << "x1[" << i << "]:" << x1[i] << "  " <<"y1[" << i << "]:" << y1[i] << endl;
				//cout << "x2[" << i << "]:" << x2[i] << "  " <<"y2[" << i << "]:" << y2[i] << endl;
			//}
			//else{
			//	line(img, Point(x1[i]*100+200,-y1[i]*100+200), Point(x2[i]*100+200,-y2[i]*100+200), Scalar(255,0,0), 3);
			//}
		}else if(change == 2){
			//if(i >= 9){
				line(img, Point(y1[i]*100+200,-x1[i]*100+200), Point(y2[i]*100+200,-x2[i]*100+200), Scalar(255,0,0), 3);
			//}
			//else if(i < 9){
				//line(img, Point(y1[i]*100+200,-x1[i]*100+200), Point(-y2[i]*100+200,-x2[i]*100+200), Scalar(255,0,0), 3);
			//}
			
		}else if(change == 3){
			line(img, Point(-x2[i]*100+200,y1[i]*100+200), Point(-x1[i]*100+200,y2[i]*100+200), Scalar(255,0,0), 3);
		
		}
		
		if(_a[i] *_a[i+1] < 0){
			change++;
		}
		
		
		//cout << "line_cnt:" << line_cnt << endl;
		line_cnt++;
	}
	cv::imshow("window", img);
  cv::waitKey(2);
  ave_cnt++;
	cnt=0;
	line_cnt=0;
	i_cnt=0;
}

void moveave(float laser_msg[]){
	//float a_final,b_final;
	int i , j ;
	
	if(ave_cnt <9 && cnt !=0){
		for(i=1+i_cnt*42;i<=43+i_cnt*42;i++){
			
			ave_laser[i-i_cnt*42][ave_cnt] = laser_msg[i];
			
			
		}
		for(i=1;i<=43;i++){
			for(j=0;j<ave_cnt;j++){
					ave_laser[i][9] += ave_laser[i][j]/ave_cnt;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	else if(ave_cnt == 0) {
		for(i=1+i_cnt*42;i<=43+i_cnt*42;i++){
			
			ave_laser[i-i_cnt*42][ave_cnt] = laser_msg[i];
			
			
		}
		
		for(i=1;i<=43;i++){
			ave_laser[i][9] = ave_laser[i][0];
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
		}
	}
	else{
		//cout << "laser91:" << laser_msg[91] << endl;
		for(i=1;i<=43;i++){
			for(j=0;j<8;j++){
				ave_laser[i][j] = ave_laser[i][j+1];
			} 
		}
		for(i=1+i_cnt*42;i<=43+i_cnt*42;i++){
			ave_laser[i-i_cnt*42][8] = laser_msg[i];
			//cout << "laser" << i << ":" << laser_msg[i] << endl;
			
			
		}
		for(i=1;i<=43;i++){
			for(j=0;j<9;j++){
					ave_laser[i][9] += ave_laser[i][j]/9;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	i_cnt++;
	
	
	
}


void calc(float ave_laser[],float &a,float &b){
	float x_11=0,x_12=0,x_21=0,x_22=0,temp,ave_laser_x[44],ave_laser_y[44],err,x_1_deg,y_1_deg;
  float det=0,a_arr[44],b_arr[44],theta,a_tan,fi;
  int i ;
  //float a=0,b=0;
  a=0,b=0;
   
	for(i=1+cnt*42;i<=43+cnt*42;i++){
		ave_laser_x[i-cnt*42] = ave_laser[i]*cos((i-1-cnt*42)*each_angle*pi/180);
		ave_laser_y[i-cnt*42] = ave_laser[i]*sin((i-1-cnt*42)*each_angle*pi/180);
		//cout << "x" << i << ":" << ave_laser_x[i] << " " <<"y" << i << ":" << ave_laser_y[i] << endl;
	}
	for(i=1+cnt*42;i<=43+cnt*42;i++){
  	x_11+=ave_laser_x[i-cnt*42]*ave_laser_x[i-cnt*42];
  	x_12+=ave_laser_x[i-cnt*42];
  	x_21+=ave_laser_x[i-cnt*42];
  	
  }
  x_22 = 43;
  det = x_11*x_22-x_12*x_21;
  temp = x_22;
  x_22 = x_11;
  x_11 = temp;
  x_11 = x_11/det;
  x_22 = x_22/det;
  x_12=-x_12/det;
  x_21=-x_21/det;
  //cout << "x_11:" << x_11 << endl << "x_12:" << x_12 <<endl << "x_21:" << x_21 <<endl << "x_22:" << x_22 <<endl ;
  
  for(i=1+cnt*42;i<=43+cnt*42;i++){
  	a_arr[i-cnt*42]=x_11*ave_laser[i]*cos((i-1-cnt*42)*each_angle*pi/180)+x_12*1;
  	b_arr[i-cnt*42]=x_21*ave_laser[i]*cos((i-1-cnt*42)*each_angle*pi/180)+x_22*1;
  	//cout << "a[" << i-79-i_cnt*2 << "]:" << a_arr[i-79-i_cnt*2] << endl;
  	//cout << "b[" << i-79-i_cnt*2 << "]:" << b_arr[i-79-i_cnt*2] << endl;
  	
  }
  
  for(i=1+cnt*42;i<=43+cnt*42;i++){
  	a+=a_arr[i-cnt*42]*ave_laser_y[i-cnt*42];
  	b+=b_arr[i-cnt*42]*ave_laser_y[i-cnt*42];
  	//cout <<"laser" << i << ":" << laser_msg[i] << endl;
  	
  }

  
  /*
  a_tan = atan(a);
  if(atan(a) > 0){
  	theta = 90-atan(a)/pi*180;
  }
  else{
  	theta = 90+atan(a)/pi*180;
  }
  fi =150;
  */
  //cout << "a:" << a << endl << "b:" << b << endl << "a_tan:" << a_tan << endl;
  
  
  
  cout << "cnt:"<< cnt <<endl;	

}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "testlmse");

  
  ros::NodeHandle n;
  
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  //ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, chatterCallback);
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
  
  //test(laser_msg);
  



  ros::spin();

  return 0;
}
