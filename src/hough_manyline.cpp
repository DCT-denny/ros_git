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
float ave_laser[44][10]={},ave_laser_forward[21][10]={},ave_laser_left[21][10]={};
int cnt=0, i_cnt=0 ,ave_cnt=0;
int x1_right,x2_right,y1_right,y2_right,x1_forward,x2_forward,y1_forward,y2_forward;
float x[44]={},y[44]={},_r[17],r_save;
int _x1[17]={},_x2[17]={},_y1[17]={},_y2[17]={},change=1;
void moveave(float [],int&,int&,int&,int&,float &);
void hough_transform(float ave_laser[44][10],int&,int&,int&,int&,float &);
void scanCallback(const sensor_msgs::LaserScan msg)
{
  int i , j;
  
  for(i=1;i<683;i++){
   laser_msg[i] = msg.ranges[i];
  }
  for(j=1;j<=16;j++){
		moveave(laser_msg,x1_right,x2_right,y1_right,y2_right,r_save);
		_x1[j]=x1_right;
		_x2[j]=x2_right;
		_y1[j]=y1_right;
		_y2[j]=y2_right;
		_r[j] = r_save;
		cout << "_x1[" << j << "]:" << _x1[j] << "  " << "_y1[" << j << "]:" << _y1[j] << endl;
		cout << "_x2[" << j << "]:" << _x2[j] << "  " << "_y2[" << j << "]:" << _y2[j] << endl;
		cout << "_r[" << j << "]:" << _r[j] << endl;
	}
	

	Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
	for(i=1;i<=3;i++){
		//if(i==15 || i==16){
		//line(img, Point(-_x1[i]+200,_y1[i]+200), Point(-_x2[i]+200,_y2[i]+200), Scalar(0,255,0), 3);
		//}
		
		//if(change == 1){
  		line(img, Point(_x1[i]+200,-_y1[i]+200), Point(_x2[i]+200,-_y2[i]+200), Scalar(0,255,0), 3);
  	//}
  	//else if(change == 2){
  	//	line(img, Point(_y1[i]+200,-_x1[i]+200), Point(_y2[i]+200,-_x2[i]+200), Scalar(0,0,255), 3);
  	//}
  	//else{
  	//	line(img, Point(-_x2[i]+200,_y1[i]+200), Point(-_x1[i]+200,_y2[i]+200), Scalar(255,0,0), 3);
  	//}
  	
  	//if(_r[i] *_r[i+1] < 0){
		//	change++;
		//}
	}
	line(img, Point(200,200), Point(200,201), Scalar(255,255,0), 3);
	cv::imshow("window", img);
  cv::waitKey(2);

  //cout << "x1_right:" << x1_right << " " << "y1_right:" << y1_right << endl;
  //cout << "x2_right:" << x2_right << " " << "y2_right:" << y2_right << endl;
  
  //cout << "x1_forward:" << x1_forward << " " << "y1_forward:" << y1_forward << endl;
  //cout << "x2_forward:" << x2_forward << " " << "y2_forward:" << y2_forward << endl;
  
 
	
	//for(i=0;i<21;i++){
	//	ave_laser_forward[i][9]=0;
	//}
	cout << "ave_cnt:" << ave_cnt << endl;
	ave_cnt++;
	change=1;
	i_cnt=0;
	cnt=0;
}

void moveave(float laser_msg[],int &x1,int &x2,int &y1,int &y2,float &r){
	int i , j ;
	/*
	for(i=1+i_cnt*42;i<=43+i_cnt*42;i++){
		cout << "laser_msg" << i << ":" << laser_msg[i] << endl;
	}
	*/
	
	if(ave_cnt <9 && ave_cnt !=0){
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
	else if(ave_cnt > 0){
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
	
	//for(i=1;i<=43;i++){
	//	cout << "ave_laser" << i << ":" << ave_laser[i][9] << endl;
	//}
	cout << "i_cnt:" << i_cnt << endl;
	hough_transform(ave_laser,x1,x2,y1,y2,r);
	for(i=1;i<=43;i++){
		ave_laser[i][9]=0;
	}
}



void hough_transform(float ave_laser[44][10],int &x1,int &x2,int &y1,int &y2,float &r_save){
	float acc_arr[41][41]={},r,err[41],err_min=100,final_r,final_angle;
	int r_angle[2],err_num;
	float max_x=0;
	float acc_num[41];
	int ticket_max=0;
	float a=0,b=0,x0,y0;
	int i , j,k,out=0;
	x1=0,x2=0,y1=0,y2=0;
	for(i=1;i<=43;i++){
		if(isnan(ave_laser[i][9])){
			//cout << "out nan" << endl;
			out = 1;
			break;
		}
	}
	
	if(out ==0){
	for(i=1;i<=43;i++){
		x[i] = ave_laser[i][9]*cos((i+cnt*42-85)*0.3515*pi/180);
		y[i] = ave_laser[i][9]*sin((i+cnt*42-85)*0.3515*pi/180);
		if(max_x < ave_laser[i][9]){
			max_x = ave_laser[i][9];
		}
		//cout << "x" << i << ":" << x[i] << endl;
	}
	//cout << "max_x:" << max_x << endl;
	for(i=0;i<41;i++){
		//acc_num[i] = -max_x+(max_x*2/40)*i;
		acc_num[i] = max_x/40*i;
		//cout << "acc_num" << i << ":" << acc_num[i] << endl;
	}
	
	for(i=1;i<=43;i++){
		for(j=0;j<41;j++){
			r = x[i]*cos(j*(pi/40))+y[i]*sin(j*(pi/40));
			if(r > 0){
			for(k=0;k<41;k++){
				err[k] = abs(acc_num[k]- r) ;
				if(err_min > err[k]){
					err_min = err[k];
					err_num = k;
					//cout << "acc_num" << k << ":" << acc_num[k] << endl;
				}
				//cout << "err" << k << ":" << err[k] << endl;
			}
			}
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
	x1 = int(x0*100+x[1]*100*(-b));
	y1 = int(y0*100+y[1]*100*a);
	x2 = int(x0*100+x[43]*100*(-b));
	y2 = int(y0*100+y[43]*100*a);

	}
	else {
		cout << "out nan break" <<endl;
	}
	cnt++;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "wallcorner2");

  
  ros::NodeHandle n;
  
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  //ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, chatterCallback);
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
  
  //test(laser_msg);
  



  ros::spin();

  return 0;
}

