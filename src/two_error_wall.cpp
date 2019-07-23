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
float lasterr_angle = 0,lasterr_r=0;
float laser_msg[683]={},err_big,err_small;
float ave_laser_right[21][10]={},ave_laser_forward[21][10]={},ave_laser_left[21][10]={};
int cnt=0, i_cnt ,wall_cnt=0;
int x1_right,x2_right,y1_right,y2_right,x1_forward,x2_forward,y1_forward,y2_forward;
int x1_left,x2_left,y1_left,y2_left,x1_f,x2_f,y1_f,y2_f;
float x[44]={},y[44]={},r_save,angle_save,_r[17]={};
int _x1[17]={},_x2[17]={},_y1[17]={},_y2[17]={},change=1;
float error_arr[9],err;
void hough_transform(float ave_laser[],float &,float &);
void scanCallback(const sensor_msgs::LaserScan msg)
{
	float angle_error=0,r_err=0,turn,rm,lm,der_r,der_angle;
  int i;
  int pose_pub[2];
  ros::NodeHandle n;
  
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
  std_msgs::Int64MultiArray msgmove;
  
  for(i=1;i<683;i++){
   laser_msg[i] = msg.ranges[i];
  }
	hough_transform(laser_msg,r_save,angle_save);
	if(r_save >0)
		angle_error = angle_save;
	else
		angle_error = -(pi-angle_save);
		
	r_err = abs(r_save) - 0.3;
	
	if(cnt < 9 && cnt !=0){
		error_arr[cnt] = angle_error ;
		err = error_arr[cnt/2];
		
	}
	else if(cnt == 0){
		error_arr[cnt] = angle_error ;
		err = error_arr[0];
	}
	else{
		for(i=0;i<8;i++){
			error_arr[i] = error_arr[i+1]; 
		}
		error_arr[8] = angle_error;
		err = error_arr[4];
		
	}
	
	der_angle = err - lasterr_angle ;
	der_r = r_err - lasterr_r ;
	
	turn = (-r_err*2+err*0.8)*300 + (-der_r*2+der_angle*0.8)*150;
	
	cout << "angle_error:" << angle_error <<  endl;
	cout << "r_err:" << r_err << endl;
	cout << "error:" << err <<  endl;
	cout << "cnt:" << cnt <<  endl;
	rm = 180+turn;
	lm = 180-turn;
  
  if(rm > 255){
  	rm = 255; 
  }
  else if(rm < -255){
	 rm = -255;
	}
  if(lm > 255){
  	lm = 255; 
  }
  else if(lm < -255){
	 lm = -255;
	}
	cout << "rm" << int(rm) << endl;
	cout << "lm" << int(lm) << endl;
	pose_pub[0] = int(lm);
	pose_pub[1] = int(rm);
	msgmove.data.push_back(pose_pub[0]);
	msgmove.data.push_back(pose_pub[1]);
	move_pub.publish(msgmove);
  
  lasterr_angle = err;
  lasterr_r = r_err;
  
  cnt++;
 
	
	
}




void hough_transform(float ave_laser[],float &r_save,float &angle_save){
	float acc_arr[41][41]={},r,err[41],err_min=100,final_r,final_angle;
	int r_angle[2],err_num;
	float max_x=0,min_x=100;
	float acc_num[41];
	int ticket_max=0;
	float a=0,b=0,x0,y0;
	int i ,j,k,out=0;
	//x1=0,x2=0,y1=0,y2=0;
	//float x1,x2,y1,y2;
	
	for(i=64;i<=106;i++){
		if(isnan(ave_laser[i])){
			cout << "out nan" << endl;
			out = 1;
			break;
		}
	}
	
	if(out == 0){
	for(i=64;i<=106;i++){
		x[i-64] = ave_laser[i]*cos((i-85)*0.3515*pi/180);
		y[i-64] = ave_laser[i]*sin((i-85)*0.3515*pi/180);
		if(max_x < ave_laser[i]){
			max_x = ave_laser[i];
		}
		/*
		if(min_x > ave_laser[i]){
			min_x = ave_laser[i];
		}
		*/
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
	angle_save = final_angle;
	/*
	a =  cos(final_angle);
	b =  sin(final_angle);
	x0 = a*final_r;
	y0 = b*final_r;
	//x1 = int(x0*100+100*(-b));
	//y1 = int(y0*100+100*a);
	
	x1 = int(x0*100+x[1]*100);
	y1 = int(y0*100+y[1]*100);
	
	x2 = int(x0*100+x[43]*100);
	y2 = int(y0*100+y[43]*100);
	
	
	
	cout << "x[1]:" << x[1] << "  " << "y[1]:" << y[1] << endl;
	cout << "x[43]:" << x[43] << "  " << "y[43]:" << y[43] << endl;
	
	//x2 = int(x0*100-100*(-b));
	//y2 = int(y0*100-100*a);
	*/
	}
	else if(out == 1){
		cout << "out nan break" << endl;
		
	}

	
	
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


