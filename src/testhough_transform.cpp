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
float ave_laser_right[21][10]={},ave_laser_forward[21][10]={},ave_laser_left[21][10]={};
int cnt, i_cnt ,wall_cnt=0;
int x1_right,x2_right,y1_right,y2_right,x1_forward,x2_forward,y1_forward,y2_forward;
int x1_left,x2_left,y1_left,y2_left;
float x[21]={},y[21]={};
void right_wall(float [],int&,int&,int&,int&);
void forward_wall(float [],int&,int&,int&,int&);
void left_wall(float [],int&,int&,int&,int&);
void showline();
void hough_transform(float ave_laser[21][10],int&,int&,int&,int&);
void scanCallback(const sensor_msgs::LaserScan msg)
{
  int i ;
  
  for(i=1;i<683;i++){
   laser_msg[i] = msg.ranges[i];
  }
	right_wall(laser_msg,x1_right,x2_right,y1_right,y2_right);
	forward_wall(laser_msg,x1_forward,x2_forward,y1_forward,y2_forward);
	left_wall(laser_msg,x1_left,x2_left,y1_left,y2_left);
	
	/*
	for(i=75;i<=95;i++){
		ave_laser_right[i-75][9] = laser_msg[i];
	}
	
  hough_transform(ave_laser_right);
  */
  cout << "x1_right:" << x1_right << " " << "y1_right:" << y1_right << endl;
  cout << "x2_right:" << x2_right << " " << "y2_right:" << y2_right << endl;
  
  //cout << "x1_forward:" << x1_forward << " " << "y1_forward:" << y1_forward << endl;
  //cout << "x2_forward:" << x2_forward << " " << "y2_forward:" << y2_forward << endl;
  
  /*
  Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
  line(img, Point(x1_right+200,y1_right+200), Point(x2_right+200,y2_right+200), Scalar(255,0,0), 3);
    
  cv::imshow("window", img);
  cv::waitKey(2);
  */
  showline();
	for(i=0;i<21;i++){
		ave_laser_right[i][9]=0;
	}
	for(i=0;i<21;i++){
		ave_laser_forward[i][9]=0;
	}
	for(i=0;i<21;i++){
		ave_laser_left[i][9]=0;
	}
	wall_cnt++;
	cout << "wall_cnt:" << wall_cnt << endl;
}

void right_wall(float laser_msg[],int &x1,int &x2,int &y1,int &y2){
	int i ,j;
	i_cnt=0;
	
	if(wall_cnt <9 && wall_cnt !=0){
		for(i=75;i<=95;i++){
			
			ave_laser_right[i-75][wall_cnt] = laser_msg[i];
			
			i_cnt++;
		}
		for(i=0;i<21;i++){
			for(j=0;j<wall_cnt;j++){
					ave_laser_right[i][9] += ave_laser_right[i][j]/wall_cnt;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	else if(wall_cnt == 0) {
		for(i=75;i<=95;i++){
			
			ave_laser_right[i-75][wall_cnt] = laser_msg[i];
			
			i_cnt++;
		}
		
		for(i=0;i<21;i++){
			ave_laser_right[i][9] = ave_laser_right[i][0];
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
		}
	}
	else{
		//cout << "laser91:" << laser_msg[91] << endl;
		for(i=0;i<21;i++){
			for(j=0;j<8;j++){
				ave_laser_right[i][j] = ave_laser_right[i][j+1];
			} 
		}
		for(i=75;i<=95;i++){
			ave_laser_right[i-75][8] = laser_msg[i];
			//cout << "laser" << i << ":" << laser_msg[i] << endl;
			
			i_cnt++;
		}
		for(i=0;i<21;i++){
			for(j=0;j<9;j++){
					ave_laser_right[i][9] += ave_laser_right[i][j]/9;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	
	hough_transform(ave_laser_right,x1,x2,y1,y2);
	
}

void forward_wall(float laser_msg[],int &x1,int &x2,int &y1,int &y2){
	int i ,j;
	i_cnt=0;
	
	if(wall_cnt <9 && wall_cnt !=0){
		for(i=331;i<=351;i++){
			
			ave_laser_forward[i-331][wall_cnt] = laser_msg[i];
			
			i_cnt++;
		}
		for(i=0;i<21;i++){
			for(j=0;j<wall_cnt;j++){
					ave_laser_forward[i][9] += ave_laser_forward[i][j]/wall_cnt;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	else if(wall_cnt == 0) {
		for(i=331;i<=351;i++){
			
			ave_laser_forward[i-331][wall_cnt] = laser_msg[i];
			
			i_cnt++;
		}
		
		for(i=0;i<21;i++){
			ave_laser_forward[i][9] = ave_laser_forward[i][0];
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
		}
	}
	else{
		//cout << "laser91:" << laser_msg[91] << endl;
		for(i=0;i<21;i++){
			for(j=0;j<8;j++){
				ave_laser_forward[i][j] = ave_laser_forward[i][j+1];
			} 
		}
		for(i=331;i<=351;i++){
			ave_laser_forward[i-331][8] = laser_msg[i];
			//cout << "laser" << i << ":" << laser_msg[i] << endl;
			
			i_cnt++;
		}
		for(i=0;i<21;i++){
			for(j=0;j<9;j++){
					ave_laser_forward[i][9] += ave_laser_forward[i][j]/9;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	
	hough_transform(ave_laser_forward,x1,x2,y1,y2);
	
}

void left_wall(float laser_msg[],int &x1,int &x2,int &y1,int &y2){
	int i ,j;
	i_cnt=0;
	
	if(wall_cnt <9 && wall_cnt !=0){
		for(i=588;i<=608;i++){
			
			ave_laser_left[i-588][wall_cnt] = laser_msg[i];
			
			i_cnt++;
		}
		for(i=0;i<21;i++){
			for(j=0;j<wall_cnt;j++){
					ave_laser_left[i][9] += ave_laser_left[i][j]/wall_cnt;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	else if(wall_cnt == 0) {
		for(i=588;i<=608;i++){
			
			ave_laser_left[i-588][wall_cnt] = laser_msg[i];
			
			i_cnt++;
		}
		
		for(i=0;i<21;i++){
			ave_laser_left[i][9] = ave_laser_left[i][0];
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
		}
	}
	else{
		//cout << "laser91:" << laser_msg[91] << endl;
		for(i=0;i<21;i++){
			for(j=0;j<8;j++){
				ave_laser_left[i][j] = ave_laser_left[i][j+1];
			} 
		}
		for(i=588;i<=608;i++){
			ave_laser_left[i-588][8] = laser_msg[i];
			//cout << "laser" << i << ":" << laser_msg[i] << endl;
			
			i_cnt++;
		}
		for(i=0;i<21;i++){
			for(j=0;j<9;j++){
					ave_laser_left[i][9] += ave_laser_left[i][j]/9;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	
	hough_transform(ave_laser_left,x1,x2,y1,y2);
	
}


void hough_transform(float ave_laser[21][10],int &x1,int &x2,int &y1,int &y2){
	float acc_arr[41][41]={},r,err[41],err_min=100,final_r,final_angle;
	int r_angle[2],err_num;
	float max_x=0;
	float acc_num[41];
	int ticket_max=0;
	float a=0,b=0,x0,y0;
	int i ,j,k,out=0;
	x1=0,x2=0,y1=0,y2=0;
	//float x1,x2,y1,y2;
	for(i=0;i<21;i++){
		if(isnan(ave_laser[i][9])){
			//cout << "out nan" << endl;
			out = 1;
			break;
		}
	}
	if(out == 0){
	for(i=0;i<21;i++){
		x[i] = ave_laser[i][9]*cos((i-10)*0.3515*pi/180);
		y[i] = ave_laser[i][9]*sin((i-10)*0.3515*pi/180);
		if(max_x < ave_laser[i][9]){
			max_x = ave_laser[i][9];
		}
		cout << "ave_laser[" << i << "]:" << ave_laser[i][9] << endl;
		//cout << "x" << i << ":" << x[i] << endl;
	}
	cout << "max_x:" << max_x << endl;
	for(i=0;i<41;i++){
		acc_num[i] = -max_x+(max_x*2/40)*i;
		//acc_num[i] = (max_x*2/20)*i;
		//cout << "acc_num" << i << ":" << acc_num[i] << endl;
	}
	
	for(i=0;i<21;i++){
		for(j=0;j<41;j++){
			r = x[i]*cos(j*(pi/40))+y[i]*sin(j*(pi/40));
			for(k=0;k<41;k++){
				err[k] = abs(acc_num[k]- r) ;
				if(err_min > err[k]){
					err_min = err[k];
					err_num = k;
					//cout << "acc_num" << k << ":" << acc_num[k] << endl;
				}
				//cout << "err" << k << ":" << err[k] << endl;
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
	
	a =  cos(final_angle);
	b =  sin(final_angle);
	x0 = a*final_r;
	y0 = b*final_r;
	x1 = int(x0*100+100*(-b));
	y1 = int(y0*100+100*a);
	//y1=int(y0*100+y[0]*1000);
	x2 = int(x0*100-100*(-b));
	y2 = int(y0*100-100*a);
	//y2=int(y0*100+y[20]*1000);
	}
	else if(out == 1){
		cout << "out nan break" << endl;
		
	}
	//cout << "x1_right2:" << x1_right << " " << "y1_right2:" << y1_right << endl;
  //cout << "x2_right2:" << x2_right << " " << "y2_right2:" << y2_right << endl;
	/*
	Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
  line(img, Point(x1_right+200,y1_right+200), Point(x2_right+200,y2_right+200), Scalar(0,255,0), 3);
  line(img, Point(y1_forward+200,-x1_forward+200), Point(y2_forward+200,-x2_forward+200), Scalar(255,0,0), 3);  
  cv::imshow("window", img);
  cv::waitKey(2);
	*/
	
}

void showline(){
	Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
  line(img, Point(x1_right+200,y2_right+200), Point(x2_right+200,y1_right+200), Scalar(0,255,0), 3);
  line(img, Point(y1_forward+200,-x1_forward+200), Point(y2_forward+200,-x2_forward+200), Scalar(0,0,255), 3);
  line(img, Point(-x2_left+200,y2_left+200), Point(-x1_left+200,y1_left+200), Scalar(255,0,0), 3);
  line(img, Point(200,200), Point(200,201), Scalar(255,255,0), 3);
  cv::imshow("window", img);
  cv::waitKey(2);
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



