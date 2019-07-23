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
float ave_laser[5][10]={},ave_laser_x[5]={},ave_laser_y[5]={};
int i_cnt,i,j,cnt=0,ab_cnt=0,point=0;
float a_mid_arr[5]={},b_mid_arr[5]={},a_final,b_final,a_final_back,b_final_back;
float a_mid_arr_sort[5]={},b_mid_arr_sort[5]={};
void sort(float [],float []);
void moveave(float ,float );
void calc(float ave_laser[5][10]);
void scanCallback(const sensor_msgs::LaserScan msg)
{
  for(i=1;i<683;i++){
  	laser_msg[i] = msg.ranges[i];
  }
	i_cnt=0;
	/*
	for(i=79;i<=91;i=i+3){
		//cout <<"laser" << i << ":"<< laser_msg[i] << endl;
		//cout <<"laser_x" << i << ":"<< laser_msg[i]*cos(((i-79-i_cnt*2)-2)*pi/180) << endl;
		//cout <<"laser_y" << i << ":"<< laser_msg[i]*sin(((i-79-i_cnt*2)-2)*pi/180) << endl;
		ave_laser[i-79-i_cnt*2] += laser_msg[i];
		ave_laser_x[i-79-i_cnt*2] += laser_msg[i]*cos(((i-79-i_cnt*2)-2)*pi/180);
		ave_laser_y[i-79-i_cnt*2] += laser_msg[i]*sin(((i-79-i_cnt*2)-2)*pi/180);
		i_cnt++;
	}
	*/
	if(cnt <9 && cnt !=0){
		for(i=79;i<=91;i=i+3){
			
			ave_laser[i-79-i_cnt*2][cnt] = laser_msg[i];
			
			i_cnt++;
		}
		for(i=0;i<5;i++){
			for(j=0;j<cnt;j++){
					ave_laser[i][9] += ave_laser[i][j]/cnt;
			}
			cout << "ave_laser" << i << ":" << ave_laser[i][9] << endl;
			
		}
		
	}
	else if(cnt == 0) {
		for(i=79;i<=91;i=i+3){
			
			ave_laser[i-79-i_cnt*2][cnt] = laser_msg[i];
			
			i_cnt++;
		}
		
		for(i=0;i<5;i++){
			ave_laser[i][9] = ave_laser[i][0];
			cout << "ave_laser" << i << ":" << ave_laser[i][9] << endl;
		}
	}
	else{
		//cout << "laser91:" << laser_msg[91] << endl;
		for(i=0;i<5;i++){
			for(j=0;j<8;j++){
				ave_laser[i][j] = ave_laser[i][j+1];
			} 
		}
		for(i=79;i<=91;i=i+3){
			ave_laser[i-79-i_cnt*2][8] = laser_msg[i];
			cout << "laser" << i << ":" << laser_msg[i] << endl;
			
			i_cnt++;
		}
		for(i=0;i<5;i++){
			for(j=0;j<9;j++){
					ave_laser[i][9] += ave_laser[i][j]/9;
			}
			cout << "ave_laser" << i << ":" << ave_laser[i][9] << endl;
			
		}
		
	}
	/*
	for(i=0;i<5;i++){
		for(j=0;j<5;j++){
			cout << "laser" << i << j << ave_laser[i][j] << endl;
		}
	}
	*/
	cout << "cnt:" << cnt << endl;
	/*
	if(cnt == 4){
		for(i=0;i<5;i++){
			ave_laser[i] =ave_laser[i]/4;
			ave_laser_x[i] =ave_laser_x[i]/4;
			ave_laser_y[i] =ave_laser_y[i]/4;
			cout << "ave_laser"  << i << ":"<< ave_laser[i]<<endl;
			cout << "ave_laser_x"  << i << ":"<< ave_laser_x[i]<<endl;
			cout << "ave_laser_y"  << i << ":"<< ave_laser_y[i]<<endl;
		}
		calc(ave_laser,ave_laser_x,ave_laser_y);
		cnt=1;
	}
	*/
	calc(ave_laser);
	for(i=0;i<5;i++){
		ave_laser[i][9]=0;
	}
	cnt++;
}

void calc(float ave_laser[5][10]){
	float x_11=0,x_12=0,x_21=0,x_22=0,temp,x_arr[5],y_arr[5],err,x_1_deg,y_1_deg;
  float det=0,a_arr[5],b_arr[5],a=0,b=0,theta,a_tan,fi;
  
	for(i=0;i<5;i++){
		ave_laser_x[i] = ave_laser[i][9]*cos((i-2)*each_angle*3*pi/180);
		ave_laser_y[i] = ave_laser[i][9]*sin((i-2)*each_angle*3*pi/180);
		cout << "x" << i << ":" << ave_laser_x[i] << " " <<"y" << i << ":" << ave_laser_y[i] << endl;
	}
	for(i=0;i<5;i++){
  	x_11+=ave_laser_x[i]*ave_laser_x[i];
  	x_12+=ave_laser_x[i];
  	x_21+=ave_laser_x[i];
  	
  }
  x_22 = 5;
  det = x_11*x_22-x_12*x_21;
  temp = x_22;
  x_22 = x_11;
  x_11 = temp;
  x_11 = x_11/det;
  x_22 = x_22/det;
  x_12=-x_12/det;
  x_21=-x_21/det;
  cout << "x_11:" << x_11 << endl << "x_12:" << x_12 <<endl << "x_21:" << x_21 <<endl << "x_22:" << x_22 <<endl ;
  
  for(i=0;i<5;i++){
  	a_arr[i]=x_11*ave_laser[i][4]*cos((i-2)*each_angle*3*pi/180)+x_12*1;
  	b_arr[i]=x_21*ave_laser[i][4]*cos((i-2)*each_angle*3*pi/180)+x_22*1;
  	//cout << "a[" << i-79-i_cnt*2 << "]:" << a_arr[i-79-i_cnt*2] << endl;
  	//cout << "b[" << i-79-i_cnt*2 << "]:" << b_arr[i-79-i_cnt*2] << endl;
  	
  }
  
  for(i=0;i<5;i++){
  	a+=a_arr[i]*ave_laser_y[i];
  	b+=b_arr[i]*ave_laser_y[i];
  	//cout <<"laser" << i << ":" << laser_msg[i] << endl;
  	
  }

  
  
  a_tan = atan(a);
  if(atan(a) > 0){
  	theta = 90-atan(a)/pi*180;
  }
  else{
  	theta = 90+atan(a)/pi*180;
  }
  fi =150;
  cout << "a:" << a << endl << "b:" << b << endl << "a_tan:" << a_tan << endl;
  moveave(a,b);
  
  point=0;
  if(a_final*a_final_back < 0){
  	point = 1 ;
  }
  
  cout << "a_final:" << a_final << endl << "b_final:" << b_final << endl;
  
  a_final_back = a_final ;
  b_final_back = b_final ;
  
  Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
  
  //if(point == 1){
  //	line(img, Point(laser_msg[1]*cos((-84)*0.3515*pi/180)*100+200,300), Point(laser_msg[170]*cos(85*0.3515*pi/180)*100+200,100), Scalar(255,0,0), 3);
  //}
  //else{
  	line(img, Point(laser_msg[1]*cos((-84)*0.3515*pi/180)*100+200,(a_final*laser_msg[1]*cos((-84)*0.3515*pi/180)+b_final)*100+200), Point(laser_msg[170]*cos(85*0.3515*pi/180)*100+200,(a_final*laser_msg[170]*cos(85*0.3515*pi/180)+b_final)*100+200), Scalar(255,0,0), 3);
  //}
  cout << "1deg:" << laser_msg[1] << endl << "170deg:" << laser_msg[170] << endl;
  cv::imshow("window", img);
  cv::waitKey(2);
	

}

void moveave(float a,float b){
	
	
	if(ab_cnt <5){
		a_mid_arr[ab_cnt] = a ; 
		b_mid_arr[ab_cnt] = b ;
		sort(a_mid_arr,b_mid_arr);
	}
	else{
		for(i=0;i<4;i++){
			a_mid_arr[i] = a_mid_arr[i+1];
			b_mid_arr[i] = b_mid_arr[i+1];
		}
		a_mid_arr[4] = a ; 
		b_mid_arr[4] = b ;
		for(i=0;i<5;i++){
			a_mid_arr_sort[i] = a_mid_arr[i];
			b_mid_arr_sort[i] = b_mid_arr[i];
		}
		sort(a_mid_arr_sort,b_mid_arr_sort);
	}
	for(i=0;i<5;i++){
		cout << "a_mid_arr:" << a_mid_arr[i] << endl;
	}
	ab_cnt++;
}


void sort(float a_mid_arr_sort[],float b_mid_arr_sort[]){
 float t ;
 int index[5]={0,1,2,3,4},index_num;
 
 if(ab_cnt <5){
 	for(int i=0;i<ab_cnt;i++){
 	 for(int j=0;j<ab_cnt-i-1;j++){
 	  if(a_mid_arr_sort[j] > a_mid_arr_sort[j+1]){
 	   t = a_mid_arr_sort[j+1];
 	   a_mid_arr_sort[j+1] = a_mid_arr_sort[j];
 	   a_mid_arr_sort[j] = t ;
 	   
 	   t = index[j+1];
 	   index[j+1] = index[j];
 	   index[j] = t ;
 	  }
 	 }
 	}
 	a_final = a_mid_arr_sort[ab_cnt/2];
 	index_num = index[ab_cnt/2];
 	b_final = b_mid_arr_sort[index_num];
 }
 else
 {
 	for(int i=0;i<4;i++){
 	 for(int j=0;j<5-i-1;j++){
 	  if(a_mid_arr_sort[j] > a_mid_arr_sort[j+1]){
 	   t = a_mid_arr_sort[j+1];
 	   a_mid_arr_sort[j+1] = a_mid_arr_sort[j];
 	   a_mid_arr_sort[j] = t ;
 	   
 	   t = index[j+1];
 	   index[j+1] = index[j];
 	   index[j] = t ;
 	  }
 	 }
 	}
 	a_final = a_mid_arr_sort[2];
 	index_num = index[2];
 	b_final = b_mid_arr_sort[index_num];
 }
 for(i=0;i<5;i++){
 	cout << "a_mid_arr_sort:" << a_mid_arr_sort[i] << endl;
 }
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

