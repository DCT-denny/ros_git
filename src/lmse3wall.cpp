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
float ave_laser[5][10]={},ave_laser_forward[5][10]={},ave_laser_x[5]={},ave_laser_y[5]={};
float ave_laser_left[5][10]={};
int i_cnt,i,j,cnt=0,ab_cnt=0;
float a_mid_arr[5],b_mid_arr[5],a_right,b_right,a_forward,b_forward,a_left,b_left;
void left_wall(float []);
void forward_wall(float []);
void calc(float ave_laser[5][10],float &,float &);
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
	calc(ave_laser,a_right,b_right);
	forward_wall(laser_msg);
	left_wall(laser_msg);
	cout << "a_right:" << a_right << endl << "b_right:" << b_right << endl;
	cout << "a_forward:" << a_forward << endl << "b_forward:" << b_forward << endl;
	cout << "a_left:" << a_left << endl << "b_left:" << b_left << endl;
	Mat img(400, 400, CV_8UC3, Scalar(255,255,255));
	line(img, Point(laser_msg[1]*cos((-84)*0.3515*pi/180)*100+200,(a_right*laser_msg[1]*cos((-84)*0.3515*pi/180)+b_right)*100+200), Point(laser_msg[170]*cos(85*0.3515*pi/180)*100+200,(a_right*laser_msg[170]*cos(85*0.3515*pi/180)+b_right)*100+200), Scalar(255,0,0), 3);
	
	line(img, Point(-laser_msg[682]*cos((-85)*0.3515*pi/180)*100+200,(a_left*laser_msg[682]*cos((-85)*0.3515*pi/180)+b_left)*100+200), Point(-laser_msg[512]*cos(85*0.3515*pi/180)*100+200,(a_left*laser_msg[512]*cos(85*0.3515*pi/180)+b_left)*100+200), Scalar(255,0,0), 3);
	
  line(img, Point((a_forward*laser_msg[256]*cos((-84)*0.3515*pi/180)+b_forward)*100+200,-laser_msg[256]*cos((-84)*0.3515*pi/180)*100+200), Point((a_forward*laser_msg[426]*cos(85*0.3515*pi/180)+b_forward)*100+200,-laser_msg[426]*cos(85*0.3515*pi/180)*100+200), Scalar(255,0,0), 3);
  //cout << "1deg:" << laser_msg[1] << endl << "170deg:" << laser_msg[170] << endl;
  cv::imshow("window", img);
  cv::waitKey(2);
	
	for(i=0;i<5;i++){
		ave_laser[i][9]=0;
	}
	cnt++;
}
void forward_wall(float laser_msg[]){
	i_cnt=0;
	
	if(cnt <9 && cnt !=0){
		for(i=335;i<=347;i=i+3){
			
			ave_laser_forward[i-335-i_cnt*2][cnt] = laser_msg[i];
			
			i_cnt++;
		}
		for(i=0;i<5;i++){
			for(j=0;j<cnt;j++){
					ave_laser_forward[i][9] += ave_laser_forward[i][j]/cnt;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	else if(cnt == 0) {
		for(i=335;i<=347;i=i+3){
			
			ave_laser_forward[i-335-i_cnt*2][cnt] = laser_msg[i];
			
			i_cnt++;
		}
		
		for(i=0;i<5;i++){
			ave_laser_forward[i][9] = ave_laser_forward[i][0];
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
		}
	}
	else{
		//cout << "laser91:" << laser_msg[91] << endl;
		for(i=0;i<5;i++){
			for(j=0;j<8;j++){
				ave_laser_forward[i][j] = ave_laser_forward[i][j+1];
			} 
		}
		for(i=335;i<=347;i=i+3){
			ave_laser_forward[i-335-i_cnt*2][8] = laser_msg[i];
			//cout << "laser" << i << ":" << laser_msg[i] << endl;
			
			i_cnt++;
		}
		for(i=0;i<5;i++){
			for(j=0;j<9;j++){
					ave_laser_forward[i][9] += ave_laser_forward[i][j]/9;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	calc(ave_laser_forward,a_forward,b_forward);
	for(i=0;i<5;i++){
		ave_laser_forward[i][9]=0;
	}
	
}

void left_wall(float laser_msg[]){
	i_cnt=0;
	
	if(cnt <9 && cnt !=0){
		for(i=591;i<=603;i=i+3){
			
			ave_laser_left[i-591-i_cnt*2][cnt] = laser_msg[i];
			
			i_cnt++;
		}
		for(i=0;i<5;i++){
			for(j=0;j<cnt;j++){
					ave_laser_left[i][9] += ave_laser_left[i][j]/cnt;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	else if(cnt == 0) {
		for(i=591;i<=603;i=i+3){
			
			ave_laser_left[i-591-i_cnt*2][cnt] = laser_msg[i];
			
			i_cnt++;
		}
		
		for(i=0;i<5;i++){
			ave_laser_left[i][9] = ave_laser_left[i][0];
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
		}
	}
	else{
		//cout << "laser91:" << laser_msg[91] << endl;
		for(i=0;i<5;i++){
			for(j=0;j<8;j++){
				ave_laser_left[i][j] = ave_laser_left[i][j+1];
			} 
		}
		for(i=591;i<=603;i=i+3){
			ave_laser_left[i-591-i_cnt*2][8] = laser_msg[i];
			//cout << "laser" << i << ":" << laser_msg[i] << endl;
			
			i_cnt++;
		}
		for(i=0;i<5;i++){
			for(j=0;j<9;j++){
					ave_laser_left[i][9] += ave_laser_left[i][j]/9;
			}
			//cout << "ave_laser" << i << ":" << ave_laser_forward[i][9] << endl;
			
		}
		
	}
	calc(ave_laser_left,a_left,b_left);
	for(i=0;i<5;i++){
		ave_laser_left[i][9]=0;
	}
	
}

void calc(float ave_laser[5][10],float &a,float &b){
	float x_11=0,x_12=0,x_21=0,x_22=0,temp,x_arr[5],y_arr[5],err,x_1_deg,y_1_deg;
  float det=0,a_arr[5],b_arr[5],theta,a_tan,fi;
  //float a=0,b=0;
  a=0,b=0;
   
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
  	a_arr[i]=x_11*ave_laser[i][9]*cos((i-2)*each_angle*3*pi/180)+x_12*1;
  	b_arr[i]=x_21*ave_laser[i][9]*cos((i-2)*each_angle*3*pi/180)+x_22*1;
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
  
  
  
  
	//movemid(a,b);

}
/*
void movemid(float a,float b){
	
	if(ab_cnt <5){
		a_mid_arr[ab_cnt] = a ; 
		b_mid_arr[ab_cnt] = b ;
		
	}
	else{
		
	
	}
	
	ab_cnt++;
}
void sort(float a_mid_arr[]){
 float t ;
 for(int i=0;i<4;i++){
  for(int j=0;j<5-i-1;j++){
   if(dist_arr[j] > dist_arr[j+1]){
    t = dist_arr[j+1];
    dist_arr[j+1] = dist_arr[j];
    dist_arr[j] = t ;
   }

  }
 }
 for(int i=0;i<5;i++){
  cout << "dist_arr" << i << ":" << dist_arr[i] << endl;
 }
 dist = dist_arr[2];

}
*/
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
