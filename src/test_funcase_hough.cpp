#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <opencv2/opencv.hpp>
#define  M_PIf32 3.14159265
#include <iostream>
using namespace std;
using namespace cv;
#define laser_sample_num 683
#define laser_start      1
#define laser_end        169
#define laser_range_min  0.02
#define laser_range_max  1.0
float error_angle;
float error_range;

    float error_sum;
    float error_dot;
    float error_back;
    float lidar_value[170];
    float x[170];
    float y[170];
    float r_save;
    float angle_save;
    float wallangle;
    float wallrange;

    double initspeed;
    double turn;
    double k_p;
    double k_i;
    double k_d;

void hough_transform(float ave_laser[], float &_r_save, float& _angle_save);
void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  for (size_t i=laser_start;i<=laser_end;i++){
  	lidar_value[i-laser_start] = scan_msg->ranges.at(i);
  }
	hough_transform(lidar_value, r_save, angle_save);
  ROS_INFO("final_r: %4.3f  final_angle: %4.3f",r_save, angle_save);
}



void hough_transform(float *ave_laser, float &_r_save, float &_angle_save){
	float laser_value[170];
  int laser_num, laser_locate[170];

  int acc_arr[101][101]={};
  float acc_num[101];

  float max_x=0, rho;

  float err,err_min = 100;
  int err_num;

  int r_angle[2];
  int ticket_max=0;
  //int out=0;

  int cnt = 0,cnt2=0,cnt3=0;
  
  for(int i=0;i<170;i++){
    if(ave_laser[i]>= static_cast<float>(laser_range_min)){
      laser_value[cnt] = ave_laser[i];
      laser_locate[cnt]= i+laser_start;
      cnt++;
    }
  }
  for(int i=0;i<cnt;i++){
    if(ave_laser[i]<= static_cast<float>(laser_range_max)){
      laser_value[cnt2] = laser_value[i];
      laser_locate[cnt2]= laser_locate[i];
      cnt2++;
    }
  }
  
  for(int i=0;i<cnt2;i++){
  	if(isnan(ave_laser[i])){
			cout << "nan" << endl;
		}
		else{
			laser_value[cnt3] = laser_value[i];
      laser_locate[cnt3]= laser_locate[i];
      cnt3++;
		}
  }
  
  //laser_num = cnt2;
  laser_num = cnt3;
  ////////////////////////////////////////////////////////
  //if(out == 0){
    for(int i=0;i<laser_num;i++){
      x[i] = laser_value[i]*cos((laser_locate[i]+171)*0.3515* M_PIf32 /180);
      y[i] = laser_value[i]*sin((laser_locate[i]+171)*0.3515* M_PIf32 /180);
      if(max_x < ave_laser[i]){
        max_x = laser_value[i];
      }
    }
	//ROS_INFO("max_x: %4.3f", max_x);
    //segmentation rho to 41 resolution
    for(int i=0;i<101;i++){
      acc_num[i] = -max_x+(max_x*2/100)*i;
    }

    for(int i=0;i<laser_num;i++){
      for(int j=0;j<101;j++){
        rho = x[i]*cos(j*(M_PIf32/100))+y[i]*sin(j*(M_PIf32/100));
        //ROS_INFO("%d rho: %4.3f", j, rho);
        //if(r > 0){
        for(int k=0;k<101;k++){
          err = abs(acc_num[k]- rho) ;
          if(err_min > err){
            err_min = err;
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

    for(int i=0;i<101;i++){
      for(int j=0;j<101;j++){
        if(ticket_max <acc_arr[j][i]){
          ticket_max=acc_arr[j][i];
          r_angle[0]=j;
          r_angle[1]=i;
        }
        //cout << "acc_arr" << i << j << ":" << acc_arr[i][j] << endl;
      }
    }
    _r_save = acc_num[r_angle[0]];
    _angle_save = r_angle[1]*(M_PIf32/100);


}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "wallcorner2");

  
  ros::NodeHandle n;
  
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  



  ros::spin();

  return 0;
}


