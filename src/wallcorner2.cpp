#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#define  pi 3.14159265
#include <iostream>
using namespace std;
float laser_msg[360]={};
int i,j,cnt=0;
float theta,err,err_back=0,dist,dist_arr[5];
float fi;
int pose_pub[2];
float imu_init=0,imu_back,imu_diff;
float a_tan ;
void test(float[],float,float,float);
void sort(float[]);
void scanCallback(const sensor_msgs::LaserScan msg)
{
  float x_11=0,x_12=0,x_21=0,x_22=0,temp,x_arr[5],y_arr[5];
  float det=0,a_arr[5],b_arr[5],a=0,b=0;
  
  for(i=0;i<360;i++){
   laser_msg[i] = msg.ranges[i];
  }
  for(i=268;i<=272;i++){
   x_arr[i-268] = laser_msg[i]*cos(((i-268)-2)*pi/180);
   y_arr[i-268] = laser_msg[i]*sin(((i-268)-2)*pi/180);
   cout << "x[" << i-268 << "]:" << x_arr[i-268] << endl;
   cout << "y[" << i-268 << "]:" << y_arr[i-268] << endl;
   //cout << "sin1:" << sin(2*pi/180) <<endl;
  }
  
  
  for(i=268;i<=272;i++){
   x_11+=x_arr[i-268]*x_arr[i-268];
   x_12+=x_arr[i-268];
   x_21+=x_arr[i-268];
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
  //cout << "x_11:" << x_11 << endl << "x_12:" << x_12 <<endl << "x_21:" << x_21 <<endl << "x_22:" << x_22 <<endl ;
  for(i=268;i<=272;i++){
   a_arr[i-268]=x_11*laser_msg[i]*cos(((i-268)-2)*pi/180)+x_12*1;
   b_arr[i-268]=x_21*laser_msg[i]*cos(((i-268)-2)*pi/180)+x_22*1;
   //cout << "a[" << i-268 << "]:" << a_arr[i-268] << endl;
   //cout << "b[" << i-268 << "]:" << b_arr[i-268] << endl;
  }
  
  for(i=268;i<=272;i++){
   a+=a_arr[i-268]*y_arr[i-268];
   b+=b_arr[i-268]*y_arr[i-268];
   //cout <<"laser" << i << ":" << laser_msg[i] << endl;
  }
  a_tan = atan(a);
  if(atan(a) > 0){
   theta = 90-atan(a)/pi*180;
  }
  else{
   theta = 90+atan(a)/pi*180;
  }
  cout << "a:" << a << endl << "b:" << b << endl; 
  test(laser_msg,a,b,theta);
  
}



void test(float laser_msg[],float a,float b,float theta){
 float y ; 
 ros::NodeHandle n;
 ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",1000);
 std_msgs::Int64MultiArray msgmove;
 int pose_pub[2];
 
 err_back=0;
 for(i=270;i<=359;i++){
   y = a*laser_msg[i]*cos((i-270)*pi/180)+b;
   err = laser_msg[i]*sin((i-270)*pi/180)-y;
   //cout << "a:" << a << endl << "b:" << b << endl; 
   //cout << "err" << i << ":"<< err << endl ;
   //cout << "laser" << i << ":" << laser_msg[i] << endl;
   //cout << "theta1:" <<theta << endl;
   //cout << "err-err_back" << i << ":"<< err-err_back << endl ;
   
   if(abs(err-err_back) > 0.3){
    ROS_INFO("corner");
    fi = i-270-1 ;
    if(atan(a) >0){
     dist=laser_msg[i-1]*sin((theta+fi)*pi/180);
    }else{
     dist=laser_msg[i-1]*sin((-theta+fi)*pi/180);
    }
    //dist_arr[cnt] = dist;
    //cout << "dist_arr" << cnt << ":" << dist_arr[cnt] << endl;
    cout << "laser" << i-1 << ":" << laser_msg[i-1] <<endl << "theta:" << theta << endl << "fi:" << fi <<endl <<"sin:" << sin((theta+fi)*pi/180) << endl;
    cout << "distance:" << dist <<endl;
    //cout << "cnt:" << cnt << endl;
    //cnt++;
    break;
   }
   
   err_back = err;
 }
 //if(cnt >5){  //會有點延遲
  //sort(dist_arr);
  //cout << "final dist:" << dist  << endl;
  if(dist >0.21){
   pose_pub[0] = 130;
   pose_pub[1] = 130;

   msgmove.data.push_back(pose_pub[0]);
   msgmove.data.push_back(pose_pub[1]);
   move_pub.publish(msgmove);
   msgmove.data.clear();
  }else{
   pose_pub[0] = 0;
   pose_pub[1] = 0;

   msgmove.data.push_back(pose_pub[0]);
   msgmove.data.push_back(pose_pub[1]);
   move_pub.publish(msgmove);
   msgmove.data.clear();
  }
  //cnt=0;
 //}
 
}
void sort(float dist_arr[]){
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

