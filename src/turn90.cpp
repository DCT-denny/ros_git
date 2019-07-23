#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include "std_msgs/Int64MultiArray.h"
#include <std_srvs/Empty.h>
#include <iostream>
using namespace std;

int cnt=1;
void chatterCallback(const sensor_msgs::Imu imu_msg)
{
  
  float imu_init,imu_back,imu_diff;
  ros::NodeHandle n;
  ros::ServiceClient client=n.serviceClient<std_srvs::Empty>("/imu/set_zero_orientation");
  if(cnt ==1){
   std_srvs::Empty srv;
   client.call(srv);
   imu_init=tf::getYaw(imu_msg.orientation)*180/3.1415926;
  }
  //cout << imu_msg.orientation.z << endl;
  cout << "imu_init_angle:" << imu_init << endl;
  cout << "imu_angle:" << imu_back << endl;
  
  imu_back=tf::getYaw(imu_msg.orientation)*180/3.1415926;
  imu_diff = imu_back - imu_init ;

  
  
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",100);
  std_msgs::Int64MultiArray msgmove;
  int pose_pub[2];
  int i=0;
  
  if(imu_diff >90){
   pose_pub[0] = 0;
   pose_pub[1] = 0;

   msgmove.data.push_back(pose_pub[0]);
   msgmove.data.push_back(pose_pub[1]);
   move_pub.publish(msgmove);
  }
  else{
   pose_pub[0] = -130;
   pose_pub[1] = 130;

   msgmove.data.push_back(pose_pub[0]);
   msgmove.data.push_back(pose_pub[1]);
   move_pub.publish(msgmove);
  }
  cnt++;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "turn90");

  
  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("imu/data", 100, chatterCallback);
  ros::ServiceClient client=n.serviceClient<std_srvs::Empty>("/imu/set_zero_orientation");
  ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move",100);
  
  ros::spin();

  return 0;
}


