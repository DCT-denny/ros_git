#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

void chatterCallback(const sensor_msgs::LaserScan msg)
{
  //cout << msg.ranges.size() << endl;
  for(int i=1;i<683;i++){
   cout << i << ":" << msg.ranges[i] << endl;
  }
  //cout << "1" << ":" << msg.ranges[1] << endl;
  //cout << "150" << ":" << msg.ranges[150] << endl;
   //ros::Publisher pub = n.advertise<std_msgs::String>("deg0",1000);
   //ros::Rate loop_rate(10);
   //pub.publish(msg.ranges[0]);
   //ROS_INFO("deg0 = %d",msg.ranges[0]);
   //ros::spinOnce();
   //loop_rate.sleep();
  
} 

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "laser_msg");
  
  
  ros::NodeHandle n;
  
  

  ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback);
  //ros::Publisher pub = n.advertise<std_msgs::String>("deg0",1000);

  
  ros::spin();

  return 0;
}
