#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

void chatterCallback(const sensor_msgs::LaserScan msg)
{
  cout << "0: " << msg.ranges[0] << endl;
  cout << "90: " << msg.ranges[89] << endl;
  cout << "180: " << msg.ranges[179] << endl;
  cout << "270: " << msg.ranges[269] << endl;

   ros::Publisher pub = n.advertise<std_msgs::String>("deg0",1000);
   ros::Rate loop_rate(10);
   pub.publish(msg.ranges[0]);
   ROS_INFO("deg0 = %d",msg.ranges[0]);
   ros::spinOnce();
   loop_rate.sleep();
  
} 

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "lidarmsg");
  
  
  ros::NodeHandle n;
  
  

  ros::Subscriber sub = n.subscribe("/scan", 1000, chatterCallback);
  ros::Publisher pub = n.advertise<std_msgs::String>("deg0",1000);

  
  ros::spin();

  return 0;
}
