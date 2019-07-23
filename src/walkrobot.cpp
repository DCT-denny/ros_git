#include <stdio.h>
#include <termios.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int64MultiArray.h"
#include <unistd.h>
#include <iostream>
using namespace std;


static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard(void);
void close_keyboard(void);
int kbhit(void);
int readch(void); 
void init_keyboard()
{
	tcgetattr(0,&initial_settings);
	new_settings = initial_settings;
	new_settings.c_lflag |= ICANON;
	new_settings.c_lflag |= ECHO;
	new_settings.c_lflag |= ISIG;
	new_settings.c_cc[VMIN] = 1;
	new_settings.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard()
{
	tcsetattr(0, TCSANOW, &initial_settings);
}

int kbhit()
{
	unsigned char ch;
	int nread;

	if (peek_character != -1) return 1;
	new_settings.c_cc[VMIN]=0;
	tcsetattr(0, TCSANOW, &new_settings);
	nread = read(0,&ch,1);
	new_settings.c_cc[VMIN]=1;
	tcsetattr(0, TCSANOW, &new_settings);
	if(nread == 1) 
	{
		peek_character = ch;
		return 1;
	}
	return 0;
}

int readch()
{
	char ch;

	if(peek_character != -1) 
	{
		ch = peek_character;
		peek_character = -1;
		return ch;
	}
	read(0,&ch,1);
	return ch;
}
 
void chatterCallback(const sensor_msgs::LaserScan msg){
	cout << "0:" << msg.ranges[0] << endl;
	ros::NodeHandle n;
	ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move", 100);
	std_msgs::Int64MultiArray msgmove;
	int pose_pub[2];
  	

	init_keyboard();
	//while(1){
		kbhit();
		switch(readch()){
			case 'g':
				cout << "go str" << endl;
				pose_pub[0]=150;
				pose_pub[1]=150;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;
			case 'l':
				cout << "left" << endl ;
				pose_pub[0]=-150;
				pose_pub[1]=150;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;
			case 'r':
				cout << "right"<< endl ;
				pose_pub[0]=150;
				pose_pub[1]=-150;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;
			case 'b':
				cout << "back" << endl;
				pose_pub[0]=-150;
				pose_pub[1]=-150;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;
			case 's':
				cout << "stop" << endl;
				pose_pub[0]=0;
				pose_pub[1]=0;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;

			
		}
		//printf("\n%d\n", readch());
	//}
	close_keyboard();

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "walkrobot");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/scan",1000,chatterCallback);
	ros::Publisher move_pub = n.advertise<std_msgs::Int64MultiArray>("/move", 100);
/*
	std_msgs::Int64MultiArray msgmove;
	int pose_pub[2];
  	

	init_keyboard();
	while(1){
		kbhit();
		switch(readch()){
			case 'g':
				cout << "go str" << endl;
				pose_pub[0]=-100;
				pose_pub[1]=-100;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;
			case 'l':
				cout << "left" << endl ;
				pose_pub[0]=-100;
				pose_pub[1]=100;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;
			case 'r':
				cout << "right"<< endl ;
				pose_pub[0]=100;
				pose_pub[1]=-100;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;
			case 'b':
				cout << "back" << endl;
				pose_pub[0]=100;
				pose_pub[1]=100;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;
			case 's':
				cout << "stop" << endl;
				pose_pub[0]=0;
				pose_pub[1]=0;
				msgmove.data.push_back(pose_pub[0]);
				msgmove.data.push_back(pose_pub[1]);
				move_pub.publish(msgmove);
				break;

			
		}
		//printf("\n%d\n", readch());
	}
	close_keyboard();
*/
	ros::spin();

	return 0;
}
