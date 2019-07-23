#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#define  pi 3.1415926
#include <iostream>
using namespace std;
using namespace cv;
RNG rng(12345);
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "viewlight");
  ros::NodeHandle n;
	ros::Publisher light_pub = n.advertise<std_msgs::Int64>("light_mode", 1000);  
  ros::Publisher light_err_pub = n.advertise<std_msgs::Float64>("light_err", 1000);  
  
	float cnt,x,y,err,ave_x=0,ave_y=0;
	int i,j,light_mode;
  float area_all=0;
  
  Mat frame;
  VideoCapture cap;
  cap.open(0);
  Mat hsv_img,mask,dst,thresh,dst_erode,dst_dilate;
  
	//Mat lower(1,1,CV_8UC3, Scalar(150,200,0));
	//Mat upper(1,1,CV_8UC3, Scalar(170,250,255));
  //Mat img = imread("/home/denny3/catkin_ws/src/turtlebot3/turtlebot3_bringup/src/lena.jpg",1);
  
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
  //cout << "lower:" << lower << endl;
  
  Mat canny_output;
  Mat kernel3(3,3,CV_8U,Scalar(1));
  Mat kernel5(5,5,CV_8U,Scalar(1));
  Mat kernel7(7,7,CV_8U,Scalar(1));
  while(true)
  {
  	int count=0;
  	ave_x=0,ave_y=0;
  	area_all=0;
  	cap.read(frame);
    if(frame.empty())
    {
    	break;
    }
    cvtColor(frame,hsv_img,COLOR_BGR2HLS);

    inRange(hsv_img,Scalar(120,230,210),Scalar(220,250,255),mask);
    bitwise_and(frame,frame,dst,mask=mask);
    
    erode(mask,dst_erode,kernel3);
    dilate(dst_erode,dst_dilate,kernel7);
    
    Canny(dst_dilate, canny_output, 100, 200, 3 );
    
    threshold(dst_dilate,thresh,127,255,0);
		findContours(thresh,contours,hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    
    //drawContours(mask,contours,-1, (0,255,0), 3);
    
    if(contours.empty() != true){
    	vector<Moments> mu(contours.size() );
			for( int i = 0; i < contours.size(); i++ )
    	{ 
    		mu[i] = moments( contours[i], false ); 
    	}

			//計算輪廓質心
			vector<Point2f> mc( contours.size() );
			for( int i = 0; i < contours.size(); i++ )
    	{ 
    		mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
    	}
    	for(int i= 0; i < contours.size(); i++)
			{
    		for(int j= 0; j < contours[i].size();j++) // run until j < contours[i].size();
    		{
        	ave_x += contours[i][j].x;
    			ave_y += contours[i][j].y;
    			//cout << "x="<< contours[i][j].x << " "; //do whatever
    			//cout << "y="<< contours[i][j].y << endl; //do whatever
    			
    		}
    		count +=contours[i].size();
			}
			/*
			for(i=0;i<contours.size();i++)
			{
				cout << "mc[" << i << "]=" << mc[i] << endl;
			}
			*/
			/// Draw contours
  		Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  		for( int i = 0; i< contours.size(); i++ )
     	{
      	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      	drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
      	circle( drawing, mc[i], 4, color, -1, 8, 0 );
    	}

  		/// Show in a window
  		//namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  		//imshow( "Contours", drawing );

  		/// Calculate the area with the moments 00 and compare with the result of the OpenCV function
  		//printf("\t Info: Area and Contour Length \n");
  		for( int i = 0; i< contours.size(); i++ )
     	{
      	//printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
       area_all += contourArea(contours[i]);
     	}
			
			
			count += contours.size();
			ave_x = ave_x/count;
			ave_y = ave_y/count;
			//cout << contours.size() << contours[i].size() << endl;
    }
    else{
    	ave_x=0;
    	ave_y=0;
    }
    cout << "area_all=" << area_all << endl;
    cout<<"ave_x="<<ave_x<<" "<<"ave_y="<<ave_y<<endl;
    imshow("video demo", frame);
    imshow("video demo2", mask);
    imshow( "erode",dst_erode);
    imshow( "dilate",dst_dilate);
    //imshow("video demo2", dst);
    if (waitKey(1) == 'q'){
    	break;
    }
    //frame.release();
    
    err = 245 - ave_x ;
    std_msgs::Float64 err_light ;
    std_msgs::Int64 light ;
  	if(area_all >1000 && ave_x>=245 && ave_y >=70){
  		light_mode = 0;
  		cout << "start closing light!" << endl;
  	}
  	else if(area_all >1000){
  		light_mode = 1;
  		cout << "light too far!" << endl;
  	}
  	else{
  		light_mode = 2;
  		cout << "light closed!" << endl;
  	}
  	cout << "err:" << err << endl;
  	err_light.data = err ;
  	light_err_pub.publish(err_light);
  	light.data = light_mode ;
  	light_pub.publish(light);
    
  }
  frame.release();
  destroyAllWindows();
  
  
  
	//imshow("dst",dst);
  //imshow("window",img);
  //waitKey(0); 


  ros::spin();

  return 0;
}


