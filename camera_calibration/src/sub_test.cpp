#include "ros/ros.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.hpp"

void leftcornerCB(const std_msgs::Float32MultiArray::ConstPtr& leftcornerData){	
	std::vector<float> left_corner_data = leftcornerData->data;

	int corner_size = 9;
	std::vector<cv::Point2f> left_coords;
	left_coords.resize(corner_size);  // depends on your corners size, please check the python file: cameracalibrator.py
	for (int i = 0; i < corner_size; ++i)
	{
		left_coords[i].x = left_corner_data[i];
		left_coords[i].y = left_corner_data[i+corner_size];
		ROS_INFO_STREAM("LEFT Corner " << i << " has x: " << left_coords[i].x << " y: " << left_coords[i].y);

	}
	ROS_INFO("----------------------");

}

void rightcornerCB(const std_msgs::Float32MultiArray::ConstPtr& rightcornerData){	
	std::vector<float> right_corner_data = rightcornerData->data;

	int corner_size = 9;
	std::vector<cv::Point2f> right_coords;
	right_coords.resize(corner_size);  // depends on your corners size, please check the python file: cameracalibrator.py
	for (int i = 0; i < corner_size; ++i)
	{
		right_coords[i].x = right_corner_data[i];
		right_coords[i].y = right_corner_data[i+corner_size];
		//ROS_INFO_STREAM("RIGHT Corner " << i << " has x: " << right_coords[i].x << " y: " << right_coords[i].y);

	}
	// ROS_INFO("----------------------");

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "subscriber_test"); //name this node
    ros::NodeHandle nh; 

    ros::Subscriber leftcorner_subscriber = nh.subscribe("/left_corners", 1, leftcornerCB);
    ros::Subscriber rightcorner_subscriber = nh.subscribe("/right_corners", 1, rightcornerCB);


    ros::spin();
    return 0; // should never get here, unless roscore dies
}

