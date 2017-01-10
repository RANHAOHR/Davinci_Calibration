/***************************	
    // 1- Tool coordinate on Camera
    // 2- Tool coordinate on Calibration pattern
    // 3- Frame from calibration pattern to tool 
    // 4- Define calibration pattern corners (world coordinates) 
    // 5- Subscribe to calibration node to get coordinates of chessboard corners
    	  a- Chessboard camera coordinates are in pixel, convert them into m or mm
    // 6- Get camera to calibration pattern transformation
    //    a- Call solvePnP with world coordinates and camera coordinates of chessboard
    // 7-
**************************/
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.hpp"

int corner_size;

void cornerSizeCB(const std_msgs::Int32::ConstPtr& cornerSizeData){

	int corner_size = cornerSizeData->data;

	ROS_INFO_STREAM("Size of corners: " << corner_size);
	ROS_INFO("----------------------");

}

void leftcornerCB(const std_msgs::Float32MultiArray::ConstPtr& leftcornerData){

	std::vector<float> left_corner_data = leftcornerData->data;

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

	std::vector<cv::Point2f> right_coords;
	right_coords.resize(corner_size);  // depends on your corners size, please check the python file: cameracalibrator.py
	for (int i = 0; i < corner_size; ++i)
	{
		right_coords[i].x = right_corner_data[i];
		right_coords[i].y = right_corner_data[i+corner_size];
		ROS_INFO_STREAM("RIGHT Corner " << i << " has x: " << right_coords[i].x << " y: " << right_coords[i].y);

	}
	ROS_INFO("----------------------");

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "subscriber_test");
    ros::NodeHandle nh; 

	ros::Subscriber corner_size_subscriber = nh.subscribe("/get_corner_size", 1, cornerSizeCB);
    ros::Subscriber leftcorner_subscriber = nh.subscribe("/left_corners", 1, leftcornerCB);
    ros::Subscriber rightcorner_subscriber = nh.subscribe("/right_corners", 1, rightcornerCB);
   
    ros::spin();
    return 0; // should never get here, unless roscore dies
}

