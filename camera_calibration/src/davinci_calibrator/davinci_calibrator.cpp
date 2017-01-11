/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Case Western Reserve University
 *
 *    Ran Hao <rxh349@case.edu>
 *	  Orhan Ozguner <oxo31@case.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

#include <davinci_calibrator/davinci_calibrator.h>


DavinciCalibrator::DavinciCalibrator(ros::NodeHandle* nodehandle) : nh_( *nodehandle ){

	corner_size = 0;

	left_g = cv::Mat::eye(4,4,CV_64F);
	right_g = cv::Mat::eye(4,4,CV_64F);

	left_rvec = cv::Mat::zeros(3,1,CV_64F);
	left_tvec = cv::Mat::zeros(3,1,CV_64F);
	right_rvec = cv::Mat::zeros(3,1,CV_64F);
	right_tvec = cv::Mat::zeros(3,1,CV_64F);

	left_corner_coordinates.resize(corner_size);  // initialization
	right_corner_coordinates.resize(corner_size);

	corner_size_subscriber = nh_.subscribe("/get_corner_size", 1, &DavinciCalibrator::cornerSizeCB, this);
	leftcorner_subscriber = nh_.subscribe("/left_corners", 1, &DavinciCalibrator::leftcornerCB, this);
	rightcorner_subscriber = nh_.subscribe("/right_corners", 1, &DavinciCalibrator::rightcornerCB, this);

}


void DavinciCalibrator::cornerSizeCB(const std_msgs::Int32::ConstPtr& cornerSizeData){

	corner_size = cornerSizeData->data;   /////it's better not to change this value frequently

	ROS_INFO_STREAM("SIZE of corners: " << corner_size);
	ROS_INFO("----------------------");

}

void DavinciCalibrator::leftcornerCB(const std_msgs::Float32MultiArray::ConstPtr& leftcornerData){

	std::vector<float> left_corner_data = leftcornerData->data;
	
	if ( corner_size != left_corner_data.size() )
	{
		ROS_INFO("The left corner size are not correct! Please check the LIGHTS or board info");
	}
	else{
		left_corner_coordinates.resize(corner_size);  // depends on your corners size, please check the python file: cameracalibrator.py
	}

	for (int i = 0; i < corner_size; ++i)
	{
		left_corner_coordinates[i].x = left_corner_data[i];
		left_corner_coordinates[i].y = left_corner_data[i+corner_size];
		ROS_INFO_STREAM("LEFT Corner " << i << " has x: " << left_corner_coordinates[i].x << " y: " << left_corner_coordinates[i].y);

	}
	ROS_INFO("----------------------");

}

void DavinciCalibrator::rightcornerCB(const std_msgs::Float32MultiArray::ConstPtr& rightcornerData){

	std::vector<float> right_corner_data = rightcornerData->data;

	if ( corner_size != right_corner_data.size() )
	{
		ROS_INFO("The right corner size are not correct! Please check the LIGHTS or board info");
	}
	else{
		right_corner_coordinates.resize(corner_size);  // depends on your corners size, please check the python file: cameracalibrator.py
	}

	for (int i = 0; i < corner_size; ++i)
	{
		right_corner_coordinates[i].x = right_corner_data[i];
		right_corner_coordinates[i].y = right_corner_data[i+corner_size];
		ROS_INFO_STREAM("RIGHT Corner " << i << " has x: " << right_corner_coordinates[i].x << " y: " << right_corner_coordinates[i].y);

	}
	ROS_INFO("----------------------");

}

void DavinciCalibrator::computeCameraPose(const std::vector<cv::Point2f> corner_coords, cv::Mat &cam_pose ){
	

}