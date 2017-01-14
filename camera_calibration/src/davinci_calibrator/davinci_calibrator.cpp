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
#include <davinci_calibrator/davinci_calibrator.h>
#include "../../include/davinci_calibrator/davinci_calibrator.h"

DavinciCalibrator::DavinciCalibrator(ros::NodeHandle* nodehandle) : nh_( *nodehandle ){

	lcorner_size = 0;
	rcorner_size = 0;
//	left_cam_pose = cv::Mat::eye(4,4,CV_64F);
//	right_cam_pose = cv::Mat::eye(4,4,CV_64F);

	freshLeftCorner = false;
	freshRightCorner = false;
	boardMatch = false;

	left_corner_coordinates.resize(lcorner_size);  // initialization
	right_corner_coordinates.resize(rcorner_size);

    // marker_poses.resize(0);
    // board_coordinates.resize(0);  ////should resize this after receive the real corner size

	leftcorner_size_subscriber = nh_.subscribe("/get_corner_size_l", 1, &DavinciCalibrator::leftCornerSizeCB, this);
	rightcorner_size_subscriber = nh_.subscribe("/get_corner_size_r", 1, &DavinciCalibrator::rightCornerSizeCB, this);

	leftcorner_subscriber = nh_.subscribe("/left_corners", 1, &DavinciCalibrator::leftcornerCB, this);
	rightcorner_subscriber = nh_.subscribe("/right_corners", 1, &DavinciCalibrator::rightcornerCB, this);
	// intrinsics_subscriber = nh_.subscribe("camera_intrinsics",1, &DavinciCalibrator::camIntrinsicCB, this);

	polaris_subscriber = nh_.subscribe("/polaris_sensor/targets", 1, &DavinciCalibrator::polarisTargetsCB, this);

}

/***DEBUG if necessary, and for the computing of corner coordinates***/
void DavinciCalibrator::leftCornerSizeCB(const std_msgs::Int32::ConstPtr& leftCornerSizeData){

	lcorner_size = leftCornerSizeData->data;   /////it's better not to change this value frequently

	ROS_INFO_STREAM("SIZE of left corners: " << lcorner_size);
	ROS_INFO("----------------------");

}

void DavinciCalibrator::rightCornerSizeCB(const std_msgs::Int32::ConstPtr& rightCornerSizeData){

	rcorner_size = rightCornerSizeData->data;   /////it's better not to change this value frequently

	ROS_INFO_STREAM("SIZE of right corners: " << rcorner_size);
	ROS_INFO("----------------------");

}

void DavinciCalibrator::leftcornerCB(const std_msgs::Float32MultiArray::ConstPtr& leftcornerData){

	std::vector<float> left_corner_data = leftcornerData->data;   /*received data has 0 points not the same size with corner_size */
	
	left_corner_coordinates.resize(lcorner_size);  //

	if (lcorner_size > 0)
	{
		if (left_corner_data.size() > 0)
		{
			for (int i = 0; i < lcorner_size; ++i)   
			{
				left_corner_coordinates[i].x = left_corner_data[i];
				left_corner_coordinates[i].y = left_corner_data[i+lcorner_size];
				// ROS_INFO_STREAM("LEFT Corner " << i << " has x: " << left_corner_coordinates[i].x << " y: " << left_corner_coordinates[i].y);

			}

			ROS_INFO("----- get LEFT corner -----");
			freshLeftCorner = true;

		}

	}

}

void DavinciCalibrator::rightcornerCB(const std_msgs::Float32MultiArray::ConstPtr& rightcornerData){

	std::vector<float> right_corner_data = rightcornerData->data;

	right_corner_coordinates.resize(rcorner_size);  

	if (rcorner_size > 0) //
	{
		if (right_corner_data.size() > 0)
		{
			for (int i = 0; i < rcorner_size; ++i)   
			{
				right_corner_coordinates[i].x = right_corner_data[i];
				right_corner_coordinates[i].y = right_corner_data[i+rcorner_size];
				// ROS_INFO_STREAM("RIGHT Corner " << i << " has x: " << right_corner_coordinates[i].x << " y: " << right_corner_coordinates[i].y);

			}

			ROS_INFO("----- get RIGHT corner -----");
			freshRightCorner = true;

		}

	}
	
	// ROS_INFO("----------------------");

}

/*  Not using this if we already have camera_info
 *  void DavinciCalibrator::camIntrinsicCB(const camera_calibration::intrinsic_param& intrinsicsData){

 *  }
*/

void DavinciCalibrator::setBoardCoord(){

    if (lcorner_size != 0 &&  lcorner_size == rcorner_size)  ///make sure received corners from both cameras
    {
    	int corner_size = lcorner_size;

    	if(corner_size == 15){  //TODO: hard coding since this depends on each chessboard: 3 * 5

    		boardMatch  = true; //

	        int row_size = 3;
	        int col_size = 5;

	        board_coordinates.resize(corner_size);
	        /****set x and y for board****/
	        for (int j = 0; j < row_size; ++j) {  ///row

	            for (int i = col_size * j; i < col_size + col_size * j; ++i) {  ///col

	                board_coordinates[i].y = 0.0 + j * 0.01;

	                if(j>0){
	                    board_coordinates[i].x = board_coordinates[i-col_size].x;
	                }
	                else{
	                    board_coordinates[i].x = 0.01 * i; ///first row
	                }
	            }
	        }

	        /*** set z coordinates for board***/
	        for (int k = 0; k < corner_size ; ++k) {
	            board_coordinates[k].z = 0.0;

	        }
    	}
    	else{
        	ROS_ERROR("---Inside setBoardCoord(), Cannot resolve the board corner SIZE---");
        	boardMatch = false;
    	}

    }
    else{ 	
        ROS_INFO("Cannot set the 3D coordinates correctly");
        boardMatch = false;
            
    }

}

void DavinciCalibrator::polarisTargetsCB(const geometry_msgs::PoseArray::ConstPtr& target_poses){

    unsigned long target_size = target_poses->poses.size();
    ROS_INFO_STREAM(target_size << " Target detected");

    marker_poses.resize(target_size);

    cv::Mat Ttvec = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat Trvec = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat Tquaternion = cv::Mat::zeros(4,1,CV_64F);
    cv::Mat Trot = cv::Mat::zeros(3,3,CV_64F);

    for (int i = 0; i < target_size; ++i) {

        marker_poses[i] = cv::Mat::zeros(4,4,CV_64F);

        Ttvec.at<double>(0,0) = target_poses->poses[i].position.x;
        Ttvec.at<double>(1,0) = target_poses->poses[i].position.y;
        Ttvec.at<double>(2,0) = target_poses->poses[i].position.z;
        Tquaternion.at<double>(0,0) = target_poses->poses[i].orientation.x;
        Tquaternion.at<double>(1,0) = target_poses->poses[i].orientation.y;
        Tquaternion.at<double>(2,0) = target_poses->poses[i].orientation.z;
        Tquaternion.at<double>(3,0) = target_poses->poses[i].orientation.w;

        convertQuaternionsToRvec(Tquaternion, Trvec);

        cv::Rodrigues(Trvec, Trot);

        Trot.copyTo(marker_poses[i].colRange(0,3).rowRange(0,3));
        Ttvec.copyTo(marker_poses[i].colRange(3,4).rowRange(0,3));

        ROS_INFO_STREAM("marker poses: " << marker_poses[i]);
    }

}

void DavinciCalibrator::convertQuaternionsToRvec( const cv::Mat &quaternion, cv::Mat &Rod_rvec ){

    double qx = quaternion.at<double>(0,0);
    double qy = quaternion.at<double>(1,0);
    double qz = quaternion.at<double>(2,0);
    double qw = quaternion.at<double>(3,0);

    double angle = 2 * acos(qw);
    double x = qx / pow(1-qw*qw, 0.5);
    double y = qy / pow(1-qw*qw, 0.5);
    double z = qz / pow(1-qw*qw, 0.5);

    Rod_rvec.at<double>(0,0) = angle * x;
    Rod_rvec.at<double>(1,0) = angle * y;
    Rod_rvec.at<double>(2,0) = angle * z;

}
void DavinciCalibrator::computeCameraPose(const std::vector<cv::Point2f> &corner_coords, const cv::Mat &cameraMatrix, cv::Mat &output_cam_pose){

    output_cam_pose = cv::Mat::zeros(4,4,CV_64F);

	cv::Mat cam_rvec = cv::Mat::zeros(3,1,CV_64F);
	cv::Mat cam_tvec = cv::Mat::zeros(3,1,CV_64F);

	cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;

	ROS_INFO_STREAM("board_coordinates: " << board_coordinates);
	ROS_INFO_STREAM("corner_coords: " << corner_coords);

	cv::solvePnP(board_coordinates, corner_coords, cameraMatrix, distCoeffs, cam_rvec, cam_tvec);

	ROS_INFO_STREAM("cam_rvec: " << cam_rvec);
	ROS_INFO_STREAM("cam_tvec: " << cam_tvec);

	cv::Mat R;
	cv::Rodrigues(cam_rvec, R); // R is 3x3

	ROS_INFO_STREAM("Rodrigues: " << R);

	/////get the inverse of R and T and put it in the output camera pose
//	R = R.t();  // rotation of inverse
//	cam_tvec = -R * cam_tvec; // translation of inverse

    cv::Mat world_pose = cv::Mat::eye(4,4,CV_64F);
	R.copyTo(world_pose.colRange(0,3).rowRange(0,3));
	cam_tvec.copyTo(world_pose.colRange(3,4).rowRange(0,3));

	ROS_INFO_STREAM("world_pose: " << world_pose);
    output_cam_pose = world_pose.inv();  ////TODO: notice the zero determinant case

}