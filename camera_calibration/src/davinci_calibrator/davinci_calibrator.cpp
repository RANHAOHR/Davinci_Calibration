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

DavinciCalibrator::DavinciCalibrator(ros::NodeHandle* nodehandle) : nh_( *nodehandle ){

	corner_size = 0;

//	left_cam_pose = cv::Mat::eye(4,4,CV_64F);
//	right_cam_pose = cv::Mat::eye(4,4,CV_64F);

	left_corner_coordinates.resize(corner_size);  // initialization
	right_corner_coordinates.resize(corner_size);

    marker_poses.resize(0);

	corner_size_subscriber = nh_.subscribe("/get_corner_size", 1, &DavinciCalibrator::cornerSizeCB, this);
	leftcorner_subscriber = nh_.subscribe("/left_corners", 1, &DavinciCalibrator::leftcornerCB, this);
	rightcorner_subscriber = nh_.subscribe("/right_corners", 1, &DavinciCalibrator::rightcornerCB, this);
	// intrinsics_subscriber = nh_.subscribe("camera_intrinsics",1, &DavinciCalibrator::camIntrinsicCB, this);

	polaris_subscriber = nh_.subscribe("/polaris_sensor/targets", 1, &DavinciCalibrator::polarisTargetsCB, this);

	board_coordinates.resize(corner_size);  ////should resize this after receive the real corner size


}

/***DEBUG if necessary, and for the computing of corner coordinates***/
void DavinciCalibrator::cornerSizeCB(const std_msgs::Int32::ConstPtr& cornerSizeData){

	corner_size = cornerSizeData->data;   /////it's better not to change this value frequently

	ROS_INFO_STREAM("SIZE of corners: " << corner_size);
	ROS_INFO("----------------------");

	board_coordinates.resize(corner_size); //TODO: hard coding this or?

}

void DavinciCalibrator::leftcornerCB(const std_msgs::Float32MultiArray::ConstPtr& leftcornerData){

	std::vector<float> left_corner_data = leftcornerData->data;   /*received data has 0 points not the same size with corner_size */
	
	left_corner_coordinates.resize(corner_size);  // debug, please check the python file: cameracalibrator.py

	if (left_corner_data.size() > 0)
	{
		for (int i = 0; i < corner_size; ++i)   
		{
			left_corner_coordinates[i].x = left_corner_data[i];
			left_corner_coordinates[i].y = left_corner_data[i+corner_size];
			ROS_INFO_STREAM("LEFT Corner " << i << " has x: " << left_corner_coordinates[i].x << " y: " << left_corner_coordinates[i].y);

		}
	}

	ROS_INFO("----------------------");

}

void DavinciCalibrator::rightcornerCB(const std_msgs::Float32MultiArray::ConstPtr& rightcornerData){

	std::vector<float> right_corner_data = rightcornerData->data;

	right_corner_coordinates.resize(corner_size);  // debug, please check the python file: cameracalibrator.py

	if (right_corner_data.size() > 0)
	{
		for (int i = 0; i < corner_size; ++i)
		{
			right_corner_coordinates[i].x = right_corner_data[i];
			right_corner_coordinates[i].y = right_corner_data[i+corner_size];
			ROS_INFO_STREAM("RIGHT Corner " << i << " has x: " << right_corner_coordinates[i].x << " y: " << right_corner_coordinates[i].y);

		}
	}

	ROS_INFO("----------------------");

}

/*  Not using this if we already have camera_info
 *  void DavinciCalibrator::camIntrinsicCB(const camera_calibration::intrinsic_param& intrinsicsData){

 *  }
*/

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

	cv::solvePnP(board_coordinates, corner_coords, cameraMatrix, 0.0, cam_rvec, cam_tvec);

	cv::Mat R;
	cv::Rodrigues(cam_rvec, R); // R is 3x3

	/////get the inverse of R and T and put it in the output camera pose
//	R = R.t();  // rotation of inverse
//	cam_tvec = -R * cam_tvec; // translation of inverse

    cv::Mat world_pose = cv::Mat::zeros(4,4,CV_64F);
	R.copyTo(world_pose.colRange(0,3).rowRange(0,3));
	cam_tvec.copyTo(world_pose.colRange(3,4).rowRange(0,3));

    output_cam_pose = world_pose.inv();  ////TODO: notice the zero determinant case

}