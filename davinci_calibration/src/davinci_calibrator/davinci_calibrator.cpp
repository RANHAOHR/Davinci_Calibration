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

	lcorner_size = 0;
	rcorner_size = 0;

	freshLeftCorner = false;
	freshRightCorner = false;
	boardMatch = false;
    freshMakers = false;
    freshCameraInfo = false;

	left_corner_coordinates.resize(lcorner_size);  // initialization
	right_corner_coordinates.resize(rcorner_size);

    g_bm = 	(cv::Mat_<double>(4,4) << 0, -1, 0, -0.08,   ///meters or millimeters
            0, 0, 1, 0,
            -1, 0, 0, 0,
            0, 0, 0, 1);

    g_M1_M0 = cv::Mat::eye(4,4,CV_64F);

    /*
     * base transformation vector
     */
    collect_data_num = 0;

    desired_test_num = 1;
    g_base_M0_data.resize(collect_data_num);

    camera_intrinsic_r = cv::Mat::eye(3,3,CV_32FC1);
    camera_intrinsic_l = cv::Mat::eye(3,3,CV_32FC1);

	leftcorner_size_subscriber = nh_.subscribe("/get_corner_size_l", 1, &DavinciCalibrator::leftCornerSizeCB, this);
	rightcorner_size_subscriber = nh_.subscribe("/get_corner_size_r", 1, &DavinciCalibrator::rightCornerSizeCB, this);

	leftcorner_subscriber = nh_.subscribe("/left_corners", 1, &DavinciCalibrator::leftcornerCB, this);
	rightcorner_subscriber = nh_.subscribe("/right_corners", 1, &DavinciCalibrator::rightcornerCB, this);

	polaris_subscriber = nh_.subscribe("/polaris_sensor/targets", 1, &DavinciCalibrator::polarisTargetsCB, this);

    cameraMat_subscriber_r = nh_.subscribe("/davinci_endo/unsynced/right/camera_info", 1, &DavinciCalibrator::cameraIntrinsicRightCB, this);
    cameraMat_subscriber_l = nh_.subscribe("/davinci_endo/unsynced/left/camera_info", 1, &DavinciCalibrator::cameraIntrinsicLeftCB, this);
}

/***DEBUG if necessary, and for the computing of corner coordinates***/
void DavinciCalibrator::leftCornerSizeCB(const std_msgs::Int32::ConstPtr& leftCornerSizeData){

	lcorner_size = leftCornerSizeData->data;   /////it's better not to change this value frequently

//	ROS_INFO_STREAM("SIZE of left corners: " << lcorner_size);
}

void DavinciCalibrator::rightCornerSizeCB(const std_msgs::Int32::ConstPtr& rightCornerSizeData){

	rcorner_size = rightCornerSizeData->data;   /////it's better not to change this value frequently

//	ROS_INFO_STREAM("SIZE of right corners: " << rcorner_size);

}

void DavinciCalibrator::leftcornerCB(const std_msgs::Float32MultiArray::ConstPtr& leftcornerData){

	std::vector<float> left_corner_data = leftcornerData->data;   /*received data has 0 points not the same size with corner_size */
	
	left_corner_coordinates.resize(lcorner_size);  //
	if (lcorner_size > 0 && left_corner_data.size() > 0) //both topics received
	{

		for (int i = 0; i < lcorner_size; ++i)   
		{
			left_corner_coordinates[i].x = left_corner_data[i];
			left_corner_coordinates[i].y = left_corner_data[i+lcorner_size];
			// ROS_INFO_STREAM("LEFT Corner " << i << " has x: " << left_corner_coordinates[i].x << " y: " << left_corner_coordinates[i].y);
		}

		ROS_INFO("----- get LEFT corner -----");
		freshLeftCorner = true;

	}else{
		freshLeftCorner = false;
	}

}

void DavinciCalibrator::rightcornerCB(const std_msgs::Float32MultiArray::ConstPtr& rightcornerData){

	std::vector<float> right_corner_data = rightcornerData->data;

	right_corner_coordinates.resize(rcorner_size);  
	if (rcorner_size > 0 && right_corner_data.size() > 0) //both topics received
	{
        for (int i = 0; i < rcorner_size; ++i)
        {
            right_corner_coordinates[i].x = right_corner_data[i];
            right_corner_coordinates[i].y = right_corner_data[i+rcorner_size];
            // ROS_INFO_STREAM("RIGHT Corner " << i << " has x: " << right_corner_coordinates[i].x << " y: " << right_corner_coordinates[i].y);
        }

        ROS_INFO("----- get RIGHT corner -----");
		freshRightCorner = true;

	}else{
		freshRightCorner = false;
	}

}

void DavinciCalibrator::cameraIntrinsicRightCB(const sensor_msgs::CameraInfo::ConstPtr &cameraIntrinsicRight){

    // ROS_INFO("IN callback");
    camera_intrinsic_r.at<float>(0,0) = cameraIntrinsicRight->K[0];
    camera_intrinsic_r.at<float>(0,1) = cameraIntrinsicRight->K[1];
    camera_intrinsic_r.at<float>(0,2) = cameraIntrinsicRight->K[2];
    camera_intrinsic_r.at<float>(1,0) = cameraIntrinsicRight->K[3];
    camera_intrinsic_r.at<float>(1,1) = cameraIntrinsicRight->K[4];
    camera_intrinsic_r.at<float>(1,2) = cameraIntrinsicRight->K[5];

    freshCameraInfo = true;
}

void DavinciCalibrator::cameraIntrinsicLeftCB(const sensor_msgs::CameraInfo::ConstPtr &cameraIntrinsicLeft){

    camera_intrinsic_l.at<float>(0,0) = cameraIntrinsicLeft->K[0];
    camera_intrinsic_l.at<float>(0,1) = cameraIntrinsicLeft->K[1];
    camera_intrinsic_l.at<float>(0,2) = cameraIntrinsicLeft->K[2];
    camera_intrinsic_l.at<float>(1,0) = cameraIntrinsicLeft->K[3];
    camera_intrinsic_l.at<float>(1,1) = cameraIntrinsicLeft->K[4];
    camera_intrinsic_l.at<float>(1,2) = cameraIntrinsicLeft->K[5];

    freshCameraInfo = true;
}

void DavinciCalibrator::setBoardCoord(){

    if (lcorner_size != 0 &&  lcorner_size == rcorner_size)  ///make sure received corners from both cameras
    {
    	int corner_size = lcorner_size;
    	if(corner_size == 35){  //TODO: hard coding since this depends on each chessboard: 3 * 5

    		boardMatch  = true;

	        int row_size = 5;  //y
	        int col_size = 7;  //x
            float corner_dist = 0.015;

	        board_coordinates.resize(corner_size);
	        /****set x and y for board****/
	        for (int j = 0; j < row_size; ++j) {  ///row

	            for (int i = col_size * j; i < col_size + col_size * j; ++i) {  ///col

                    board_coordinates[i].y = j * corner_dist;

	                if(j>0){
	                    board_coordinates[i].x = board_coordinates[i-col_size].x;
	                }
	                else{
                        board_coordinates[i].x = corner_dist * i; ///first row
	                }
	            }
	        }

	        /*** set z coordinates for board***/
	        for (int k = 0; k < corner_size ; ++k) {
	            board_coordinates[k].z = 0.0;
	        }

            if(collect_data_num <= desired_test_num){
                for (int i = 0; i < board_coordinates.size(); ++i) {
                    total_board_coordinates.push_back(board_coordinates[i]);
                }
            }

    	}
    	else{
        	ROS_ERROR("---Inside setBoardCoord(), Cannot resolve the board corner SIZE---");
        	boardMatch = false;
    	}

    }
    else{ 	
        ROS_ERROR("Cannot set the 3D coordinates correctly");
        boardMatch = false;
    }
}

void DavinciCalibrator::polarisTargetsCB(const geometry_msgs::PoseArray::ConstPtr& target_poses){

    unsigned long target_size = target_poses->poses.size();

    marker_poses.resize(target_size);

    cv::Mat Ttvec = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat Trvec = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat Tquaternion = cv::Mat::zeros(4,1,CV_64F);
    cv::Mat Trot = cv::Mat::zeros(3,3,CV_64F);

    if(target_size == 2){
        for (int i = 0; i < target_size; ++i) {
            marker_poses[i] = cv::Mat::eye(4,4,CV_64F);

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

            // ROS_INFO_STREAM("marker poses " << i << "is:" << marker_poses[i]);
        }

        double marker_1 = marker_poses[0].at<double>(0,0);
        double marker_2 = marker_poses[1].at<double>(0,0);

        if( ( marker_1 != marker_1 ) || ( marker_2 != marker_2 ) )  //if the marker poses are not NAN
        {
        	ROS_ERROR("Polaris sensor gives NAN poses, please check the positions of markers and tracker!");
        	freshMakers = false;
        }
        else{

        	computeMakersGeometry( marker_poses, g_M1_M0);
            ROS_INFO_STREAM("The transformation between two markers: " << g_M1_M0);
			freshMakers = true;
        }

    }
    else{
        ROS_INFO_STREAM("Non-expected number of target detected, " << target_size << " Targets");
        freshMakers = false;
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

void DavinciCalibrator::convertQuaternionsToRot( const cv::Mat &quaternion, cv::Mat &rot_matrix ){

    double qx = quaternion.at<double>(0,0);
    double qy = quaternion.at<double>(1,0);
    double qz = quaternion.at<double>(2,0);
    double qw = quaternion.at<double>(3,0);

    const double Norm = 1.0f/sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    qx *= Norm;
    qy *= Norm;
    qz *= Norm;
    qw *= Norm;

    rot_matrix = (cv::Mat_<double>(3,3) << 1.0 - 2.0*qy*qy - 2.0f*qz*qz, 2.0f*qx*qy - 2.0f*qz*qw, 2.0f*qx*qz + 2.0f*qy*qw,
            2.0f*qx*qy + 2.0f*qz*qw, 1.0f - 2.0f*qx*qx - 2.0f*qz*qz, 2.0f*qy*qz - 2.0f*qx*qw,
            2.0f*qx*qz - 2.0f*qy*qw, 2.0f*qy*qz + 2.0f*qx*qw, 1.0f - 2.0f*qx*qx - 2.0f*qy*qy);

}

void DavinciCalibrator::computeCameraPose(const std::vector<cv::Point2f> &corner_coords, std::vector<cv::Point2f> &total_corner_coords, const cv::Mat &cameraMatrix, cv::Mat &output_cam_pose){

    output_cam_pose = cv::Mat::eye(4,4,CV_64F);

	cv::Mat cam_rvec = cv::Mat::zeros(3,1,CV_64F);
	cv::Mat cam_tvec = cv::Mat::zeros(3,1,CV_64F);

	cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;

    ROS_INFO_STREAM("total_board_coordinates: " << total_board_coordinates.size());
    ROS_INFO_STREAM("COUNT:" << collect_data_num );


    for (int i = 0; i < corner_coords.size(); ++i) {
        total_corner_coords.push_back(corner_coords[i]);
    }
    // ROS_INFO_STREAM("total_corner_coords: " << total_corner_coords);


     if(collect_data_num == desired_test_num){
        cv::solvePnP(total_board_coordinates, total_corner_coords, cameraMatrix, distCoeffs, cam_rvec, cam_tvec);

        cv::Mat R;
        cv::Rodrigues(cam_rvec, R); // R is 3x3

        //	ROS_INFO_STREAM("Rodrigues: " << R);
        R.copyTo(output_cam_pose.colRange(0,3).rowRange(0,3));
        cam_tvec.copyTo(output_cam_pose.colRange(3,4).rowRange(0,3));

        cv::Mat world_frame = cv::Mat::eye(4,4, CV_32FC1);

        world_frame = output_cam_pose.inv();
    }

}

//use this computation to avoid the singularity of opencv inv() for a SE(3) group
void DavinciCalibrator::computeInvSE(const cv::Mat &inputMat, cv::Mat &outputMat){

    outputMat = cv::Mat::eye(4,4,CV_64F);

    cv::Mat R = inputMat.colRange(0,3).rowRange(0,3);
    cv::Mat p = inputMat.colRange(3,4).rowRange(0,3);

    /*debug: opencv......*/
    cv::Mat R_rat = R.clone();
    cv::Mat p_tra = p.clone();

    R_rat = R_rat.t();  // rotation of inverse
    p_tra = -R_rat * p_tra; // translation of inverse

    R_rat.copyTo(outputMat.colRange(0,3).rowRange(0,3));
    p_tra.copyTo(outputMat.colRange(3,4).rowRange(0,3));

}

void DavinciCalibrator::computeMakersGeometry( const std::vector<cv::Mat> &markers, cv::Mat &outputGeometry){

    unsigned long maker_size = markers.size();

    if(maker_size != 2){
        ROS_INFO("Makers size is not 2, check the tracker! ");
    }
    else{
        cv::Mat inv_marker_board = cv::Mat::eye(4,4,CV_64F);
        outputGeometry = cv::Mat::eye(4,4,CV_64F);

        // ROS_INFO_STREAM("marker1 :" << markers[1]);
        // computeInv(markers[1], inv_marker_board);
        inv_marker_board = markers[1].inv();

        // ROS_INFO_STREAM("marker2 :" << markers[0]);
        outputGeometry =  inv_marker_board * markers[0];

    }

}

void DavinciCalibrator::computeBaseTransformation(const cv::Mat &robotBase){

    cv::Mat g_tool_M = cv::Mat::eye(4, 4, CV_32FC1);  ///need to be set

    cv::Mat g_base_M0 = cv::Mat::eye(4, 4, CV_32FC1);
    g_base_M0 = robotBase * g_tool_M * g_M1_M0;

}

/***********************Testing functions************************/
void DavinciCalibrator::testCamToBoard(const std::vector<cv::Mat> &markers, const cv::Mat &g_BM, const cv::Mat &solvePnpCamBoard ){

    ROS_INFO_STREAM("THE solvePnpCamBoard is : " << solvePnpCamBoard);
    cv::Mat G_CT2 = (cv::Mat_<double>(4,4) <<0.1607793043354856, -0.9851130261966847, -0.06084686446323454, 0.08109460651262057,
    0.9858146006241727, 0.1632905939146688, -0.03880406080811315, 0.02128999322747714,
    0.04816210640744724, -0.05374483748793639, 0.9973925525838829, -0.16771031313948,
    0, 0, 0, 1);

    cv::Mat g_CB = cv::Mat::eye(4, 4, CV_32FC1);

    cv::Mat g_T2P;
    cv::Mat g_T1B;

    g_T2P = markers[0].inv();
    g_T1B = g_BM.inv();

    g_CB = G_CT2 * g_T2P * markers[1] * g_T1B;

    ROS_INFO_STREAM("THE G_CB is : " << g_CB);
    cv::Mat diffMat = g_CB * solvePnpCamBoard.inv();

    ROS_INFO_STREAM("THE diffMat is : " << diffMat);

}

void DavinciCalibrator::testCamToBoard2(const std::vector<cv::Mat> &markers, const cv::Mat &g_BM){

    cv::Mat g_PT1 = markers[1];

    cv::Mat g_T1B;
    g_T1B = g_BM.inv();

    cv::Mat G_PB = g_PT1 * g_T1B;

    ROS_INFO_STREAM("THE G_PB is : " << G_PB);

}

void DavinciCalibrator::testCamToBoard3(){
        cv::Mat G_PB = (cv::Mat_<double>(4,4) << 0.08355063217803883, 0.2769054032829358, 0.9572579012446775, 0.2008550584494323,
    0.9887885734369832, -0.1423397653662674, -0.04512813131442052, 0.03006885632803519,
    0.1237596416571481, 0.9502961584833384, -0.285693476069552, -0.8546840414991106,
    0, 0, 0, 1);

    cv::Mat G_PB_ = (cv::Mat_<double>(4,4) << 0.08706701189203561, 0.2956263601711824, 0.9513276988567769, 0.2050166998206809,
    0.9861718929741012, -0.1607504217701431, -0.04030259803782906, 0.01324719257089722,
    0.1410118184694809, 0.9416816644029236, -0.3055344661069692, -0.8206988231121584,
    0, 0, 0, 1);

    cv::Mat G_CB = (cv::Mat_<double>(4,4) << 0.9950254357753222, -0.004331943488460038, -0.09952696331016452, -0.005120913236402995,
    0.009169681362372699, 0.9987955449729388, 0.04820141373365264, -0.05647352268646368,
    0.09919828175851937, -0.04887426324583309, 0.9938666949286179, 0.2363078204198479,
    0, 0, 0, 1);

    cv::Mat G_CB_ = (cv::Mat_<double>(4,4) << 0.9966854093782799, -0.03133994751117807, -0.07507331365037354, -0.01758979821069988,
    0.03001680359941484, 0.9993746623307878, -0.01868892166225873, -0.0212040637179222,
    0.07561207730333108, 0.01637351462638645, 0.9970028694967005, 0.2289192065542711,
    0, 0, 0, 1);

    cv::Mat G_PB_INV = cv::Mat::eye(4,4,CV_32FC1);

    G_PB_INV = G_PB.inv();

    cv::Mat G_PBB = G_PB_INV * G_PB_;

    ROS_INFO_STREAM("G_PBB: " << G_PBB );

    cv::Mat G_CB_INV = cv::Mat::eye(4,4,CV_32FC1);
    G_CB_INV = G_CB.inv();

    cv::Mat G_CBB = G_CB_INV * G_CB_;

    ROS_INFO_STREAM("G_CBB: " << G_CBB );

    cv::Mat diffMat = G_PBB - G_CBB;

    ROS_INFO_STREAM("diffMat: " << diffMat );
}