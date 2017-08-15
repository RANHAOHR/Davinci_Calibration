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

boost::mt19937 rng((const uint32_t &) time(0));

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

    g_bm = 	(cv::Mat_<double>(4,4) << 0, -1, 0, -0.08,   ///meters
            0, 0, 1, 0,
            -1, 0, 0, 0,
            0, 0, 0, 1);

    g_M1_M0 = cv::Mat::eye(4,4,CV_64FC1);


    camera_mat_r = cv::Mat::eye(3,3,CV_64FC1);
    camera_mat_l = cv::Mat::eye(3,3,CV_64FC1);

	leftcorner_size_subscriber = nh_.subscribe("/get_corner_size_l", 1, &DavinciCalibrator::leftCornerSizeCB, this);
	rightcorner_size_subscriber = nh_.subscribe("/get_corner_size_r", 1, &DavinciCalibrator::rightCornerSizeCB, this);

	leftcorner_subscriber = nh_.subscribe("/left_corners", 1, &DavinciCalibrator::leftcornerCB, this);
	rightcorner_subscriber = nh_.subscribe("/right_corners", 1, &DavinciCalibrator::rightcornerCB, this);

	polaris_subscriber = nh_.subscribe("/polaris_sensor/targets", 1, &DavinciCalibrator::polarisTargetsCB, this);

    cameraMat_subscriber_r = nh_.subscribe("/davinci_endo/unsynced/right/camera_info", 1, &DavinciCalibrator::cameraMatRightCB, this);
    cameraMat_subscriber_l = nh_.subscribe("/davinci_endo/unsynced/left/camera_info", 1, &DavinciCalibrator::cameraMatLeftCB, this);

    srand((unsigned) time( NULL));  //random number generator
};

/***DEBUG if necessary, and for the computing of corner coordinates***/
void DavinciCalibrator::leftCornerSizeCB(const std_msgs::Int32::ConstPtr& leftCornerSizeData){

	lcorner_size = leftCornerSizeData->data;   /////it's better not to change this value frequently

//	ROS_INFO_STREAM("SIZE of left corners: " << lcorner_size);
}

void DavinciCalibrator::rightCornerSizeCB(const std_msgs::Int32::ConstPtr& rightCornerSizeData){

	rcorner_size = rightCornerSizeData->data;   /////it's better not to change this value frequently

};

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

};

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

};

void DavinciCalibrator::cameraMatRightCB(const sensor_msgs::CameraInfo::ConstPtr &cameraMatRight){

    camera_mat_r.at<double>(0,0) = cameraMatRight->P[0];
    camera_mat_r.at<double>(0,1) = cameraMatRight->P[1];
    camera_mat_r.at<double>(0,2) = cameraMatRight->P[2];
    camera_mat_r.at<double>(1,0) = cameraMatRight->P[4];
    camera_mat_r.at<double>(1,1) = cameraMatRight->P[5];
    camera_mat_r.at<double>(1,2) = cameraMatRight->P[6];
    //ROS_INFO_STREAM("RIGHT: " << camera_mat_r);
    freshCameraInfo = true;
};

void DavinciCalibrator::cameraMatLeftCB(const sensor_msgs::CameraInfo::ConstPtr &cameraMatLeft){

    camera_mat_l.at<double>(0,0) = cameraMatLeft->P[0];
    camera_mat_l.at<double>(0,1) = cameraMatLeft->P[1];
    camera_mat_l.at<double>(0,2) = cameraMatLeft->P[2];
    camera_mat_l.at<double>(1,0) = cameraMatLeft->P[4];
    camera_mat_l.at<double>(1,1) = cameraMatLeft->P[5];
    camera_mat_l.at<double>(1,2) = cameraMatLeft->P[6];
    //ROS_INFO_STREAM("left: " << camera_mat_l);
    freshCameraInfo = true;
};
    
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
	            board_coordinates[k].z = 0.0;  //TODO:
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
};

void DavinciCalibrator::polarisTargetsCB(const geometry_msgs::PoseArray::ConstPtr& target_poses){

    unsigned long target_size = target_poses->poses.size();

    marker_poses.resize(target_size);

    cv::Mat Ttvec = cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat Trvec = cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat Tquaternion = cv::Mat::zeros(4,1,CV_64FC1);
    cv::Mat Trot = cv::Mat::zeros(3,3,CV_64FC1);

    if(target_size == 2){
        for (int i = 0; i < target_size; ++i) {
            marker_poses[i] = cv::Mat::eye(4,4,CV_64FC1);

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
        ROS_INFO_STREAM("G_PM1= " << marker_poses[0]);
        ROS_INFO_STREAM("G_PM2= " << marker_poses[1]);

        double marker_1 = marker_poses[0].at<double>(0,0);   //first one should be on the camera
        double marker_2 = marker_poses[1].at<double>(0,0);     //second one should be on the board

        if( ( marker_1 != marker_1 ) || ( marker_2 != marker_2 ) )  //if the marker poses are not NAN
        {
         	ROS_ERROR("Polaris sensor gives NAN poses, please check the positions of markers and tracker!");
         	freshMakers = false;
        }
        else{

         	computeMakersGeometry( marker_poses, g_M1_M0);
             ROS_INFO_STREAM("G_M1_M0= " << g_M1_M0);  //G_T2_T1, Compute M1 relative to M2

             cv::Mat G_c_base = g_M1_M0.inv();
             ROS_INFO_STREAM(" inv of G_M1_M0= " << G_c_base);  //G_T2_T1
			 freshMakers = true;
        }

    }
    else{
        ROS_INFO_STREAM("Non-expected number of target detected, " << target_size << " Targets");
        freshMakers = false;
    }
};

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

};

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

};

void DavinciCalibrator::computeCameraPose(const std::vector<cv::Point2f> &corner_coords, const cv::Mat &cameraMatrix, cv::Mat &output_cam_pose){

    output_cam_pose = cv::Mat::eye(4,4,CV_64FC1);

	cv::Mat cam_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	cv::Mat cam_tvec = cv::Mat::zeros(3,1,CV_64FC1);

	cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;

    //ROS_INFO_STREAM("board_coordinates: " << board_coordinates);
    //ROS_INFO_STREAM("corner_coords: " << corner_coords);

	cv::solvePnP(board_coordinates, corner_coords, cameraMatrix, distCoeffs, cam_rvec, cam_tvec);

	cv::Mat R;
	cv::Rodrigues(cam_rvec, R); // R is 3x3

	R.copyTo(output_cam_pose.colRange(0,3).rowRange(0,3));
	cam_tvec.copyTo(output_cam_pose.colRange(3,4).rowRange(0,3));

};

//use this computation to avoid the singularity of opencv inv() for a SE(3) group
void DavinciCalibrator::computeInvSE(const cv::Mat &inputMat, cv::Mat &outputMat){

    outputMat = cv::Mat::eye(4,4,CV_64FC1);

    cv::Mat R = inputMat.colRange(0,3).rowRange(0,3);
    cv::Mat p = inputMat.colRange(3,4).rowRange(0,3);

    /*debug: opencv......*/
    cv::Mat R_rat = R.clone();
    cv::Mat p_tra = p.clone();

    R_rat = R_rat.t();  // rotation of inverse
    p_tra = -R_rat * p_tra; // translation of inverse

    R_rat.copyTo(outputMat.colRange(0,3).rowRange(0,3));
    p_tra.copyTo(outputMat.colRange(3,4).rowRange(0,3));

};

void DavinciCalibrator::computeMakersGeometry( const std::vector<cv::Mat> &markers, cv::Mat &outputGeometry){

    unsigned long maker_size = markers.size();

    if(maker_size != 2){
        ROS_INFO("Makers size is not 2, check the tracker! ");
    }
    else{
        cv::Mat temp_marker_board = markers[1].clone();
        outputGeometry = cv::Mat::eye(4,4,CV_64FC1);

        cv::Mat inv_marker_board = temp_marker_board.inv();

        outputGeometry =  inv_marker_board * markers[0];

    }

};

/***********************Testing functions************************/
double DavinciCalibrator::errorFunc(const cv::Mat &vec_CM0){

    cv::Mat eye_prime = cv::Mat::eye(4,4,CV_64FC1);

    ////compute G_CM0
    cv::Mat G_CM0 = cv::Mat::eye(4,4,CV_64FC1);
    computeSE3(vec_CM0, G_CM0);

    //ROS_INFO_STREAM("G_CM0: " << G_CM0);
    ///start computing the error for G_CM0
    double coefficient = 0.1;  //TODO: HOW MUCH
    double error = 0.0;

    cv::Mat g_C_M0 = G_CM0.clone();

    cv::Mat g_M0_C = g_C_M0.inv();
    int test_size = G_CB.size();

    for (int i = 0; i < test_size; ++i) {
        cv::Mat temp_G_CB = G_CB[i].clone();
        cv::Mat temp_G_M1_M0 = G_M1_M0[i].clone();

        eye_prime = temp_G_CB * g_bm * temp_G_M1_M0 * g_M0_C; //g_CC, G_M1_M0 and G_CB must be consistent
        //ROS_INFO_STREAM("THE supposed eye matrix: " << eye_prime);

        cv::Mat Rot_x = eye_prime.colRange(0,3).rowRange(0,3);   //Rx
        cv::Mat p_x = eye_prime.colRange(3,4).rowRange(0,3);   //Px

        double trace = Rot_x.at<double>(0,0) + Rot_x.at<double>(1,1) + Rot_x.at<double>(2,2);
        double errorTranslation = pow(p_x.at<double>(0, 0), 2) + pow(p_x.at<double>(1, 0), 2) + pow(p_x.at<double>(2, 0), 2);
        double errorRotation = acos((trace -1)/2);  ///ambiguity in +/- eliminated
        errorRotation = pow(errorRotation, 2);

        double temp_error = errorTranslation + coefficient * errorRotation;

        error += temp_error;
    }

    return error;

};


void DavinciCalibrator::validationFunc(const cv::Mat &G_CM0){
    //initialization
    cv::Mat Rot = G_CM0.colRange(0, 3).rowRange(0, 3);   //Rx
    cv::Mat p = G_CM0.colRange(3, 4).rowRange(0, 3);   //Px

    cv::Mat Rvec(3, 1, CV_64FC1);
    cv::Rodrigues(Rot, Rvec);

    cv::Mat vec_CM0(6, 1, CV_64FC1);  ///a new vec for representing G_CM0

    vec_CM0.at<double>(0, 0) = p.at<double>(0, 0);
    vec_CM0.at<double>(1, 0) = p.at<double>(1, 0);
    vec_CM0.at<double>(2, 0) = p.at<double>(2, 0);
    vec_CM0.at<double>(3, 0) = Rvec.at<double>(0, 0);
    vec_CM0.at<double>(4, 0) = Rvec.at<double>(1, 0);
    vec_CM0.at<double>(5, 0) = Rvec.at<double>(2, 0);

    double error = errorFunc(vec_CM0);
    ROS_INFO_STREAM("error is: " << error);
};

void DavinciCalibrator::particleSwarmOptimization(const cv::Mat &G_CM0) {

    int Num = 40000;  //number of particles
    double c1 = 2; //flying weights according to the local best
    double c2 = 2; //flying weights according to the global best
    int MaxIter = 100;  //max iteration
    double w = 0.75;  //speed weight

    //initialization
    cv::Mat Rot = G_CM0.colRange(0, 3).rowRange(0, 3);   //Rx
    cv::Mat p = G_CM0.colRange(3, 4).rowRange(0, 3);   //Px

    cv::Mat Rvec(3, 1, CV_64FC1);
    cv::Rodrigues(Rot, Rvec);

    cv::Mat vec_CM0(6, 1, CV_64FC1);  ///a new vec for representing G_CM0

    vec_CM0.at<double>(0, 0) = p.at<double>(0, 0);
    vec_CM0.at<double>(1, 0) = p.at<double>(1, 0);
    vec_CM0.at<double>(2, 0) = p.at<double>(2, 0);
    vec_CM0.at<double>(3, 0) = Rvec.at<double>(0, 0);
    vec_CM0.at<double>(4, 0) = Rvec.at<double>(1, 0);
    vec_CM0.at<double>(5, 0) = Rvec.at<double>(2, 0);

    std::vector<double> local_errorG_CM0;

    std::vector<cv::Mat> particles;
    std::vector<cv::Mat> local_best_paticles;
    std::vector<cv::Mat> velocities;

    particles.resize(Num);
    local_best_paticles.resize(Num);
    velocities.resize(Num);
    local_errorG_CM0.resize(Num);

    cv::Mat temp_vel(6, 1, CV_64FC1);  ///a new vec for representing G_CM0
    cv::Mat temp_particle(6, 1, CV_64FC1);  ///a new vec for representing G_CM0

    double temp_errorValue;

    cv::Mat final_G_CM0 = cv::Mat::eye(4,4,CV_64FC1); ///final G_CT1

    //initialization for all particles
    for (int i = 0; i < Num; i++) {
        //give random velocity
        double dev_xy = randomNum(-0.05, 0.05);
        double dev_z = randomNum(-0.01, 0.01);
        double dev_theta = randomNum(-0.5, 0.5); //(-pi/2, pi/2)

        ///random velocity
        temp_vel.at<double>(0, 0) = dev_xy;
        temp_vel.at<double>(1, 0) = dev_xy;
        temp_vel.at<double>(2, 0) = dev_z;
        temp_vel.at<double>(3, 0) = dev_theta;
        temp_vel.at<double>(4, 0) = dev_theta;
        temp_vel.at<double>(5, 0) = dev_theta;

        //random particles
        temp_particle.at<double>(0, 0) = vec_CM0.at<double>(0,0) + dev_xy;
        temp_particle.at<double>(1, 0) = vec_CM0.at<double>(1,0) + dev_xy;
        temp_particle.at<double>(2, 0) = vec_CM0.at<double>(2,0) + dev_z;
        temp_particle.at<double>(3, 0) = vec_CM0.at<double>(3,0) + dev_theta;
        temp_particle.at<double>(4, 0) = vec_CM0.at<double>(4,0) + dev_theta;
        temp_particle.at<double>(5, 0) = vec_CM0.at<double>(5,0) + dev_theta;

        ////Opencv is stupid so don't use push_back unless you like to debug for really long time
        velocities[i] = temp_vel.clone();

        temp_errorValue = errorFunc(temp_particle);
        local_errorG_CM0[i] = temp_errorValue; //temporary local best

        particles[i] = temp_particle.clone();
        local_best_paticles[i] = temp_particle.clone();
    }

    ///initialize global best
    cv::Mat global_best(6, 1, CV_64FC1);
    global_best = particles[0].clone();

    double best_value = errorFunc(global_best);
    for (int i = 1; i < Num; i++) {

        if (local_errorG_CM0[i] < best_value){
            global_best = particles[i].clone();
            best_value = errorFunc(global_best);
        }
    }

    ////main iteration
    for (int iter = 0; iter < MaxIter; iter++) {

        double dev_xy = randomNum(-1, 1);
        double dev_z = randomNum(-1, 1);
        double dev_theta = randomNum(-1, 1); //not as much as (-pi/2, pi/2)

        for (int n = 0; n < Num; n++) {
            //update have to use different metric
            velocities[n].at<double>(0,0) = w * velocities[n].at<double>(0,0) + c1 * dev_xy *(local_best_paticles[n].at<double>(0,0) - particles[n].at<double>(0,0)) + c2 * dev_xy * (global_best.at<double>(0,0) - particles[n].at<double>(0,0));
            velocities[n].at<double>(1,0) = w * velocities[n].at<double>(1,0) + c1 * dev_xy *(local_best_paticles[n].at<double>(1,0) - particles[n].at<double>(1,0)) + c2 * dev_xy * (global_best.at<double>(1,0) - particles[n].at<double>(1,0));
            velocities[n].at<double>(2,0) = w * velocities[n].at<double>(2,0) + c1 * dev_z *(local_best_paticles[n].at<double>(2,0) - particles[n].at<double>(2,0)) + c2 * dev_z * (global_best.at<double>(2,0) - particles[n].at<double>(2,0));
            velocities[n].at<double>(3,0) = w * velocities[n].at<double>(3,0) + c1 * dev_theta *(local_best_paticles[n].at<double>(3,0) - particles[n].at<double>(3,0)) + c2 * dev_theta * (global_best.at<double>(3,0) - particles[n].at<double>(3,0));
            velocities[n].at<double>(4,0) = w * velocities[n].at<double>(4,0) + c1 * dev_theta *(local_best_paticles[n].at<double>(4,0) - particles[n].at<double>(4,0)) + c2 * dev_theta * (global_best.at<double>(4,0) - particles[n].at<double>(4,0));
            velocities[n].at<double>(5,0) = w * velocities[n].at<double>(5,0) + c1 * dev_theta *(local_best_paticles[n].at<double>(5,0) - particles[n].at<double>(5,0)) + c2 * dev_theta * (global_best.at<double>(5,0) - particles[n].at<double>(5,0));

            particles[n] = particles[n] + velocities[n];

            ///need to do boundary check
            //boundaryCheck(particles[n]);

            temp_errorValue = errorFunc(particles[n]);  // the temp error for n-th new G_CM0

//            ROS_INFO_STREAM("local_errorG_CM0 " << local_errorG_CM0[n]);
            // ROS_INFO_STREAM("temp_errorValue " << temp_errorValue);

            if (local_errorG_CM0[n] > temp_errorValue) { /// update local best if new error is smaller

                local_errorG_CM0[n] = temp_errorValue;
                local_best_paticles[n] = particles[n].clone();
            }

            best_value = errorFunc(global_best);
            if (best_value > local_errorG_CM0[n]) {  ///if local error is smaller than global best then update
                global_best = local_best_paticles[n].clone();
                ROS_INFO("update global best");
            }
        }

        best_value = errorFunc(global_best);
        ROS_INFO_STREAM("minimal error : " << best_value);
        computeSE3(global_best, final_G_CM0);
        ROS_INFO_STREAM("global best G_CT: " << final_G_CM0);
        ROS_INFO("------------------------");
    }

    computeSE3(global_best, final_G_CM0);
    ROS_INFO_STREAM("global best G_CT: " << final_G_CM0);

};

void DavinciCalibrator::boundaryCheck(cv::Mat &particle){

	///need to do boundary check
	double temp_x = particle.at<double>(0,0);
	double temp_y = particle.at<double>(1,0);
	double temp_z = particle.at<double>(2,0);

	double temp_roll = particle.at<double>(3,0);
	double temp_pitch = particle.at<double>(4,0);
	double temp_yaw = particle.at<double>(5,0);

    /********numbers should be close to the real constrains*******/
	if (temp_x > 0.1 || temp_x < -0.1) {
		particle.at<double>(0,0) = randomNum(-0.08, 0.08);
	}
	if (temp_y > 0.1 || temp_y < -0.1) {
		particle.at<double>(1,0) = randomNum(-0.08, 0.08);
	}
	if (temp_z > 0.19 || temp_z < -0.16) {
		particle.at<double>(2,0) = randomNum(-0.16, 0.18);
	}
	if (temp_roll > 1 || temp_roll < -1) {
		particle.at<double>(3,0) = randomNum(-1, 1);
	}
	if (temp_pitch > 1 || temp_pitch < -1) {
		particle.at<double>(4,0) = randomNum(-1, 1);
	}
	if (temp_yaw > 1 || temp_yaw < -1) {
		particle.at<double>(5,0) = randomNum(-1, 1);
	}

};

void DavinciCalibrator::computeSE3(const cv::Mat &vec_6_1, cv::Mat &outputGeometry){

	outputGeometry = cv::Mat::eye(4,4,CV_64FC1);
	cv::Mat Rotation_CM0(3,3,CV_64FC1);
	cv::Mat Rvec_CM0(3,1,CV_64FC1);
	cv::Mat Tvec_CM0(3,1,CV_64FC1);

	Tvec_CM0.at<double>(0,0) = vec_6_1.at<double>(0,0);
	Tvec_CM0.at<double>(1,0) = vec_6_1.at<double>(1,0);
	Tvec_CM0.at<double>(2,0) = vec_6_1.at<double>(2,0);

	Rvec_CM0.at<double>(0,0) = vec_6_1.at<double>(3,0);
	Rvec_CM0.at<double>(1,0) = vec_6_1.at<double>(4,0);
	Rvec_CM0.at<double>(2,0) = vec_6_1.at<double>(5,0);

	cv::Rodrigues(Rvec_CM0, Rotation_CM0 );  //get rotation mat

	Rotation_CM0.copyTo(outputGeometry.colRange(0,3).rowRange(0,3));
	Tvec_CM0.copyTo(outputGeometry.colRange(3,4).rowRange(0,3));

};

double DavinciCalibrator::randomNumber(double stdev, double mean) {

    boost::normal_distribution<> nd(mean, stdev);
    boost::variate_generator<boost::mt19937 &, boost::normal_distribution<> > var_nor(rng, nd);
    double d = var_nor();

    return d;

};

double DavinciCalibrator::randomNum(double min, double max) {

    /// srand((unsigned) time( NULL));  //do this in main or constructor
    int N = 999;

    double randN = rand() % (N + 1) / (double) (N + 1);  // a rand number frm 0 to 1
    double res = randN * (max - min) + min;

    return res;
};

void DavinciCalibrator::stereoCameraCalibration(){

    cv::Mat left_cam_pose, right_cam_pose;
    left_cam_pose = cv::Mat::eye(4, 4, CV_64FC1);
    right_cam_pose = cv::Mat::eye(4, 4, CV_64FC1);

    cv::Mat G_CM_l = cv::Mat::eye(4,4,CV_64FC1); ///g of maker relative to left camera
    cv::Mat G_CM_r = cv::Mat::eye(4,4,CV_64FC1); ///g of maker relative to right camera

    // make sure camera information is ready.
        if(freshCameraInfo)
        {
            //retrive camera info
            freshMakers = true;
            if(freshMakers){  //get fresh maker poses

                if (freshLeftCorner && freshRightCorner )
                {
                    setBoardCoord();  ///set the 3d coordinates for the chessboard or circle board

                    if (boardMatch) //when get the board corners match the 3d board set up
                    {
                        computeCameraPose(left_corner_coordinates, camera_mat_l, left_cam_pose);  ///get camera poses
                        computeCameraPose(right_corner_coordinates, camera_mat_r, right_cam_pose);

                        ROS_INFO_STREAM("G_CB_l= " << left_cam_pose );
                        ROS_INFO_STREAM("G_CB_r= " << right_cam_pose );

                        G_CM_l = left_cam_pose * g_bm * g_M1_M0;
                        G_CM_r = right_cam_pose * g_bm * g_M1_M0;

                        ROS_INFO_STREAM("G_CM1_l= " << G_CM_l );
                        ROS_INFO_STREAM("G_CM1_r= " << G_CM_r );

                        // cv::Mat test_point = (cv::Mat_<double>(4,1) << 0, 0.015, 0, 1);

                        // cv::Mat camera_point = left_cam_pose * test_point;
                        // ROS_INFO_STREAM("camera_point = " << camera_point );

                        /***testing using left camera now*/
                        //testCamToBoard(G_CM_l, calibrator.marker_poses, calibrator.g_bm, left_cam_pose);
                        //errorFunc(calibrator.g_M1_M0, left_cam_pose, calibrator.g_bm, G_CM_l); //this will be for the left camera
                        //testCamToBoard3();

                    }

                }else{
                ROS_INFO(" No corner information received! ");
                }

            }else{
            ROS_INFO(" No marker detected! ");
            }

        }

        freshCameraInfo = false;
        freshLeftCorner = false;
        freshRightCorner = false;
        boardMatch = false;
        freshMakers = false;
};

void DavinciCalibrator::stupidGenerator() {

    G_M1_M0.resize(20);
    G_CB.resize(20);

    /************LEFT data set*************/
//    G_CB[0] = (cv::Mat_<double>(4,4) << 0.9945101621690301, -0.06520176114622502, 0.08184294524245891, -0.06545206211814748,
//    0.05795424818855873, 0.9944326271219892, 0.08800599543302623, -0.06880656661674799,
//    -0.08712544094251945, -0.08277971042888177, 0.9927520723130635, 0.3190269258453948,
//    0, 0, 0, 1);
//
//    G_CB[1] = (cv::Mat_<double>(4,4) << 0.9952636341199845, -0.06686986247526848, 0.07056004599502635, -0.03738885665021956,
//    0.06173795377832765, 0.9954510811749978, 0.07256424774496763, -0.07188578585787549,
//    -0.07509143534083387, -0.06786432405960176, 0.9948646691175572, 0.3174213292867002,
//    0, 0, 0, 1);
//
//    G_CB[2] = (cv::Mat_<double>(4,4) << 0.9956921942779411, -0.07883378352238918, 0.04880869624896729, -0.01470701163615565,
//    0.07495072731700192, 0.9942223217934165, 0.07683985503864871, -0.07255505080625313,
//    -0.05458427180636648, -0.07285059658817633, 0.9958480545987503, 0.3157644649744919,
//    0, 0, 0, 1);
//
//    G_CB[3] = (cv::Mat_<double>(4,4) << 0.9926626020940931, -0.1171861274073707, 0.02980553551016454, 0.009333312827856155,
//    0.1141535335499328, 0.9895055872225791, 0.08858703987245144, -0.05583473699049292,
//    -0.03987391605860376, -0.08453463431376752, 0.9956224015255929, 0.3120524048294543,
//    0, 0, 0, 1);
//
//    G_CB[4] = (cv::Mat_<double>(4,4) << 0.9947454715218867, -0.093943839882112, 0.04069400244389156, 0.01084727670307426,
//    0.09037414354123571, 0.9925168499736479, 0.0821146558025764, -0.03660403657328029,
//    -0.04810364919512312, -0.07800549638705964, 0.9957917359907749, 0.3105721030918792,
//    0, 0, 0, 1);
//
//    G_CB[5] = (cv::Mat_<double>(4,4) << 0.9959988092343806, -0.08644315206737202, 0.02267054177458526, 0.0115008252620545,
//    0.08443253461324376, 0.993352480194978, 0.07824319260657329, -0.01484376735305089,
//    -0.02928342709587481, -0.07601599536375962, 0.9966765018532229, 0.3069196875311663,
//    0, 0, 0, 1);
//
//
//    G_CB[6] = (cv::Mat_<double>(4,4) << 0.9936382156885383, -0.09774655967323718, 0.05593484062142418, -0.003315580893180654,
//    0.09182139867610083, 0.9907249104545549, 0.1001647769926076, -0.01564619961794488,
//    -0.06520680231741466, -0.09439153498519105, 0.9933973580872244, 0.3100325937740121,
//    0, 0, 0, 1);
//    G_CB[7] = (cv::Mat_<double>(4,4) << 0.9920452104932053, -0.1115256908506422, 0.05838082405875267, -0.02920385118620493,
//    0.1045860757007063, 0.988323051613915, 0.1108119958221351, -0.01485649134979489,
//    -0.07005749857808358, -0.1038246884360639, 0.9921251841194921, 0.3120238431890075,
//    0, 0, 0, 1);
//    G_CB[8] = (cv::Mat_<double>(4,4) <<0.9968291823730863, -0.05811951098163871, 0.05434798627970906, -0.0392822250858952,
//    0.0528178174897395, 0.9941359100536814, 0.09436138244726611, -0.024635199414807,
//    -0.05951352220315007, -0.09119163769224581, 0.9940534321101564, 0.3130772226916309,
//    0, 0, 0, 1);
//    G_CB[9] = (cv::Mat_<double>(4,4) <<0.9974060944143051, -0.04444137154473495, 0.05662196853187925, -0.04691659428272903,
//    0.03950003422707776, 0.9955476604553299, 0.08558389485165288, -0.03901831716673438,
//    -0.06017333397163554, -0.08312532861373789, 0.9947207395151647, 0.3146487853078859,
//    0, 0, 0, 1);
//    G_CB[10] = (cv::Mat_<double>(4,4) <<0.9904578933505407, -0.1271318856024233, 0.05320380778460689, 0.01005058778925368,
//    0.1218240329216719, 0.9881557817586336, 0.09331160688619092, -0.02969700798509657,
//    -0.06443653080596781, -0.08593971515054022, 0.9942144129198367, 0.3241940106141761,
//    0, 0, 0, 1);
//
//    G_CB[11] = (cv::Mat_<double>(4,4) <<0.9957813389859875, -0.05386331808576333, 0.07428504487489901, -0.02971786788369081,
//    0.04673418669243124, 0.9944198985777811, 0.09457791024733202, -0.02531444808630066,
//    -0.07896480685388182, -0.09070726694894567, 0.9927420364833867, 0.3272818750124114,
//    0, 0, 0, 1);
//    G_CB[12] = (cv::Mat_<double>(4,4) <<0.9957238452424605, -0.05698662041169598, 0.07270865911032871, -0.02068233871760668,
//    0.05111696889399171, 0.9954685956375734, 0.08018310664004044, -0.0534721306698143,
//    -0.07694855103677649, -0.07612358500104083, 0.9941248011693149, 0.3290888901115834,
//    0, 0, 0, 1);
//    G_CB[13] = (cv::Mat_<double>(4,4) <<0.9950143495145751, -0.09227907810447707, 0.03782877217508725, 0.01146628817067382,
//    0.08854945575673627, 0.9919422527560335, 0.0906066282479982, -0.0546652910331988,
//    -0.04588505361523117, -0.08680517807984131, 0.9951680375259518, 0.3263022633673045,
//    0, 0, 0, 1);
//    G_CB[14] = (cv::Mat_<double>(4,4) <<0.9940823465701955, -0.07453933564628683, 0.07902009667739, -0.06117546508096174,
//    0.06634459251203244, 0.9926010367999153, 0.1016935434929162, -0.06219984703410534,
//    -0.08601559906148558, -0.09584920023215354, 0.9916724497196392, 0.2839772591701671,
//    0, 0, 0, 1);
//    G_CB[15] = (cv::Mat_<double>(4,4) <<0.9964012612946954, -0.06194516694015847, 0.05785605225986357, -0.03464755078487977,
//    0.0570565136518127, 0.9949453206879564, 0.08263391005537803, -0.06051889816049265,
//    -0.06268237982272752, -0.07903546756928621, 0.9948991477159198, 0.2815938132218793,
//    0, 0, 0, 1);
//    G_CB[16] = (cv::Mat_<double>(4,4) <<0.9985536915828295, -0.05044307817465446, 0.01860163677123821, -0.005444698126401466,
//    0.04842665891635223, 0.994168576384002, 0.09635196125043921, -0.06026708796551472,
//    -0.02335345226091136, -0.0953117914786652, 0.9951734917453463, 0.2790944057093889,
//    0, 0, 0, 1);
//    G_CB[17] = (cv::Mat_<double>(4,4) <<0.9964281458078738, -0.08201510457188144, 0.020111510732566, 0.00334377638025082,
//    0.07981034095039805, 0.9924580166284324, 0.093045121889003, -0.03838546770439955,
//    -0.02759093545467392, -0.09110767175173319, 0.9954587547601932, 0.2764363671155285,
//    0, 0, 0, 1);
//    G_CB[18] = (cv::Mat_<double>(4,4) <<0.9985064776731739, -0.03038614727391039, 0.04540370137510432, -0.01864679203920693,
//    0.0258191256638661, 0.9948522971318426, 0.09799122226735101, -0.01736895790901704,
//    -0.04814755232267693, -0.09667258631765328, 0.9941510067690879, 0.2758917204333182,
//    0, 0, 0, 1);
//    G_CB[19] = (cv::Mat_<double>(4,4) <<0.9985169621775715, -0.03644069666789337, 0.04044690185951182, -0.04506006392476346,
//    0.03302871925408567, 0.9960817704229571, 0.08203785915967134, -0.03532922002178894,
//    -0.04327793835326709, -0.08058028454545109, 0.9958081330228554, 0.2785671015679631,
//    0, 0, 0, 1);

    /************RIGHT data set*************/
//
//    G_CB[0] = (cv::Mat_<double>(4,4) <<0.995496434238292, -0.05857841903212668, 0.07453467812060703, -0.06054642962987073,
//    0.05072279821918435, 0.9933619734680805, 0.1032433407461051, -0.06781679133205051,
//    -0.08008774662619637, -0.09899777013295318, 0.9918595638239524, 0.3198075419692131,
//    0, 0, 0, 1);
//
//    G_CB[1] = (cv::Mat_<double>(4,4) <<0.9955782180385637, -0.06494075166288554, 0.06787275256402565, -0.03227744534808497,
//    0.05758910641770645, 0.9927859384144999, 0.1051645154434274, -0.07099364102981444,
//    -0.0742125770282107, -0.1007907697157841, 0.9921359353188108, 0.3178754427182175,
//    0, 0, 0, 1);
//
//    G_CB[2] = (cv::Mat_<double>(4,4) <<0.9946137942218002, -0.08107491983576189, 0.0645775326049176, -0.009432696829099306,
//    0.07427174258681264, 0.9920524585474523, 0.1015658788327737, -0.07180188541633672,
//    -0.07229874547205263, -0.09622253823081274, 0.9927306354392349, 0.3161410588279756,
//    0, 0, 0, 1);
//
//    G_CB[3] = (cv::Mat_<double>(4,4) <<0.9902625729547532, -0.1222679694406885, 0.06656260401969716, 0.01485494851579328,
//    0.1154061231282341, 0.9884052961102482, 0.09867318463353406, -0.05532332613397408,
//    -0.07785540025934631, -0.09003062962160979, 0.9928912439841503, 0.313226622792446,
//    0, 0, 0, 1);
//
//    G_CB[4] = (cv::Mat_<double>(4,4) <<0.992819773935992, -0.09858058184023974, 0.06775518700236496, 0.0163209336991663,
//    0.09272249277109768, 0.9920778172993205, 0.0847593284360466, -0.03618358369061951,
//    -0.07557404194562703, -0.07786830745981109, 0.9940951115850761, 0.310978465320964,
//    0, 0, 0, 1);
//
//    G_CB[5] = (cv::Mat_<double>(4,4) <<0.9940261408102384, -0.09044802116307114, 0.06108344173005318, 0.01699452303725674,
//    0.08538738518576554, 0.9930565833848807, 0.08091734453802341, -0.01438538587280618,
//    -0.0679781276370707, -0.07521820034826754, 0.9948473232106151, 0.3084221034257548,
//    0, 0, 0, 1);
//
//    G_CB[6] = (cv::Mat_<double>(4,4) <<0.9931057805391067, -0.09740396282295657, 0.06521791691089002, 0.001952270485563862,
//    0.09234934419154561, 0.9927844604287549, 0.07648930486399787, -0.01525676629284263,
//    -0.07219769585799191, -0.06993913895359492, 0.9949351785695522, 0.3093891106908854,
//    0, 0, 0, 1);
//
//    G_CB[7] = (cv::Mat_<double>(4,4) <<0.9917461003401649, -0.1106853445429556, 0.06471805747612365, -0.02396071250044676,
//    0.1049843393422712, 0.9907742813502284, 0.08570071124445454, -0.0144878671987415,
//    -0.07360679963795801, -0.0781989636654203, 0.9942167576181321, 0.3115319072169026,
//    0, 0, 0, 1);
//
//    G_CB[8] = (cv::Mat_<double>(4,4) <<0.9963243293030126, -0.05820996146258394, 0.06284450035943105, -0.03400294070637883,
//    0.05227161574310148, 0.9943562021381616, 0.09232237787761535, -0.02417058719954376,
//    -0.06786390076106429, -0.08869804764421901, 0.9937440049215875, 0.3132219608491079,
//    0, 0, 0, 1);
//
//    G_CB[9] = (cv::Mat_<double>(4,4) <<0.9966835183442893, -0.04425191808677513, 0.06829152221535353, -0.04164219621991967,
//    0.03747239547817387, 0.9945253546574808, 0.09754557150654926, -0.03838508422684789,
//    -0.07223422899135917, -0.09466301647979174, 0.9928852549378355, 0.3153363224029931,
//    0, 0, 0, 1);
//
//    G_CB[10] = (cv::Mat_<double>(4,4) <<0.9887834905177818, -0.1299497187107605, 0.07362254740543789, 0.01550662170675937,
//    0.1232534735300347, 0.9883595374811611, 0.0891852338271657, -0.02927235089405733,
//    -0.08435514295081703, -0.07911065210841958, 0.9932903475725151, 0.3244828060521676,
//    0, 0, 0, 1);
//
//    G_CB[11] = (cv::Mat_<double>(4,4) <<0.9960852994515788, -0.05363544712775654, 0.07026603039779804, -0.02447306380754934,
//    0.04658522600802333, 0.994056486693044, 0.09839470504688469, -0.02486013650208608,
//    -0.07512584731129514, -0.09473616033430785, 0.9926636726459129, 0.3266865340114512,
//    0, 0, 0, 1);
//
//    G_CB[12] = (cv::Mat_<double>(4,4) <<0.995922054525889, -0.05724313589113886, 0.06973151871486005, -0.01546181424837923,
//    0.04988064569946103, 0.9934196509502619, 0.1030985852980759, -0.05284079929457763,
//    -0.07517434731034986, -0.09919990170967112, 0.9922238643608883, 0.3291087676287172,
//    0, 0, 0, 1);
//
//    G_CB[13] = (cv::Mat_<double>(4,4) <<0.9929349038768523, -0.09716170641140921, 0.06811666073941942, 0.01690794015171502,
//    0.08979579733253042, 0.9905225482967887, 0.1039316895708474, -0.05412397665743138,
//    -0.07756926868600196, -0.09708081233106254, 0.9922491241783284, 0.3270171300863313,
//    0, 0, 0, 1);
//
//    G_CB[14] = (cv::Mat_<double>(4,4) <<0.9951124634501534, -0.06904824686496901, 0.07059408396630371, -0.05617413199144041,
//    0.06094664571732548, 0.9919388619221031, 0.111098157430669, -0.06140386444983666,
//    -0.0776961483084847, -0.1062526885003725, 0.9912989835183272, 0.2845116119921003,
//    0, 0, 0, 1);
//
//    G_CB[15] = (cv::Mat_<double>(4,4) <<0.9959805133758636, -0.06175882686115377, 0.064874218918498, -0.02939320810591345,
//    0.05442491089896847, 0.9925179921296255, 0.1092975954566976, -0.05979863064673024,
//    -0.07113892077611604, -0.1053275016494297, 0.9918897979851888, 0.282163702590541,
//    0, 0, 0, 1);
//
//    G_CB[16] = (cv::Mat_<double>(4,4) <<0.9968055922937394, -0.05582174272112327, 0.05711868530965979, 0.000012,
//    0.04966285330923854, 0.993327905291249, 0.1040830128833922, -0.05980821899295077,
//    -0.0625476791984505, -0.1009138524151965, 0.9929270779959682, 0.2801870533983926,
//    0, 0, 0, 1);
//
//    G_CB[17] = (cv::Mat_<double>(4,4) <<0.9945201796493759, -0.08662237447599563, 0.05853355029650539, 0.008860703865764388,
//    0.08095542737451636, 0.9923610911868654, 0.09308965289989618, -0.03800339025912833,
//    -0.0661500646166166, -0.08784092974550148, 0.9939356820301117, 0.2774576602802562,
//    0, 0, 0, 1);
//
//    G_CB[18] = (cv::Mat_<double>(4,4) <<0.9979709646051578, -0.03026546729623399, 0.05601745526522522, -0.01332260084141496,
//    0.02583089259075046, 0.9965981404503739, 0.07826181341383508, -0.01701846351345961,
//    -0.05819552210450586, -0.07665603655419066, 0.995357791583904, 0.2757451442196044,
//    0, 0, 0, 1);
//
//    G_CB[19] = (cv::Mat_<double>(4,4) <<0.9975028405767539, -0.03595519330171208, 0.06078903779419228, -0.0397787737669795,
//    0.0304812240110962, 0.9955924386945842, 0.08869380472702564, -0.03474841223455803,
//    -0.06371010927704167, -0.0866193978783442, 0.9942022439559772, 0.2793948653547106,
//    0, 0, 0, 1);


    /*******validation data sets: ******/
    /////left
    G_CB[0] = (cv::Mat_<double>(4,4) << 0.9876479537173266, -0.1467740725434155, 0.05485336039840982, -0.01411675979539273,
    0.1419346500922012, 0.9863371540545399, 0.08362758895720578, -0.03178483945554118,
    -0.06637826919393136, -0.07480902459336831, 0.9949861985063948, 0.2770053954348569,
    0, 0, 0, 1);

    G_CB[1] = (cv::Mat_<double>(4,4) << 0.9835960588588012, -0.1730052870684933, 0.05106822538312755, -0.01601912415706762,
    0.1677118962775803, 0.9813208312852902, 0.09424513744764206, -0.02135797554030977,
    -0.06641922044417423, -0.08413439684157155, 0.9942383468885583, 0.276321056452304,
    0, 0, 0, 1);

    G_CB[2] = (cv::Mat_<double>(4,4) << 0.9859411515666956, -0.1607687051302641, 0.04553536096350167, -0.02876594942003822,
    0.1551305558659768, 0.9819674193289762, 0.108048590981574, -0.02211137700837991,
    -0.06208507295680036, -0.09946562635970604, 0.9931022268068983, 0.2772074463721979,
    0, 0, 0, 1);

    G_CB[3] = (cv::Mat_<double>(4,4) << 0.9872625150093239, -0.1481851012650529, 0.05791288475400301, -0.01606318165158984,
    0.1431713365270368, 0.9862265028922395, 0.08282060969351565, -0.04953147045581347,
    -0.0693880022376087, -0.07347421830825901, 0.9948803166157545, 0.2795627208236053,
    0, 0, 0, 1);

    G_CB[4] = (cv::Mat_<double>(4,4) << 0.9845351515516561, -0.1739131939117044, 0.02108877290379221, 0.0006637702035517105,
    0.1710759144520775, 0.9803649859748962, 0.09806898474455183, -0.06317769265084305,
    -0.03773018491265721, -0.09294458164880595, 0.9949561487264578, 0.2795028858892252,
    0, 0, 0, 1);

    G_CB[5] = (cv::Mat_<double>(4,4) << 0.9805690363128462, -0.190617829411844, 0.0463595527891864, -0.02015084299067183,
    0.1849461699309642, 0.9770638884700488, 0.1055512769967997, -0.01925948879809748,
    -0.06541620022871318, -0.09492629223826977, 0.9933326330034323, 0.2763938179548631,
    0, 0, 0, 1);

    G_CB[6] = (cv::Mat_<double>(4,4) << 0.9830903394094023, -0.1767302862754255, 0.04795613071248038, -0.01074793270327752,
    0.1720390220212735, 0.9810823082673928, 0.08876981078434418, -0.04778574722189581,
    -0.06273722548750553, -0.07901841758558986, 0.9948970450355148, 0.2648870153798558,
    0, 0, 0, 1);
    G_CB[7] = (cv::Mat_<double>(4,4) << 0.9744776208786166, -0.2172689119377452, 0.05645871334116211, -0.01805314766056069,
    0.2111858685759242, 0.9725596732875535, 0.09761255456466417, -0.02554229884673257,
    -0.07611764132304462, -0.08319796752442565, 0.9936217604698585, 0.263526306183511,
    0, 0, 0, 1);

    G_CB[8] = (cv::Mat_<double>(4,4) <<0.9812391282900522, -0.1870520623278341, 0.04670437978908056, -0.03754417100651101,
    0.1812544544037815, 0.9775810526048382, 0.1071546002129831, -0.02495009828846749,
    -0.06570080571321853, -0.09667890962831982, 0.9931447993931767, 0.2643794144958321,
    0, 0, 0, 1);

    G_CB[9] = (cv::Mat_<double>(4,4) <<0.9822619904471458, -0.1791362484505688, 0.05542189651994388, -0.05282646469940867,
    0.1742604330601114, 0.9811974578019659, 0.08297499787684104, -0.04402660522943488,
    -0.06924365380678377, -0.07184534288327549, 0.9950093281539982, 0.2665382787205602,
    0, 0, 0, 1);

    G_CB[10] = (cv::Mat_<double>(4,4) <<0.9965056352331434, -0.06850458418992976, 0.04778745539944849, -0.01160907472051215,
    0.07990617672319543, 0.9484760035664582, -0.3066076867596136, 0.009307171537928106,
    -0.02432122262697572, 0.3093548005180535, 0.9506356218480162, 0.3163293700921541,
    0, 0, 0, 1);

    G_CB[11] = (cv::Mat_<double>(4,4) <<0.9943595003587581, -0.0621726371969679, 0.08592873343797812, -0.03407731802386991,
    0.09168694891378439, 0.9111519833479108, -0.4017406708811662, -0.02143147243931736,
    -0.05331685892062647, 0.4073531961641428, 0.9117130503232338, 0.313135475813022,
    0, 0, 0, 1);
    G_CB[12] = (cv::Mat_<double>(4,4) <<0.9957556971413279, -0.0268001272046643, 0.08804740082706408, -0.05273480536737151,
    0.06678840787653528, 0.8686328795192499, -0.490934037515646, -0.03402847812942015,
    -0.06332377266007949, 0.4947309104957057, 0.8667360763324581, 0.3041748382060835,
    0, 0, 0, 1);
    G_CB[13] = (cv::Mat_<double>(4,4) <<0.9949033336004083, -0.04960156220100893, 0.08778975918644638, -0.06792653011376464,
    0.09005038815093144, 0.8288042746807316, -0.5522448749104089, -0.04475171824422511,
    -0.04536831917386175, 0.5573357689025226, 0.8290467757112217, 0.2957030116838216,
    0, 0, 0, 1);
    G_CB[14] = (cv::Mat_<double>(4,4) <<0.9967508726970544, 0.006706715575343306, 0.08026654187051996, -0.08751631727472947,
    0.02444846101003607, 0.924320401829508, -0.3808333855060209, -0.02641116563143198,
    -0.07674614343340799, 0.3815584027746318, 0.9211531983010049, 0.3168853374748226,
    0, 0, 0, 1);
    G_CB[15] = (cv::Mat_<double>(4,4) <<0.9963566891873394, 0.004244501740019464, 0.08517823734160317, -0.02088604339771236,
    0.02937272858144339, 0.9205769452166441, -0.3894551716838186, -0.02199210950611832,
    -0.08006616468474288, 0.390538182692257, 0.9170983246806772, 0.3128396909497663,
    0, 0, 0, 1);
    G_CB[16] = (cv::Mat_<double>(4,4) <<0.9964192838618834, 0.001735753807972522, 0.08453163849642008, -0.003476916364813078,
    0.03255944699994669, 0.9148056133283737, -0.402579895466671, -0.02076924450444877,
    -0.07802879698693939, 0.4038906745415351, 0.9114734389213736, 0.3108113508386073,
    0, 0, 0, 1);
    G_CB[17] = (cv::Mat_<double>(4,4) <<0.9965815528165287, 0.00151367884284815, 0.08260095254993836, -0.005619081610933568,
    0.02326766241542768, 0.9542163825296798, -0.2982108502346847, 0.001337015958663715,
    -0.07927057759041591, 0.2991133632727441, 0.9509192244560782, 0.3219783102495691,
    0, 0, 0, 1);
    G_CB[18] = (cv::Mat_<double>(4,4) <<0.9963463054916291, -0.01531397862875108, 0.0840209592407732, -0.008410041908196879,
    0.04146036777204642, 0.9468209419001438, -0.3190785826148965, -0.001004040683396261,
    -0.07466644117264577, 0.3213963068205402, 0.9439964706103139, 0.3205326822170484,
    0, 0, 0, 1);
    G_CB[19] = (cv::Mat_<double>(4,4) <<0.9960033187530597, -0.001338725121529868, 0.08930619714185536, -0.04098000856682916,
    0.05853407771218549, 0.7650174661919771, -0.641343931262775, -0.02709602722337731,
    -0.06746221742038117, 0.6440081398835392, 0.7620382962715646, 0.2840273945842327,
    0, 0, 0, 1);


    /////RIGHT

    G_CB[0] = (cv::Mat_<double>(4,4) <<0.9872444454006627, -0.1475675002400913, 0.0597682013986422, -0.008765165787232666,
    0.1423492860327815, 0.98626486358453, 0.08377529244679313, -0.03139327334190962,
    -0.07130978748738043, -0.07419873133335242, 0.9946906365684889, 0.2770876940117861,
    0, 0, 0, 1);

    G_CB[1] = (cv::Mat_<double>(4,4) <<0.9833312981860387, -0.1726697509882459, 0.0569624007694292, -0.01071874262657473,
    0.1682463567564216, 0.9828867082468541, 0.0750125468828691, -0.02100872607840718,
    -0.06893998437736143, -0.06417846870501714, 0.995554319315992, 0.2758515923833693,
    0, 0, 0, 1);

    G_CB[2] = (cv::Mat_<double>(4,4) <<0.9852984266136027, -0.1602433334016601, 0.05923752706759025, -0.02350302500774676,
    0.1553187277128214, 0.9846147608115566, 0.08006163634145594, -0.02177394474010001,
    -0.07115548702966987, -0.06968390698235505, 0.9950281653165628, 0.2769845912465206,
    0, 0, 0, 1);

    G_CB[3] = (cv::Mat_<double>(4,4) <<0.9869832348746537, -0.1486677226026342, 0.06133516391524892, -0.01073378434444804,
    0.1421832538713364, 0.984857633739223, 0.09919356624367226, -0.04900257279822738,
    -0.07515328598885967, -0.08918155370773106, 0.9931760337842188, 0.2798014184632112,
    0, 0, 0, 1);

    G_CB[4] = (cv::Mat_<double>(4,4) <<0.9824059583683638, -0.1780439700407627, 0.0563815368224445, 0.006110063774844914,
    0.171598225643477, 0.9796901756455518, 0.1037362458333543, -0.06265443374252082,
    -0.07370605075804175, -0.09223613432766882, 0.9930054952546543, 0.2800475436555618,
    0, 0, 0, 1);

    G_CB[5] = (cv::Mat_<double>(4,4) <<0.9801723960133961, -0.1901373079524971, 0.05576628208815235, -0.01487587383317923,
    0.1856156934262307, 0.979568221979158, 0.07741390600202611, -0.01897001500635036,
    -0.06934614947679246, -0.06552787661116657, 0.9954381994576932, 0.2758983366696476,
    0, 0, 0, 1);

    G_CB[6] = (cv::Mat_<double>(4,4) <<0.982232631757759, -0.1777931558353017, 0.06007204714633033, -0.005397125727985077,
    0.1721672776300633, 0.9810778316315968, 0.08857040586164776, -0.04735582852706418,
    -0.07468256572774865, -0.07665430202649064, 0.9942568241441203, 0.26487284918388,
    0, 0, 0, 1);

    G_CB[7] = (cv::Mat_<double>(4,4) <<0.974230594246166, -0.2171524793264289, 0.06098811324468989, -0.01277062119517245,
    0.2119520257587136, 0.9738558395250849, 0.08173825664687034, -0.02521354421772731,
    -0.07714329531164626, -0.06670535619631229, 0.9947860611423861, 0.2630985419913023,
    0, 0, 0, 1);

    G_CB[8] = (cv::Mat_<double>(4,4) <<0.9807271072912356, -0.1860787509256695, 0.05957381537309765, -0.03226041305463353,
    0.1808788864339526, 0.9799753110155549, 0.08325393829951631, -0.02459802005156253,
    -0.07387265709705136, -0.07087374869377987, 0.9947460692455701, 0.2643066129776077,
    0, 0, 0, 1);

    G_CB[9] = (cv::Mat_<double>(4,4) <<0.981984861920791, -0.1768907128582259, 0.06644852641642042, -0.04763798155363876,
    0.1700976097911069, 0.9806561359817343, 0.09685218688665018, -0.04337816258293534,
    -0.08229540753746468, -0.08380464584903952, 0.9930781677352291, 0.267728605998007,
    0, 0, 0, 1);

    G_CB[10] = (cv::Mat_<double>(4,4) <<0.996403928891634, -0.07136302968193803, 0.04567853417011569, -0.006307276449778372,
    0.08212007604100191, 0.9461368936601051, -0.313179296196647, 0.00971643727750947,
    -0.02086872301641044, 0.3158042058773473, 0.9485948555362518, 0.3161769157349614,
    0, 0, 0, 1);

    G_CB[11] = (cv::Mat_<double>(4,4) <<0.9943786405353304, -0.0616944612711577, 0.08605180239582255, -0.0288359496499354,
    0.09147961825893199, 0.9098110524238655, -0.404815177989386, -0.02094751590823833,
    -0.05331602658027085, 0.4104115523908163, 0.9103404632190376, 0.3122533772319052,
    0, 0, 0, 1);

    G_CB[12] = (cv::Mat_<double>(4,4) <<0.9959738771649813, -0.02325472450497648, 0.08657513380383722, -0.0475895040975372,
    0.06310432850306259, 0.8678173756521986, -0.492859864708325, -0.03330346912827684,
    -0.06367008504103189, 0.4963388160363126, 0.8657909100738695, 0.3039149162274624,
    0, 0, 0, 1);

    G_CB[13] = (cv::Mat_<double>(4,4) <<0.9957481644396893, -0.0472515456709675, 0.07907518224255955, -0.06271711661949735,
    0.08344393535476145, 0.8263121153240379, -0.5569967663471882, -0.04373832296196205,
    -0.03902182296484781, 0.5612268520842694, 0.8267416270105684, 0.2954495333622854,
    0, 0, 0, 1);

    G_CB[14] = (cv::Mat_<double>(4,4) <<0.9967521341620926, 0.0110169379911451, 0.07977349259379933, -0.08244860783043337,
    0.02034389474354554, 0.9240201425347764, -0.3818021767049988, -0.02556034516071811,
    -0.07791860490266257, 0.3821850379950933, 0.9207949216534114, 0.3183595149787352,
    0, 0, 0, 1);

    G_CB[15] = (cv::Mat_<double>(4,4) <<0.9966579291719622, 0.00395955632938861, 0.08159224308922082, -0.01562198784119726,
    0.02873777356387279, 0.9179804080205832, -0.3955832540198779, -0.02141496329219869,
    -0.07646641477961209, 0.3966059661730243, 0.9147986636450199, 0.3117593370999868,
    0, 0, 0, 1);

    G_CB[16] = (cv::Mat_<double>(4,4) <<0.9966750445141972, -0.001232299999444506, 0.08146985380700336, 0.001880297807487855,
    0.03413919227464677, 0.9142014975194765, -0.4038194367337724, -0.02037541850959667,
    -0.07398223566139259, 0.4052580700860353, 0.9112038879617911, 0.3100204959528546,
    0, 0, 0, 1);

    G_CB[17] = (cv::Mat_<double>(4,4) <<0.9968973399411374, -0.001407794070450205, 0.07870013808208697, -0.0003548503010941503,
    0.02585482163911534, 0.9502196381136936, -0.3105063083756765, 0.001717998972358425,
    -0.07434528778808973, 0.3115776908877406, 0.9473078278599674, 0.3204930432848446,
    0, 0, 0, 1);

    G_CB[18] = (cv::Mat_<double>(4,4) <<0.9966826108472449, -0.01780826186690156, 0.07941435036565683, -0.003075290326205674,
    0.04262860819159155, 0.9454396177970849, -0.3229964873854669, -0.0005910591116293147,
    -0.06932946702786064, 0.3253103055683637, 0.9430623680821172, 0.320121273826414,
    0, 0, 0, 1);

    G_CB[19] = (cv::Mat_<double>(4,4) <<0.9963771695108025, 0.0001875175229141303, 0.08504411158228446, -0.03573529428802688,
    0.05476676170431709, 0.7636264513518166, -0.6433313645457168, -0.0264496816149352,
    -0.06506256903984038, 0.6456582746569659, 0.7608496924340088, 0.2834310646577199,
    0, 0, 0, 1);


    /*******/
//    G_M1_M0[0] = (cv::Mat_<double>(4,4) << -0.09921298456929659, 0.03414504075592249, -0.9944802159342482, 0.4830624743877725,
//    -0.1611347907074084, 0.9856692046562827, 0.04991791477947267, -0.1825744220379382,
//    0.9819329727209146, 0.1651978667659706, -0.09228922959609293, 0.1246287191875777,
//    0, 0, 0, 1);
//
//    G_M1_M0[1] = (cv::Mat_<double>(4,4) << -0.09877676304943189, 0.03332435572524371, -0.9945514759915512, 0.4832793767870248,
//    -0.1667455436575609, 0.984753796156614, 0.04955688272563323, -0.1547309100885895,
//    0.981039792644427, 0.1707320950220993, -0.09171410457254685, 0.1288516222448447,
//    0, 0, 0, 1);
//
//    G_M1_M0[2] = (cv::Mat_<double>(4,4) << -0.09911069196247102, 0.03236275163014846, -0.9945500103291166, 0.4834434974012267,
//    -0.1819730854540794, 0.9820268999799517, 0.05008955865335536, -0.1337057653452687,
//    0.9782958994644941, 0.1859457448362046, -0.09144021581485097, 0.1303703617893309,
//    0, 0, 0, 1);
//
//    G_M1_M0[3] = (cv::Mat_<double>(4,4) << -0.1003014877508479, 0.03165808438855278, -0.9944533057151622, 0.4833912094267507,
//    -0.2222230562743019, 0.973532097873123, 0.05340568949900699, -0.1128916987254248,
//    0.9698229347747322, 0.2263471230292163, -0.09061156152126429, 0.114503835374182,
//    0, 0, 0, 1);
//
//    G_M1_M0[4] = (cv::Mat_<double>(4,4) << -0.09922398010779054, 0.03144714935208213, -0.994568086442148, 0.4833408601334332,
//    -0.2012709622706907, 0.9782065225341748, 0.05100979335604997, -0.1067194702914143,
//    0.9744971018521291, 0.2052390705231939, -0.09073214652247855, 0.09581598069066655,
//    0, 0, 0, 1);
//
//    G_M1_M0[5] = (cv::Mat_<double>(4,4) << -0.09877990451615454, 0.03094652416509303, -0.9946279923196805, 0.4832260210747155,
//    -0.1939733249045185, 0.9797446225194747, 0.04974760164729499, -0.1029741805835386,
//    0.9760209422390812, 0.1978453620539819, -0.09077628007614841, 0.0741610321403956,
//    0, 0, 0, 1);
//    G_M1_M0[6] = (cv::Mat_<double>(4,4) << -0.09873793158796369, 0.03148308162970365, -0.9946153208335514, 0.4831949227351496,
//    -0.2008505868910617, 0.9782983537379532, 0.0509055283748859, -0.1188957592686129,
//    0.9746331938792396, 0.2047953774983889, -0.09027176050152994, 0.0736107868853102,
//    0, 0, 0, 1);
//    G_M1_M0[7] = (cv::Mat_<double>(4,4) << -0.09922074253677335, 0.03318955905021063, -0.9945117884773934, 0.4829075644984591,
//    -0.2132604352947763, 0.9755111958047891, 0.05383208706146458, -0.1464449837958501,
//    0.9719440472518638, 0.2174312765670182, -0.08971292539039749, 0.06953526889449069,
//    0, 0, 0, 1);
//    G_M1_M0[8] = (cv::Mat_<double>(4,4) << -0.09838526536891457, 0.03256712521819754, -0.994615363803169, 0.4830812806742582,
//    -0.1613924944708867, 0.9857105811471014, 0.04824015902863566, -0.1526268092888103,
//    0.9819739315718886, 0.1652695754507329, -0.09172330752639393, 0.08200457652912763,
//    0, 0, 0, 1);
//    G_M1_M0[9] = (cv::Mat_<double>(4,4) << -0.09890773498887888, 0.0327153986271765, -0.9945586773298168, 0.4830592537795366,
//    -0.1474042376773344, 0.9879515094398352, 0.04715724451571926, -0.1599266245459512,
//    0.98411851454697, 0.1512663799005462, -0.09289365768258961, 0.09714732613634547,
//    0, 0, 0, 1);
//    G_M1_M0[10] = (cv::Mat_<double>(4,4) << -0.1067639321551482, 0.02996420402954088, -0.9938327873780614, 0.4969474351977282,
//    -0.2318523594212527, 0.971240400026971, 0.054190117053278, -0.1111183790882143,
//    0.9668743176967568, 0.2362080266043594, -0.0967461727723896, 0.09309078206558385,
//    0, 0, 0, 1);
//    G_M1_M0[11] = (cv::Mat_<double>(4,4) << -0.1072774933778697, 0.03100723716305243, -0.9937454858554459, 0.4967681042171346,
//    -0.156937779531873, 0.9864548553318594, 0.04772160671861, -0.1438164709502354,
//    0.9817647746635477, 0.161075664318713, -0.1009581972606431, 0.0892588886119916,
//    0, 0, 0, 1);
//    G_M1_M0[12] = (cv::Mat_<double>(4,4) << -0.1087575422932471, 0.03000729098841845, -0.993615297528107, 0.4967609945149469,
//    -0.158672372936941, 0.9862047671142982, 0.0471511970960755, -0.1361940942326992,
//    0.9813230227916251, 0.1627873453575831, -0.1024958786026937, 0.1183656350474899,
//    0, 0, 0, 1);
//    G_M1_M0[13] = (cv::Mat_<double>(4,4) << -0.1116408642462993, 0.02897441794540539, -0.9933261299971247, 0.4967126237121174,
//    -0.1975980331654765, 0.9789678497527831, 0.05076383003229827, -0.1086795982994172,
//    0.9739051980144039, 0.2019466074365642, -0.1035675288176153, 0.1213753623576069,
//    0, 0, 0, 1);
//    G_M1_M0[14] = (cv::Mat_<double>(4,4) << -0.1073918785505353, 0.0311672748929224, -0.9937281244873452, 0.447234184956006,
//    -0.1709899835038055, 0.9840364723457207, 0.04934213853045054, -0.1755621627410352,
//    0.9794025780866397, 0.1752165005618446, -0.1003482335076492, 0.1181801264331971,
//    0, 0, 0, 1);
//    G_M1_M0[15] = (cv::Mat_<double>(4,4) << -0.1062856207927515, 0.02827846747381478, -0.9939334460063369, 0.4475887844573958,
//    -0.1716140579001098, 0.9840733461902442, 0.04634937377132142, -0.1444184648270739,
//    0.9794141013604806, 0.1754992239164273, -0.09973986395105265, 0.1175146681370821,
//    0, 0, 0, 1);
//    G_M1_M0[16] = (cv::Mat_<double>(4,4) << -0.1071508350486726, 0.02530504036434782, -0.9939206977825398, 0.4479698673393226,
//    -0.158103935655312, 0.9865219581322358, 0.04216125772839913, -0.114943181758344,
//    0.9815914853332178, 0.1616603880209937, -0.1017058251342008, 0.1207377721253222,
//    0, 0, 0, 1);
//    G_M1_M0[17] = (cv::Mat_<double>(4,4) << -0.1052801953655415, 0.02399098227858731, -0.9941531638701864, 0.4483029458388502,
//    -0.1888163498631559, 0.9810409315075745, 0.04367008966219756, -0.1073918576655832,
//    0.9763526342915999, 0.1923099671781434, -0.09875429122723535, 0.09774545987487981,
//    0, 0, 0, 1);
//    G_M1_M0[18] = (cv::Mat_<double>(4,4) << -0.09333918036424824, 0.02477996441449571, -0.9953259520240325, 0.4480807976169245,
//    -0.1351776387820265, 0.9901180265352926, 0.0373269273202745, -0.1227889345070744,
//    0.9864151273080954, 0.1380298768146717, -0.08906710798201734, 0.07222187958475501,
//    0, 0, 0, 1);
//    G_M1_M0[19] = (cv::Mat_<double>(4,4) << -0.09712530275863523, 0.02651568861928795, -0.9949188880612772, 0.4478456737973661,
//    -0.1409779004305398, 0.9891992516602814, 0.04012570379364518, -0.1513589367194853,
//    0.9852369802003158, 0.1441587970669273, -0.09233815069610318, 0.09025713532846158,
//    0, 0, 0, 1);

    /***validation G_M1_M0****/

    G_M1_M0[0] = (cv::Mat_<double>(4,4) << -0.09597547008642393, 0.02576644524886519, -0.9950501492090376, 0.4480698925666283,
    -0.2482419123412272, 0.9674582072380831, 0.04899561413886638, -0.129426690590321,
    0.9639318762749063, 0.2517155290142258, -0.0864559445858297, 0.08234927575507045,
    0, 0, 0, 1);

    G_M1_M0[1] = (cv::Mat_<double>(4,4) << -0.0951754483391513, 0.02544898792121301, -0.9951351581806502, 0.4481250881986678,
    -0.2732409172484411, 0.9606086683876564, 0.05069898775837693, -0.1315793118907638,
    0.9572256970927354, 0.2767369422976982, -0.08447265588963308, 0.07012121113847081,
    0, 0, 0, 1);

    G_M1_M0[2] = (cv::Mat_<double>(4,4) << -0.0968175705228269, 0.02548295116473594, -0.9949758676661427, 0.4480798032172952,
    -0.2605968449954513, 0.9641494503963646, 0.05005119058328839, -0.143471460665056,
    0.9605808880133279, 0.2641334066344573, -0.08670583073359127, 0.07003286561672484,
    0, 0, 0, 1);

    G_M1_M0[3] = (cv::Mat_<double>(4,4) << -0.1075580228170585, 0.02685251896352799, -0.9938361102078139, 0.4479458987236127,
    -0.2480398201385734, 0.9673000576275715, 0.05297967666304656, -0.1358308519596018,
    0.9627603644486559, 0.2522093192945298, -0.09738038769526744, 0.1043869813066439,
    0, 0, 0, 1);

    G_M1_M0[4] = (cv::Mat_<double>(4,4) << -0.1064523305655772, 0.0254427320805082, -0.993992237747074, 0.448116679968052,
    -0.2748406930224379, 0.9599718351973074, 0.05400619489336025, -0.1240197803908233,
    0.9555786177892969, 0.2789386007927069, -0.09519854101667732, 0.1191682609750355,
    0, 0, 0, 1);

    G_M1_M0[5] = (cv::Mat_<double>(4,4) << -0.09497045559942141, 0.0257299419839457, -0.9951475180337546, 0.4480612624564435,
    -0.2901813666614013, 0.9555360481780464, 0.05239880795212631, -0.1367140403754501,
    0.9522475450248011, 0.2937496054768164, -0.08328134410811941, 0.06628208394524404,
    0, 0, 0, 1);
    G_M1_M0[6] = (cv::Mat_<double>(4,4) << -0.09769090509373851, 0.02781719560505864, -0.9948279704002273, 0.4344951791708217,
    -0.2763189055002621, 0.9595497112384224, 0.05396493422026559, -0.1314513361259607,
    0.9560880448604632, 0.2801616592093399, -0.08605286272964111, 0.0963324436590639,
    0, 0, 0, 1);
    G_M1_M0[7] = (cv::Mat_<double>(4,4) << -0.09615894982004605, 0.02944197933600082, -0.9949304630084881, 0.4342449362875485,
    -0.3165816998000521, 0.9467525948456019, 0.05861357781972942, -0.1384831038998763,
    0.9436786972911998, 0.3206129972504208, -0.08171794339608737, 0.07026475953020861,
    0, 0, 0, 1);
    G_M1_M0[8] = (cv::Mat_<double>(4,4) << -0.09652914704517769, 0.02923377607824833, -0.9949007538980646, 0.4341912054329188,
    -0.2858598356352287, 0.956642831223536, 0.0558448550827877, -0.1551101575477256,
    0.9533972299840874, 0.289792822210669, -0.08398715410016673, 0.06829495095854032,
    0, 0, 0, 1);
    G_M1_M0[9] = (cv::Mat_<double>(4,4) << -0.09901657388720415, 0.03057790532250752, -0.9946158604213631, 0.4340659269995281,
    -0.2762647812949648, 0.9593899208446948, 0.05699781046369541, -0.1736428297236075,
    0.9559673052524787, 0.2804210610629985, -0.08654790465757643, 0.08557035878787034,
    0, 0, 0, 1);
    G_M1_M0[10] = (cv::Mat_<double>(4,4) << 0.3040957313305351, 0.06607330647025264, -0.9503473598419869, 0.4738579777197773,
    -0.1929232819052612, 0.9811924100574597, 0.006485502643338456, -0.1030284826513774,
    0.9329021349988702, 0.1813719179413441, 0.3111235026429668, -0.1514134294983306,
    0, 0, 0, 1);
    G_M1_M0[11] = (cv::Mat_<double>(4,4) << 0.3925848375586832, 0.1207582071371722, -0.9117535855306729, 0.4611360560091238,
    -0.2058753260585703, 0.9777252457030899, 0.04084965159104235, -0.1450377381507243,
    0.8963774291219629, 0.1716706128719998, 0.4087012420299851, -0.171661749203529,
    0, 0, 0, 1);
    G_M1_M0[12] = (cv::Mat_<double>(4,4) << 0.4780151615508212, 0.1338558868654091, -0.8680922225656775, 0.439805589788951,
    -0.1794265634120671, 0.9823601862000957, 0.05267421485934892, -0.1687920109180228,
    0.8598299911434112, 0.1305797308941486, 0.493600364880268, -0.1988188544282956,
    0, 0, 0, 1);
    G_M1_M0[13] = (cv::Mat_<double>(4,4) << 0.5420495710874673, 0.1383737306560195, -0.8288757284106154, 0.4213016733295889,
    -0.203163558341289, 0.9786690412560549, 0.03052009582324122, -0.1747537713328708,
    0.8154181939630719, 0.1518539375560922, 0.5585996335504632, -0.2211682852941368,
    0, 0, 0, 1);
    G_M1_M0[14] = (cv::Mat_<double>(4,4) << 0.3727214209556048, 0.1130049803824621, -0.9210367076124568, 0.465930768809432,
    -0.1364031293058786, 0.9884472931615431, 0.0660767353701904, -0.2084923867986767,
    0.9178632407261982, 0.1010040744246379, 0.3838297125982093, -0.1530034289084198,
    0, 0, 0, 1);
    G_M1_M0[15] = (cv::Mat_<double>(4,4) << 0.3791490841356608, 0.1127037575291057, -0.9184464246965689, 0.4653488689755587,
    -0.1430983939633444, 0.98775605306799, 0.06213557171747835, -0.1399614716660204,
    0.9142039278214682, 0.107869563206527, 0.3906345295662328, -0.1594030145615447,
    0, 0, 0, 1);
    G_M1_M0[16] = (cv::Mat_<double>(4,4) << 0.3873515775428475, 0.11489845150543, -0.914744281871563, 0.4637823233992309,
    -0.1497624920550195, 0.9868668044080681, 0.06054012166223592, -0.1217402792191238,
    0.9096867325340832, 0.1135440716156614, 0.3994718919428498, -0.163915180867315,
    0, 0, 0, 1);
    G_M1_M0[17] = (cv::Mat_<double>(4,4) << 0.2941436655269234, 0.09888900898513156, -0.9506316152602563, 0.4807518758741989,
    -0.1372827871582049, 0.9886904889617983, 0.06037013654577322, -0.1237941955702497,
    0.9458503794895157, 0.1127478644516997, 0.3043928032676215, -0.1405940807280199,
    0, 0, 0, 1);
    G_M1_M0[18] = (cv::Mat_<double>(4,4) << 0.3082431417287376, 0.1018449104080535, -0.9458402506772341, 0.478464135935299,
    -0.1557883260517761, 0.9862343412855691, 0.05542401586861952, -0.1244313464600372,
    0.9384647905186336, 0.1302667965868319, 0.3198662199480002, -0.1461626724508002,
    0, 0, 0, 1);
    G_M1_M0[19] = (cv::Mat_<double>(4,4) << 0.6276259824513661, 0.1502383330685175, -0.7638809262108529, 0.3786464558095278,
    -0.1735881891440148, 0.9835067842219405, 0.050808916334826, -0.153678651923612,
    0.7589155201612754, 0.1007117106706603, 0.6433540118722588, -0.2670515057659569,
    0, 0, 0, 1);

    /*************after bumping left ***/

//    G_CB[0] = (cv::Mat_<double>(4,4) << 0.9981429944865514, -0.0506696624676273, 0.03381046972221532, 0.0116872522392597,
//    0.04786825230343491, 0.9957191257191558, 0.07906992537301885, -0.04692848233629661,
//    -0.03767217778194959, -0.07730464399049253, 0.9962955379999794, 0.322747378105623,
//    0, 0, 0, 1);
//
//    G_CB[1] = (cv::Mat_<double>(4,4) << 0.9962606829258598, -0.05352279244486523, 0.06782302223436706, -0.003788497743414527,
//    0.04857681133293689, 0.9961783946941722, 0.07258718444233937, -0.04562572687356051,
//    -0.07144889821980341, -0.06902113178908194, 0.9950533344046089, 0.3249881027114974,
//    0, 0, 0, 1);
//
//    G_CB[2] = (cv::Mat_<double>(4,4) << 0.9969677319173078, -0.02563294607997628, 0.07347308072294233, -0.007976247753700982,
//    0.02011431610691024, 0.9969888778871376, 0.0748905311564648, -0.04345601718686892,
//    -0.07517150925191181, -0.0731855822181334, 0.9944813295141255, 0.3252809990037451,
//    0, 0, 0, 1);
//
//    G_CB[3] = (cv::Mat_<double>(4,4) << 0.9966307854148868, -0.05798841951435575, 0.05800362717565918, -0.002264419496212785,
//    0.05359156939950237, 0.9957662133598194, 0.07468327805034329, -0.05273443240354306,
//    -0.06208881745213147, -0.07132314864946478, 0.9955189537191756, 0.3249664780133008,
//    0, 0, 0, 1);
//
//    G_CB[4] = (cv::Mat_<double>(4,4) << 0.9989425259086663, -0.04267040862746267, -0.0171191751781148, 0.02499664158369121,
//    0.04403006078175017, 0.9950555200433618, 0.0890273316391658, -0.05124172084083855,
//    0.01323569713951659, -0.08968694586616265, 0.9958820552969267, 0.321203222805342,
//    0, 0, 0, 1);
//
//    G_CB[5] = (cv::Mat_<double>(4,4) << 0.9995796815854163, -0.02672892880447594, 0.01122606460710838, 0.01923098395388613,
//    0.02589878392436649, 0.9973134490321833, 0.06852107245781923, -0.02207147537854103,
//    -0.01302740007970474, -0.0682015303676977, 0.9975865065760801, 0.3191939099361498,
//    0, 0, 0, 1);
//
//
//    G_CB[6] = (cv::Mat_<double>(4,4) << 0.9982322499808768, -0.05282840769558327, 0.02723112995205192, 0.02175809943698363,
//    0.05042144695555752, 0.9953094653118369, 0.08256358729835585, -0.04008509582965229,
//    -0.03146510424302135, -0.08104460254091302, 0.9962136917418678, 0.3355963048138974,
//    0, 0, 0, 1);
//    G_CB[7] = (cv::Mat_<double>(4,4) << 0.9984155868350326, -0.0561553026713548, 0.003591371151256772, 0.02221718338381298,
//    0.05561892008739167, 0.9945318902382385, 0.08839035596414782, -0.05925968266446842,
//    -0.00853532033200197, -0.08805056093543615, 0.9960794381101264, 0.3366300939294228,
//    0, 0, 0, 1);
//    G_CB[8] = (cv::Mat_<double>(4,4) <<0.997807791064072, -0.0618282630308858, 0.02359826227544774, 0.008233749333865394,
//    0.05965223405210605, 0.9946917205132005, 0.0838450482442562, -0.0688283302793553,
//    -0.0286569898005701, -0.08225355331578382, 0.9961993424523515, 0.3392661019045268,
//    0, 0, 0, 1);
//    G_CB[9] = (cv::Mat_<double>(4,4) <<0.9969136248907466, -0.07657245306340384, 0.01731715736039031, 0.009516846139530816,
//    0.07465579425138012, 0.9929010833316978, 0.09259563220496075, -0.07572953818999643,
//    -0.02428449900424761, -0.09101702121358171, 0.9955531954433772, 0.3408496341225382,
//    0, 0, 0, 1);
//    G_CB[10] = (cv::Mat_<double>(4,4) <<0.9924356450748774, 0.1218530941671789, 0.01494368852381306, -0.02726705481052964,
//    -0.1227396869056844, 0.9873646721564081, 0.1002296035897735, -0.04874339897108514,
//    -0.002541582795558449, -0.101305614944845, 0.9948521059622581, 0.2341062750503555,
//    0, 0, 0, 1);
//
//
//    G_CB[11] = (cv::Mat_<double>(4,4) <<0.992702610948179, 0.1141944795610305, 0.03874463904146953, -0.03943739841362071,
//    -0.1172541086343226, 0.9891066741618779, 0.08899135428118429, -0.04860543638096516,
//    -0.02816025967633926, -0.09288491786191214, 0.9952785498586549, 0.2353316696928548,
//    0, 0, 0, 1);
//    G_CB[12] = (cv::Mat_<double>(4,4) <<0.9976575628751403, -0.03563034535274059, 0.05839405558856062, 0.002938834717249791,
//    0.03459519110574804, 0.9992274988296145, 0.0186434528741196, -0.0344151662957096,
//    -0.05901321877674731, -0.01657962824544579, 0.9981195098468172, 0.3271953621371128,
//    0, 0, 0, 1);
//    G_CB[13] = (cv::Mat_<double>(4,4) <<0.9981356797430496, -0.04200898410638249, 0.04427651836165613, -0.05018121960058201,
//    0.04058277209923115, 0.9986431456789426, 0.03263290053317369, -0.02147187733878145,
//    -0.04558731657623898, -0.03077519850164814, 0.9984861960610982, 0.3290386612756258,
//    0, 0, 0, 1);
//    G_CB[14] = (cv::Mat_<double>(4,4) <<0.9867141885308268, -0.1469696829701353, 0.06924610053721267, -0.009539045821019748,  ///15
//    0.142046073107768, 0.9872839367367713, 0.07136764936724244, -0.05523044303353262,
//    -0.078854443543877, -0.06058333557341442, 0.9950435348185448, 0.3276878087903624,
//    0, 0, 0, 1);
//    G_CB[15] = (cv::Mat_<double>(4,4) <<0.9920640558120994, -0.1169559649386011, 0.04615421357718421, -0.0004135349397457523,  ///20
//    0.1134848977428265, 0.9909440521607636, 0.07177090964663883, -0.06808352806780378,
//    -0.05413027941871203, -0.06596353350516095, 0.9963527111909546, 0.3277077524679659,
//    0, 0, 0, 1);
//    G_CB[16] = (cv::Mat_<double>(4,4) <<0.9839106931570076, -0.172675362582753, 0.04585811867283177, 0.01176147340305161,
//    0.1685853236650522, 0.9822971308324134, 0.08167824314441438, -0.03599932694040796,
//    -0.05915011864777897, -0.07263309104893764, 0.9956031827734534, 0.3241719636779815,
//    0, 0, 0, 1);
//    G_CB[17] = (cv::Mat_<double>(4,4) <<0.9979165633934075, -0.02959556021144593, 0.0573291838496067, -0.01439571966948813,
//    0.05397661683574535, 0.8697273369061433, -0.4905719970331724, -0.04941032078924069,
//    -0.03534200532027658, 0.4926443567665406, 0.8695127833482513, 0.2554175430194314,
//    0, 0, 0, 1);
//    G_CB[18] = (cv::Mat_<double>(4,4) <<0.9721195295261201, -0.1404629386114576, 0.1877599083685033, -0.01519486671602243,  ///23
//    0.2182607527471929, 0.8347091185650279, -0.5055916644829331, -0.01267927109324167,
//    -0.08570801668539291, 0.5324761499457303, 0.8420945823450169, 0.2511570949334484,
//    0, 0, 0, 1);
//    G_CB[19] = (cv::Mat_<double>(4,4) <<0.946495212072537, -0.318825087254585, 0.04997376572632505, -0.03772276840165999,
//    0.3130663113812217, 0.9447007312055835, 0.09762178618419037, -0.04749774597985102,
//    -0.07833452752087873, -0.07675345071553524, 0.9939681129700986, 0.2356897564137327,
//    0, 0, 0, 1);
//
//    G_CB[20] = (cv::Mat_<double>(4,4) <<0.9533405379428028, -0.1756728535441617, 0.245521622759746, -0.01593878393150837,
//    0.2863813514342462, 0.7835997098938554, -0.5513231504344439, -0.01520481959859468,
//    -0.09553816130542962, 0.5959115229477588, 0.7973467981703196, 0.24393353990813,
//    0, 0, 0, 1);
//
//    G_M1_M0[0] = (cv::Mat_<double>(4,4) << -0.08931394699001373, 0.04176006873844338, -0.9951276880541643, 0.4959831314171442,
//    -0.08984930742892261, 0.9947092063066667, 0.04980659437568308, -0.1040800710710775,
//    0.9919425995629023, 0.0938599571048343, -0.08508929206821843, 0.1048544231086973,
//    0, 0, 0, 1);
//
//    G_M1_M0[1] = (cv::Mat_<double>(4,4) << -0.08907566769791796, 0.04174854267755119, -0.9951495287686525, 0.4959774218356768,
//    -0.09040652119501005, 0.9946580362022975, 0.0498201961417782, -0.1196623851984883,
//    0.9919133965974981, 0.09440577420099766, -0.08482548819738081, 0.1028165165663008,
//    0, 0, 0, 1);
//
//    G_M1_M0[2] = (cv::Mat_<double>(4,4) << -0.08856234375216337, 0.04164186883360834, -0.995199812112707, 0.4959766149606828,
//    -0.06091612184519377, 0.9970291428340242, 0.04713930885144618, -0.1206825807074932,
//    0.9942061845354133, 0.06479848068971283, -0.08576257652286215, 0.1014171733302544,
//    0, 0, 0, 1);
//
//    G_M1_M0[3] = (cv::Mat_<double>(4,4) << -0.0886623529217014, 0.04159940617400475, -0.9951926831424944, 0.4960379173588483,
//    -0.09631593643345018, 0.9940874292338815, 0.05013405457491224, -0.1191169610658165,
//    0.9913940828769027, 0.1002979184487041, -0.08413144472503672, 0.1096928779608882,
//    0, 0, 0, 1);
//
//    G_M1_M0[4] = (cv::Mat_<double>(4,4) << -0.08859750784803769, 0.04005713366546883, -0.9952617282130483, 0.4969634796509087,
//    -0.04106019758654858, 0.9981948516378226, 0.04383033581777334, -0.07938459579839777,
//    0.9952208507548811, 0.04474890173235499, -0.08679282237879818, 0.08158860279529245,
//    0, 0, 0, 1);
//
//    G_M1_M0[5] = (cv::Mat_<double>(4,4) << -0.08810085073935817, 0.04047327438851168, -0.9952889802259814, 0.4969503508311658,
//    -0.06852136120818716, 0.9965611673648531, 0.04659037195149572, -0.09267612941362835,
//    0.9937520129072341, 0.07230320712578259, -0.0850246028049981, 0.08028692415845207,
//    0, 0, 0, 1);
//    G_M1_M0[6] = (cv::Mat_<double>(4,4) << -0.09575539575903512, 0.0380797303127398, -0.994676247993456, 0.510145518600887,
//    -0.09454181202949302, 0.994402725356929, 0.0471706008112244, -0.09368931805280734,
//    0.9909050156100785, 0.09855533441686906, -0.09161929980505519, 0.1032159393066466,
//    0, 0, 0, 1);
//    G_M1_M0[7] = (cv::Mat_<double>(4,4) << -0.09518079864355158, 0.03802181597206561, -0.9947336111139314, 0.5103054107268222,
//    -0.09988828360752336, 0.9938620034312518, 0.04754628201617295, -0.09482739346179062,
//    0.9904357356070701, 0.1038877261557152, -0.09079864529097585, 0.1217018193413809,
//    0, 0, 0, 1);
//    G_M1_M0[8] = (cv::Mat_<double>(4,4) << -0.0934780003827454, 0.03708701725153431, -0.9949303576611922, 0.5103531629416611,
//    -0.101309629479455, 0.993764728204224, 0.04656204411234607, -0.107087214824982,
//    0.9904535437965674, 0.1051485526698723, -0.08913786766718257, 0.1292311978269054,
//    0, 0, 0, 1);
//    G_M1_M0[9] = (cv::Mat_<double>(4,4) << -0.09528659122752452, 0.03621092590003121, -0.994791050611987, 0.5109375559480407,
//    -0.1183582041894639, 0.9918370550165916, 0.0474404025811284, -0.1106642904029484,
//    0.9883884868983873, 0.1222621165426021, -0.09022291186411245, 0.1372683371207305,
//    0, 0, 0, 1);
//    G_M1_M0[10] = (cv::Mat_<double>(4,4) << -0.08997527712791727, 0.02676245074203641, -0.9955843614360534, 0.4035523942935796,
//    0.08226976817802861, 0.9964222331398024, 0.0193498978969685, -0.1110321597314041,
//    0.9925402433905069, -0.08016548219133804, -0.09185510718159498, 0.1053684329241671,
//    0, 0, 0, 1);
//    G_M1_M0[11] = (cv::Mat_<double>(4,4) << -0.08923814467687979, 0.02907568799283311, -0.9955858365316232, 0.4034004887391985,
//    0.08325679870157832, 0.9962932737982696, 0.02163372492871667, -0.117979139553337,
//    0.9925244878614298, -0.08095873610716708, -0.09132811201014107, 0.1080938163325653,
//    0, 0, 0, 1);
//    G_M1_M0[12] = (cv::Mat_<double>(4,4) << -0.02271326954664851, 0.04115732375876206, -0.9988944799564756, 0.5052579683225309,
//    -0.07857854917385301, 0.995987695879527, 0.04282430695671816, -0.1083686369239891,
//    0.9966491453847909, 0.07946435903980653, -0.01938805421281531, 0.05915118255416429,
//    0, 0, 0, 1);
//    G_M1_M0[13] = (cv::Mat_<double>(4,4) << -0.02074233757700193, 0.03989474652599962, -0.9989885708212424, 0.5040556003653623,
//    -0.08208439060008862, 0.9957622062615753, 0.04147024717665002, -0.1606392293125576,
//    0.9964095083105402, 0.08286155791865507, -0.01737969987196623, 0.04341264310443704,
//    0, 0, 0, 1);
//    G_M1_M0[14] = (cv::Mat_<double>(4,4) << -0.09230378536632816, 0.03701068360032234, -0.9950428234535856, 0.4981225998616953,
//    -0.1861519230534125, 0.9810491857337733, 0.05375831763193769, -0.1343420237443529,
//    0.9781755838041363, 0.1901912313187354, -0.08366494354805971, 0.1072386211176891,
//    0, 0, 0, 1);
//    G_M1_M0[15] = (cv::Mat_<double>(4,4) << -0.09303729193726035, 0.03610027141411972, -0.9950079561052806, 0.4981768823914036,
//    -0.1581643229502334, 0.9861171608485141, 0.05056670867042677, -0.1230793161463862,
//    0.9830198926037431, 0.1620793493443642, -0.08603589519167143, 0.1232573725965872,
//    0, 0, 0, 1);
//    G_M1_M0[16] = (cv::Mat_<double>(4,4) << -0.08991871398921114, 0.03491982632768708, -0.9953367423157707, 0.4985538976056396,
//    -0.2122538037371175, 0.9757540310923096, 0.05340780473145586, -0.1116794668450192,
//    0.9730688298746653, 0.2160663706742727, -0.08032668168171814, 0.08964390144657264,
//    0, 0, 0, 1);
//    G_M1_M0[17] = (cv::Mat_<double>(4,4) << 0.4914657804051865, 0.07389060407065773, -0.8677566279325065, 0.4076174185128106,
//    -0.1094338736047248, 0.9937362439579412, 0.02263856780426476, -0.1134810718395983,
//    0.8639939895616091, 0.08383588774764938, 0.4964734936801533, -0.1617576144182182,
//    0, 0, 0, 1);
//    G_M1_M0[18] = (cv::Mat_<double>(4,4) << 0.4930670986286515, 0.2004886552179465, -0.8465749437461654, 0.3750129401369631,
//    -0.2728397350111249, 0.9596282052643376, 0.0683533953816894, -0.1359472353494255,
//    0.8261012742085456, 0.1972764729764951, 0.5278623664943081, -0.2080823831291941,
//    0, 0, 0, 1);
//    G_M1_M0[19] = (cv::Mat_<double>(4,4) << -0.1102335067319961, 0.03577467295349371, -0.9932616708444143, 0.4020717442278134,
//    -0.3517728527803257, 0.9332616343035229, 0.07265385044103748, -0.170386379861606,
//    0.9295721779616455, 0.3574113802219898, -0.09029214388557723, 0.08200270049042491,
//    0, 0, 0, 1);
//    G_M1_M0[20] = (cv::Mat_<double>(4,4) << 0.5324759839717476, 0.2613931355382502, -0.8050732607575544, 0.3530187923244555,
//    -0.3435874896433667, 0.935987427609324, 0.0766496726532459, -0.1421380773172418,
//    0.7735741486463261, 0.2357989907755382, 0.5882022377510555, -0.2303255133034405,
//    0, 0, 0, 1);

};
