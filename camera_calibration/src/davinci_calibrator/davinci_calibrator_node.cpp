/***************************	
    // 1- Tool coordinate on Camera
    // 2- Tool coordinate on Calibration pattern
    // 3- Frame from calibration pattern to tool 
    // 4- Define calibration pattern corners (world coordinates) 
    // 5- Subscribe to calibration node to get coordinates of chessboard corners
    	  a- Chessboard camera coordinates are in pixel, convert them into m or mm
    // 6- Get camera to calibration pattern transformation
    //    a- Call solvePnP with world coordinates and camera coordinates of chessboard
    // 7- Get NDI tracker pose to communicate with the cammera_calibration node
**************************/
#include <ros/ros.h>
#include <davinci_calibrator/davinci_calibrator.h>

#include <stdio.h>
#include <stdlib.h>

#include <cv_bridge/cv_bridge.h>

using namespace cv_projective;

int main(int argc, char **argv) {

    ros::init(argc, argv, "davinci_calibrator");
    ros::NodeHandle nh; 

   	DavinciCalibrator calibrator(&nh);

    const std::string leftCameraTopic("/stereo_example/left/camera_info");
    const std::string rightCameraTopic("/stereo_example/right/camera_info");
    
    cameraProjectionMatrices cameraInfoObj(nh, leftCameraTopic, rightCameraTopic);   //camera info object

    cv::Mat P_l, P_r;
    P_l.setTo(0);
    P_r.setTo(0);

    cv::Mat left_cam_pose, right_cam_pose;
    left_cam_pose = cv::Mat::eye(4, 4, CV_64F);
    right_cam_pose = cv::Mat::eye(4, 4, CV_64F);


    while(nh.ok()){

        ros::spinOnce();

        P_l = cameraInfoObj.getLeftProjectionMatrix();
        P_r = cameraInfoObj.getRightProjectionMatrix();

        cv::Mat intrinsic_l = P_l.colRange(0,3).rowRange(0,3);    ///peel off the intirnsic matrix from projection matrix 
        cv::Mat intrinsic_r = P_r.colRange(0,3).rowRange(0,3);

        calibrator.computeCameraPose(calibrator.left_corner_coordinates, intrinsic_l, left_cam_pose);  ///get camera poses
        calibrator.computeCameraPose(calibrator.right_corner_coordinates, intrinsic_r, right_cam_pose);

    }

    return 0; // should never get here, unless roscore dies
}


