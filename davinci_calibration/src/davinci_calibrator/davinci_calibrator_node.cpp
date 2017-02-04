/***************************	
    // 1- Tool coordinate on Camera
    // 2- Tool coordinate on Calibration pattern
    // 3- Frame from calibration pattern to tool 
    // 4- Define calibration pattern corners (world coordinates) 
    // 5- Subscribe to calibration node to get coordinates of chessboard corners
    	  a- Chessboard camera coordinates are in pixel, convert them into m or mm
    // 6- Get camera to calibration pattern transformation
    //    a- Call solvePnP with world coordinates and camera coordinates of chessboard
    // 7- Get NDI tracker pose to communicate with the camera_calibration node
**************************/
#include <ros/ros.h>
#include <davinci_calibrator/davinci_calibrator.h>

using namespace cv_projective;

int main(int argc, char **argv) {

    ros::init(argc, argv, "davinci_calibrator");
    ros::NodeHandle nh; 

   	DavinciCalibrator calibrator(&nh);

    const std::string leftCameraTopic("/davinci_endo/left/camera_info");
    const std::string rightCameraTopic("/davinci_endo/right/camera_info");
    
    cameraProjectionMatrices cameraInfoObj(nh, leftCameraTopic, rightCameraTopic);   //camera info object

    ros::Duration(1).sleep();

    cv::Mat P_l, P_r;
    P_l.setTo(0);
    P_r.setTo(0);

    cv::Mat left_cam_pose, right_cam_pose;
    left_cam_pose = cv::Mat::eye(4, 4, CV_64F);
    right_cam_pose = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat G_CM_l = cv::Mat::eye(4,4,CV_64F); ///g of maker relative to left camera
    cv::Mat G_CM_r = cv::Mat::eye(4,4,CV_64F); ///g of maker relative to right camera

    bool freshCameraInfo = false;

    cv::Mat camera_matrix_l = (cv::Mat_<float>(3,3) << 764.0635671930186, 0, 344.710190632238,
            0, 760.4650393204188, 284.5192831458843,
            0, 0, 1);

    cv::Mat camera_matrix_r = (cv::Mat_<float>(3,3) << 755.7644364668356, 0, 287.1735406571376,
            0, 756.1752058068007, 254.5592709895131,
            0, 0, 1);

    while(nh.ok()){

        ros::spinOnce();

        // make sure camera information is ready.
        if(!freshCameraInfo)
        {
            //retrive camera info
            P_l = cameraInfoObj.getLeftProjectionMatrix();
            P_r = cameraInfoObj.getRightProjectionMatrix();

            ROS_INFO_STREAM("projection mat left: " << P_l);
            ROS_INFO_STREAM("projection mat right: " << P_r);

            if(P_l.at<double>(0,0) != 0 && P_r.at<double>(0,0))
            {
                // ROS_INFO("obtained camera info");
                freshCameraInfo = true;

                cv::Mat intrinsic_l = P_l.colRange(0,3).rowRange(0,3);    ///peel off the intirnsic matrix from projection matrix 
                cv::Mat intrinsic_r = P_r.colRange(0,3).rowRange(0,3);


                if (calibrator.freshLeftCorner && calibrator.freshRightCorner )
                {
                    calibrator.setBoardCoord();  ///set the 3d coordinates for the chessboard or circle board
                    
                    if (calibrator.boardMatch) //when get the board corners match the 3d board set up
                    {
                        calibrator.computeCameraPose(calibrator.left_corner_coordinates, intrinsic_l, left_cam_pose);  ///get camera poses
                        calibrator.computeCameraPose(calibrator.right_corner_coordinates, intrinsic_r, right_cam_pose);

                        ROS_INFO_STREAM("LEFT Camera Pose: " << left_cam_pose );
                        ROS_INFO_STREAM("RIGHT Camera Pose: " << right_cam_pose );

                        if(calibrator.freshMakers){  //get fresh maker poses
                            //when get everything ready, compute G_CM

                            G_CM_l = left_cam_pose * calibrator.g_bm * calibrator.g_mm;
                            G_CM_r = right_cam_pose * calibrator.g_bm * calibrator.g_mm;
                            ROS_INFO_STREAM("LEFT G_CM_l: " << G_CM_l );
                            ROS_INFO_STREAM("LEFT G_CM_r: " << G_CM_r );

                            cv::Mat diffmat = G_CM_l - G_CM_r;

                            ROS_INFO_STREAM("difference mat for l and r: " << diffmat);
                            /***testing using left camera now*/
                            // calibrator.testCamToBoard(G_CM_l, calibrator.marker_poses, calibrator.g_bm, left_cam_pose);

                            // calibrator.testCamToBoard2(calibrator.marker_poses, calibrator.g_bm);



                        }else{
                            ROS_INFO(" No marker detected! ");
                        }

                    }
                }
                else{
                    ROS_INFO(" No corner information received! ");
                }
                
            }
        
        }

        calibrator.freshLeftCorner = false;
        calibrator.freshRightCorner = false;
        calibrator.boardMatch = false;
        calibrator.freshMakers = false;

        freshCameraInfo = false;
        ros::Duration(1).sleep();
    }

    return 0; // should never get here, unless roscore dies
}


