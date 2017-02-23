/***************************	
    // 1- Tool coordinate on Camera
    // 2- Tool coordinate on Calibration pattern
    // 3- Frame from calibration pattern to tool 
    // 4- Define calibration pattern corners (world coordinates) 
    // 5- Subscribe to calibration node to get coordinates of chessboard corners
    	  a- Chessboard camera coordinates are in pixel, convert them into m
    // 6- Get camera to calibration pattern transformation
    //    a- Call solvePnP with world coordinates and camera coordinates of chessboard
    // 7- Get NDI tracker pose to communicate with the camera_calibration node
**************************/
#include <ros/ros.h>
#include <davinci_calibrator/davinci_calibrator.h>

using namespace cv_projective;
using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "davinci_calibrator");
    ros::NodeHandle nh; 

   	DavinciCalibrator calibrator(&nh);

    ros::Duration(1).sleep();

    cv::Mat left_cam_pose, right_cam_pose;
    left_cam_pose = cv::Mat::eye(4, 4, CV_64F);
    right_cam_pose = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat G_CM_l = cv::Mat::eye(4,4,CV_64F); ///g of maker relative to left camera
    cv::Mat G_CM_r = cv::Mat::eye(4,4,CV_64F); ///g of maker relative to right camera

    calibrator.collect_data_num = 1;

    while(nh.ok()){

        ros::spinOnce();

        // make sure camera information is ready.
        if(calibrator.freshCameraInfo)
        {
            //retrive camera info
//            ROS_INFO_STREAM("camera_intrinsic_r" << calibrator.camera_intrinsic_r);
//            ROS_INFO_STREAM("camera_intrinsic_l" << calibrator.camera_intrinsic_l);

            if(calibrator.freshMakers){  //get fresh maker poses
                /*
                 * for computing base transformation
                 */
                //computeBaseTransformation();

                if (calibrator.freshLeftCorner && calibrator.freshRightCorner )
                {
                    calibrator.setBoardCoord();  ///set the 3d coordinates for the chessboard or circle board
                    
                    if (calibrator.boardMatch) //when get the board corners match the 3d board set up
                    {
                        cin >> calibrator.collect_data_num;
                        if(calibrator.collect_data_num <= 20){
                            calibrator.computeCameraPose(calibrator.left_corner_coordinates, calibrator.total_corners_left, calibrator.camera_intrinsic_l, left_cam_pose);  ///get camera poses
                            calibrator.computeCameraPose(calibrator.right_corner_coordinates, calibrator.total_corners_right, calibrator.camera_intrinsic_r, right_cam_pose);
                            ROS_INFO_STREAM("calibrator.total_corners_left size " << calibrator.total_corners_left.size());
                            ROS_INFO_STREAM("calibrator.total_corners_left " << calibrator.total_corners_left);
                        }

                        if(calibrator.collect_data_num == 20){
                            ROS_INFO_STREAM("Left camera pose: " << left_cam_pose );
                            ROS_INFO_STREAM("Right camera pose: " << right_cam_pose );

                            //void cv_projective::reprojectPoints(board_coordinates, temp_Points, P_l, const cv::Mat &P_r, const cv::Mat & = cv::Mat(),  const cv::Mat & = cv::Mat() );
                            //when get everything ready, compute G_CM

                            cv::Mat g_cm = left_cam_pose * calibrator.g_bm;
                            ROS_INFO_STREAM("g_bm:" << calibrator.g_bm);

//                          ROS_INFO_STREAM("LEFT g_cm: " << g_cm );
//                          cv::Mat g_cm_inv = g_cm.inv();
//                          ROS_INFO_STREAM("LEFT g_cm_inv: " << g_cm_inv );

                            G_CM_l = left_cam_pose * calibrator.g_bm * calibrator.g_M1_M0;
                            G_CM_r = right_cam_pose * calibrator.g_bm * calibrator.g_M1_M0;

                            ROS_INFO_STREAM("LEFT G_CM_l: " << G_CM_l );
                            ROS_INFO_STREAM("RIGHT G_CM_r: " << G_CM_r );

//                            // ROS_INFO_STREAM("difference mat for l and r: " << diffmat);
                            /***testing using left camera now*/
                            calibrator.testCamToBoard(calibrator.marker_poses, calibrator.g_bm, left_cam_pose);
                            // calibrator.testCamToBoard2(calibrator.marker_poses, calibrator.g_bm);
                        }

                    }

                }else{
                ROS_INFO(" No corner information received! ");
                }

            }else{
            ROS_INFO(" No marker detected! ");
            }
        
        }

        calibrator.freshCameraInfo = false;
        calibrator.freshLeftCorner = false;
        calibrator.freshRightCorner = false;
        calibrator.boardMatch = false;
        calibrator.freshMakers = false;

        ros::Duration(1.2).sleep();
    }

    return 0; // should never get here, unless roscore dies
}


