#ifndef DAVINCI_CALIBRATOR_H
#define DAVINCI_CALIBRATOR_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.hpp"
#include <davinci_calibration/intrinsic_param.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

class DavinciCalibrator {
private:

    ros::NodeHandle nh_;

    int lcorner_size;
    int rcorner_size;

    std::vector<cv::Point3f> board_coordinates;  /////the  3d coordinates that pass to SolvePnP


    ros::Subscriber leftcorner_size_subscriber;
    ros::Subscriber rightcorner_size_subscriber;

    ros::Subscriber leftcorner_subscriber;
    ros::Subscriber rightcorner_subscriber;

    ros::Subscriber polaris_subscriber;

    void leftCornerSizeCB(const std_msgs::Int32::ConstPtr &leftCornerSizeData);

    void rightCornerSizeCB(const std_msgs::Int32::ConstPtr &rightCornerSizeData);

    void leftcornerCB(const std_msgs::Float32MultiArray::ConstPtr &leftcornerData);

    void rightcornerCB(const std_msgs::Float32MultiArray::ConstPtr &rightcornerData);

    void polarisTargetsCB(const geometry_msgs::PoseArray::ConstPtr &target_poses);

public:
    /*
   * This constructor uses an input charactor to set who it subscribes to
   * It also then initializes the publisher.
   */
    DavinciCalibrator(ros::NodeHandle *nodehandle);

    std::vector<cv::Mat> marker_poses;
    
    bool freshLeftCorner;
    bool freshRightCorner;
    bool boardMatch; ///TODO: CURRENTLY support 5 * 7 chessboard
    bool freshMakers;

    cv::Mat g_bm;
    cv::Mat g_mm;

    std::vector<cv::Point2f> left_corner_coordinates;
    std::vector<cv::Point2f> right_corner_coordinates;

    /*  Not using this if we already have camera_info
    *   ros::Subscriber intrinsics_subscriber;
    *   void camIntrinsicCB(const camera_calibration::intrinsic_param& intrinsicsData);
    */

    void setBoardCoord();

    void computeCameraPose(const std::vector<cv::Point2f> &corner_coords, const cv::Mat &cameraMatrix,
                           cv::Mat &output_cam_pose);

    void convertQuaternionsToRvec(const cv::Mat &quaternion, cv::Mat &Rod_rvec);
    // void convertQuaternionsToRot( const cv::Mat &quaternion, cv::Mat &rot_matrix);

    void computeMakersGeometry(const std::vector<cv::Mat> &markers, cv::Mat &outputGeometry);

    void computeInv(const cv::Mat &inputMat, cv::Mat &outputMat);

    /* couple testing functions*/
    void testCamToBoard(const cv::Mat &G_CT2, const std::vector<cv::Mat> &markers, const cv::Mat &g_BM, const cv::Mat &solvePnpCamBoard );
    void testCamToBoard2(const std::vector<cv::Mat> &markers, const cv::Mat &g_BM);
    void testCamToBoard3();
};

#endif