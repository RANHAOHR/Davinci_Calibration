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

#include <boost/random.hpp>

class DavinciCalibrator {
private:

    ros::NodeHandle nh_;

    /**
     * @brief Left and right corner sizes, amid to debug if correct number of corners are collected
     */
    int lcorner_size;
    int rcorner_size;

    /**
     * @brief The 3d coordinates that pass to SolvePnP
     */
    std::vector<cv::Point3f> board_coordinates;

    /**
     * @brief Subscribers
     */
    ros::Subscriber leftcorner_size_subscriber;
    ros::Subscriber rightcorner_size_subscriber;

    ros::Subscriber leftcorner_subscriber;
    ros::Subscriber rightcorner_subscriber;

    ros::Subscriber polaris_subscriber;

    ros::Subscriber cameraMat_subscriber_r;
    ros::Subscriber cameraMat_subscriber_l;

    /**
     * @brief left and right corner size callback functions
     */
    void leftCornerSizeCB(const std_msgs::Int32::ConstPtr &leftCornerSizeData);
    void rightCornerSizeCB(const std_msgs::Int32::ConstPtr &rightCornerSizeData);
    /**
    * @brief left and right corners callback functions
    */
    void leftcornerCB(const std_msgs::Float32MultiArray::ConstPtr &leftcornerData);
    void rightcornerCB(const std_msgs::Float32MultiArray::ConstPtr &rightcornerData);

    /**
     * @brief Polaris call back function, getting the transformations of two markers defined from the launch functions
     * @param target_poses : should be the two markers relative to the polaris frame in order defined from the launch functions
     */
    void polarisTargetsCB(const geometry_msgs::PoseArray::ConstPtr &target_poses);

    /**
     * @brief Left and right callback functions for camera projection matrices
     */
    void cameraMatRightCB(const sensor_msgs::CameraInfo::ConstPtr &cameraMatRight);
    void cameraMatLeftCB(const sensor_msgs::CameraInfo::ConstPtr &cameraMatLeft);


public:

    /**
     * This constructor uses an input charactor to set who it subscribes to
     * It also then initializes the publisher.
     */
    DavinciCalibrator(ros::NodeHandle *nodehandle);

    /**
     * @brief Store Polaris Markers poses
     */
    std::vector<cv::Mat> marker_poses;

    /**
     * @brief The indicators for callback function, turns true when getting new datas
     */
    bool freshLeftCorner; /// when left corner callback function is ready
    bool freshRightCorner; ///when right corner callback function is ready
    bool boardMatch; ///TODO: CURRENTLY support 5 * 7 chessboard
    bool freshMakers;  /// when polaris callback function is ready

    bool freshCameraInfo; ///need to be true when getting the camera images

    /**
     * The transformation of the borad frame relative to the marker on the board.
     */
    cv::Mat g_bm;
    /**
     * the transformation of the first marker[0] relative to the second marker[1]
     */
    cv::Mat g_M1_M0;

    std::vector<cv::Point2f> left_corner_coordinates;
    std::vector<cv::Point2f> right_corner_coordinates;

    /**
     * camera extrinsic parameters, using the left and right projection matrices
     */
    cv::Mat camera_mat_r;
    cv::Mat camera_mat_l;

    /**
     * @brief Main function od camera calibration
     */
    void stereoCameraCalibration();

    /**
     * @brief set 3D coordinates for the corners on board for SolvePnP
     */
    void setBoardCoord();

    /**
     * @brief Compute the transformation of board frame relative to camera frame. This gives the Camera Extrinsic parameters
     * @param corner_coords : 2D corners coordinates collected from the left and right corner callback functions
     * @param cameraMatrix : camera projections matricies
     * @param output_cam_pose : The output camera extrinsic parameter
     */
    void computeCameraPose(const std::vector<cv::Point2f> &corner_coords, const cv::Mat &cameraMatrix,
                           cv::Mat &output_cam_pose);

    /**
     * @brief compute quaternions to Rodrigues vectors
     * @param quaternion : input quaternion
     * @param Rod_rvec
     */
    void convertQuaternionsToRvec(const cv::Mat &quaternion, cv::Mat &Rod_rvec);
    /**
     * @brief compute quaternions to Rotation matrices
     * @param quaternion
     * @param rot_matrix
     */
    void convertQuaternionsToRot( const cv::Mat &quaternion, cv::Mat &rot_matrix);

    /**
     * @brief compute the transformation of the first maker relative to the second marker
     * @param markers : The marker posees collected from the Polaris callback function
     * @param outputGeometry
     */
    void computeMakersGeometry(const std::vector<cv::Mat> &markers, cv::Mat &outputGeometry);

    /**
     * @brief Compute the inverse of a SE(3) matrix
     * @param inputMat
     * @param outputMat
     */
    void computeInvSE(const cv::Mat &inputMat, cv::Mat &outputMat);

    /**
     * @brief couple of testing functions
     */
    void testCamToBoard(cv::Mat &G_CT2, const std::vector<cv::Mat> &markers, const cv::Mat &g_BM, const cv::Mat &solvePnpCamBoard );
    void testCamToBoard3();

    /**
     * @brief Data for optimizations
     */
    std::vector<cv::Mat> G_M1_M0;
    std::vector<cv::Mat> G_CB;

    /**
     * @brief validation function for optimization results
     * @param G_CM0
     */
    void validationFunc(const cv::Mat &G_CM0);

    /**
     * @brief The object function for optimization
     * @param vec_CM0
     * @return
     */
    double errorFunc(const cv::Mat &vec_CM0);
    /**
     * @brief Main optimization function
     * @param G_CM0
     */
    void particleSwarmOptimization(const cv::Mat &G_CM0);
    /**
     * @brief This is data generator takes care of the G_M1_M0 and G_CB, this one needs to be replaced
     */
    void stupidGenerator();
    /**
     * @brief A boundry check function according to the searching range
     * @param particle
     */
    void boundaryCheck(cv::Mat &particle);
    /**
     * @brief Convert a 6x1 vector to a SE(3) matrix
     * @param vec_6_1
     * @param outputGeometry
     */
    void computeSE3(const cv::Mat &vec_6_1, cv::Mat &outputGeometry);

    /**
     * @brief Random number generators
     */
    double randomNumber(double stdev, double mean);
    double randomNum(double min, double max);
    
};

#endif
