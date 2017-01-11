#ifndef DAVINCI_CALIBRATOR_H
#define DAVINCI_CALIBRATOR_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.hpp"

 class DavinciCalibrator{
 private:
    
    ros::NodeHandle nh_;

    int corner_size;

    cv::Mat left_rvec, left_tvec;
    cv::Mat right_rvec, right_tvec;

    cv::Mat left_g, right_g;

    std::vector<cv::Point2f> left_corner_coordinates;
    std::vector<cv::Point2f> right_corner_coordinates;

 public:
 	 /*
     * This constructor uses an input charactor to set who it subscribes to
     * It also then initializes the publisher.
     */
    DavinciCalibrator(ros::NodeHandle* nodehandle);

    ros::Subscriber corner_size_subscriber;
    ros::Subscriber leftcorner_subscriber;
    ros::Subscriber rightcorner_subscriber;

    void cornerSizeCB(const std_msgs::Int32::ConstPtr& cornerSizeData);
    void leftcornerCB(const std_msgs::Float32MultiArray::ConstPtr& leftcornerData);
    void rightcornerCB(const std_msgs::Float32MultiArray::ConstPtr& rightcornerData);

    void computeCameraPose(const std::vector<cv::Point2f> corner_coords, cv::Mat &cam_pose );

};

#endif