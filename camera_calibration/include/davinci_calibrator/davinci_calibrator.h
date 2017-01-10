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


 public:
 	 /*
     * This constructor uses an input charactor to set who it subscribes to
     * It also then initializes the publisher.
     */
    DavinciCalibrator(ros::NodeHandle* nodehandle);
    
     /*
     * The default constructor 
     */
    DavinciCalibrator();

    /*
     * The deconstructor 
     */
    ~DavinciCalibrator();

    ros::Subscriber corner_size_subscriber;
    ros::Subscriber leftcorner_subscriber;
    ros::Subscriber rightcorner_subscriber;




};

#endif