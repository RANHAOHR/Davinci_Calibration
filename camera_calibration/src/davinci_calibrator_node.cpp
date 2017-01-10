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


int main(int argc, char **argv) {

    ros::init(argc, argv, "davinci_calibrator");
    ros::NodeHandle nh; 

   	DavinciCalibrator calibrator(&nh);


    ros::spin();
    return 0; // should never get here, unless roscore dies
}


