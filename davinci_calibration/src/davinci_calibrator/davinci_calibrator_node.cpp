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

int main(int argc, char **argv) {

    ros::init(argc, argv, "davinci_calibrator");
    ros::NodeHandle nh;

   	DavinciCalibrator calibrator(&nh);

//    /*******Testing*********/
//     cv::Mat seed_G_CM = (cv::Mat_<double>(4,4) << 0.1097918037424463, -0.9935220554546673, 0.02932038806212234, -0.001536981165755624,
//     0.993880576851076, 0.1100958731669279, 0.008960896706521795, 0.01123477036806568,
//     -0.01213090223987345, 0.0281571311881208, 0.9995298980891474, -0.179828180848247,
//     0, 0, 0, 1);

//    calibrator.stupidGenerator();  //get vectors
//      calibrator.particleSwarmOptimization(seed_G_CM);

    // calibrator.stupidGenerator();  //get vectors
    // cv::Mat seed_G_CM = (cv::Mat_<double>(4,4) << 0.115854435066732, -0.9930306921941986, 0.02162855140038672, 0.001063018834244381,
    // 0.993196398521018, 0.1160769784091117, 0.009330007946915353, 0.01383477036806569,
    // -0.01177556114362578, 0.02040047655623282, 0.9997225398659526, -0.179208180848247,
    // 0, 0, 0, 1);

    // calibrator.validationFunc(seed_G_CM);

   ros::Duration(1).sleep(); ///wait for camera and polaris get ready

   while(nh.ok()){

       ros::spinOnce();

        calibrator.stereoCameraCalibration();

       ros::Duration(1).sleep();  //wait for publishing data
   }

    return 0; // should never get here, unless roscore dies
}
