# Davinci_Calibration

# This package is for daVinci Robot systems calibration

# Using Camera_calibration and NDI Polaris Vicra, requires the calibrated camera

 
To run davinci_launch(/cwru_davinci/davinci_launch)

'roslaunch davinci_launch davinci_endo.launch'





To run polaris_sensor(https://github.com/RANHAOHR/polaris_sensor):

'rosrun polaris_sensor polaris_sensor_node _roms:="$(rospack find polaris_sensor)/rom/polaris_file1.rom,$(rospack find polaris_sensor)/rom/polaris_file1.rom" _port:=/dev/ttyUSB0'




To run camera calibration node:

'rosrun davinci_calibration cameracalibrator.py --size 5x7 --square 0.015 right:=/davinci_endo/right/image_rect left:=/davinci_endo/left/image_rect left_camera:=/davinci_endo/unsynced/right right_camera:=/davinci_endo/unsynced/left'
'

This node send corners informations (sizes and positions in pixel frame) to cv::solvePnP for computing the camera extrinsic parameters.

NOTE: this node is subscribing the rectified image instead of the raw image, Please calibrate the camera before you send out the corner info: http://wiki.ros.org/camera_calibration. 



Or you can run all the nodes above by:

'roslaunch davinci_calibration davinci_calibration.launch'

-------

To run calibration node (calculations for all tranformations ):

'rosrun davinci_calibration davinci_calibrator_node'


--------

If you would like to run polaris alone and record the data, please run:

'roslaunch davinci_calibration polaris_launch.launch'


