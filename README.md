# Davinci_Calibration

# This package is for daVinci Robot systems calibration

# Using Camera_calibration and NDI Polaris Sensor
 
To run davinci_launch(/cwru_davinci/davinci_launch)

'roslaunch davinci_launch davinci_endo.launch'

To run polaris_sensor:

'rosrun polaris_sensor polaris_sensor_node _roms:="$(rospack find polaris_sensor)/rom/polaris_file1.rom,$(rospack find polaris_sensor)/rom/polaris_file1.rom" _port:=/dev/ttyUSB0'

To run camera calibration node (send the corners sizes and positions (in pixel frame), for cv::solvePnP):

'rosrun davinci_calibration cameracalibrator.py --size 5x7 --square 0.015 right:=/davinci_endo/right/image_raw left:=/davinci_endo/left/image_raw left_camera:=/davinci_endo/unsynced/right right_camera:=/davinci_endo/unsynced/left'
'

To run calibration node (calculations for all tranformations ):

'rosrun davinci_calibration davinci_calibrator_node'

Or you can run the nodes above in:

'roslaunch davinci_calibration davinci_calibration.launch'


