#!/usr/bin/python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import sensor_msgs.msg
import sensor_msgs.srv
import message_filters
from message_filters import ApproximateTimeSynchronizer

import os
from collections import deque
import threading
import functools
import time

import cv2
import numpy

# from davinci_calibration.msg import corners
# from davinci_calibration.msg import points
# from davinci_calibration.msg import intrinsic_param

from camera_calibration.calibrator import MonoCalibrator, StereoCalibrator, ChessboardInfo, Patterns
from std_msgs.msg import String
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Int32

class DisplayThread(threading.Thread):
    """
    Thread that displays the current images
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and 
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, queue, opencv_calibration_node):
        threading.Thread.__init__(self)
        self.queue = queue
        self.opencv_calibration_node = opencv_calibration_node

    def run(self):
        cv2.namedWindow("display", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("display", self.opencv_calibration_node.on_mouse)
        cv2.createTrackbar("scale", "display", 0, 100, self.opencv_calibration_node.on_scale)
        while True:
            # wait for an image (could happen at the very beginning when the queue is still empty)
            while len(self.queue) == 0:
                time.sleep(0.1)
            im = self.queue[0]
            cv2.imshow("display", im)
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                rospy.signal_shutdown('Quit')
            elif k == ord('s'):
                self.opencv_calibration_node.screendump(im)

class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            # wait for an image (could happen at the very beginning when the queue is still empty)
            while len(self.queue) == 0:
                time.sleep(0.1)
            self.function(self.queue[0])


class CalibrationNode:
    def __init__(self, boards, service_check = True, synchronizer = message_filters.TimeSynchronizer, flags = 0,
                 pattern=Patterns.Chessboard, camera_name='', checkerboard_flags = 0):
        if service_check:
            # assume any non-default service names have been set.  Wait for the service to become ready
            for svcname in ["camera", "left_camera", "right_camera"]:
                remapped = rospy.remap_name(svcname)
                if remapped != svcname:
                    fullservicename = "%s/set_camera_info" % remapped
                    print("Waiting for service", fullservicename, "...")
                    try:
                        rospy.wait_for_service(fullservicename, 5)
                        print("OK")
                    except rospy.ROSException:
                        print("Service not found")
                        rospy.signal_shutdown('Quit')

        self._boards = boards
        self._calib_flags = flags
        self._checkerboard_flags = checkerboard_flags
        self._pattern = pattern
        self._camera_name = camera_name
        lsub = message_filters.Subscriber('left', sensor_msgs.msg.Image)
        rsub = message_filters.Subscriber('right', sensor_msgs.msg.Image)
        ts = synchronizer([lsub, rsub], 4)
        ts.registerCallback(self.queue_stereo)

        msub = message_filters.Subscriber('image', sensor_msgs.msg.Image)
        msub.registerCallback(self.queue_monocular)
        
        self.set_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("camera"),
                                                          sensor_msgs.srv.SetCameraInfo)
        self.set_left_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("left_camera"),
                                                               sensor_msgs.srv.SetCameraInfo)
        self.set_right_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("right_camera"),
                                                                sensor_msgs.srv.SetCameraInfo)

        self.q_mono = deque([], 1)
        self.q_stereo = deque([], 1)

        self.c = None

        # add publisher when received the corner information TODO:
        self.left_pub = rospy.Publisher('/left_corners', Float32MultiArray, queue_size = 100)
        self.right_pub = rospy.Publisher('/right_corners', Float32MultiArray, queue_size = 100)

        self.lcorner_size_pub = rospy.Publisher('/get_corner_size_l', Int32, queue_size = 100)
        self.rcorner_size_pub = rospy.Publisher('/get_corner_size_r', Int32, queue_size = 100)
        # self.intrinsic_pub = rospy.Publisher('/camera_intrinsics', intrinsic_param, queue_size = 100)

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

        sth = ConsumerThread(self.q_stereo, self.handle_stereo)  
        sth.setDaemon(True)
        sth.start()

    def redraw_stereo(self, *args):
        pass
    def redraw_monocular(self, *args):
        pass

    def queue_monocular(self, msg):
        self.q_mono.append(msg)

    def queue_stereo(self, lmsg, rmsg):
        self.q_stereo.append((lmsg, rmsg))

    def convert_point2f_to_list(self, input_corners):  # change cv::Point2f(numpy.ndarray in python) to list

        corner_Xs = input_corners[:,:,0]
        corner_Ys = input_corners[:,:,1]
        new_corner = zip(corner_Xs, corner_Ys)

        corner_tuple = tuple(new_corner)
        return corner_tuple

    def handle_monocular(self, msg):
        if self.c == None:
            if self._camera_name:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._pattern, name=self._camera_name,
                                        checkerboard_flags=self._checkerboard_flags)
            else:
                self.c = MonoCalibrator(self._boards, self._calib_flags, self._pattern,
                                        checkerboard_flags=self.checkerboard_flags)

        # This should just call the MonoCalibrator
        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.scrib.shape[1]
        self.redraw_monocular(drawable)

    def handle_stereo(self, msg):
        if self.c == None:
            if self._camera_name:
                self.c = StereoCalibrator(self._boards, self._calib_flags, self._pattern, name=self._camera_name,
                                          checkerboard_flags=self._checkerboard_flags)
            else:
                self.c = StereoCalibrator(self._boards, self._calib_flags, self._pattern,
                                          checkerboard_flags=self._checkerboard_flags)

        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.lscrib.shape[1] + drawable.rscrib.shape[1]
        self.redraw_stereo(drawable)

        # added lines TODO:   
        # left_temp = points()   #get msg type from corners
        # right_temp = points()   #get msg type from corners
        # corner_msgs = corners()
                
        lcorner_size_msg = Int32()
        rcorner_size_msg = Int32()

# The mat is giving the ready-to-publish corner coordinates, push everything in a mat TODO:
        if drawable.lcorner is not None:

            lcorner_size = len(drawable.lcorner)

            # publish the coner size for computation
            lcorner_size_msg.data = lcorner_size
            self.lcorner_size_pub.publish(lcorner_size_msg)

            left_mat = Float32MultiArray()
            left_mat.layout.dim.append(MultiArrayDimension())
            left_mat.layout.dim.append(MultiArrayDimension())
            left_mat.layout.dim[0].label = "row"
            left_mat.layout.dim[1].label = "col"
            left_mat.layout.dim[0].size = lcorner_size    # if there are 9 points give 9 dimension
            left_mat.layout.dim[1].size = lcorner_size
            left_mat.layout.dim[0].stride = lcorner_size
            left_mat.layout.dim[1].stride = lcorner_size
            left_mat.layout.data_offset = 0
            left_mat.data = [0]*lcorner_size*lcorner_size
            dstride1 = left_mat.layout.dim[1].stride
            offset = left_mat.layout.data_offset

            print("left")  # debug
            print(drawable.lcorner)  # debug
            i_l = 0
            for temp_left in drawable.lcorner:
                temp_x = temp_left[0,0]
                temp_y = temp_left[0,1]
                left_mat.data[offset + i_l + dstride1*0] = temp_x
                left_mat.data[offset + i_l + dstride1*1] = temp_y
                i_l += 1

            self.left_pub.publish(left_mat)         
        else:
            print()
            print("No LEFT corner coordinates")
            print()

        if drawable.rcorner is not None:

            rcorner_size = len(drawable.rcorner)

            rcorner_size_msg.data = rcorner_size
            self.rcorner_size_pub.publish(rcorner_size_msg)

            right_mat = Float32MultiArray()
            right_mat.layout.dim.append(MultiArrayDimension())
            right_mat.layout.dim.append(MultiArrayDimension())
            right_mat.layout.dim[0].label = "row"
            right_mat.layout.dim[1].label = "col"
            right_mat.layout.dim[0].size = rcorner_size
            right_mat.layout.dim[1].size = rcorner_size
            right_mat.layout.dim[0].stride = rcorner_size
            right_mat.layout.dim[1].stride = rcorner_size
            right_mat.layout.data_offset = 0
            right_mat.data = [0]*rcorner_size*rcorner_size
            dstride1 = right_mat.layout.dim[1].stride
            offset = right_mat.layout.data_offset

            print("right")  # debug
            print(drawable.rcorner)   #debug
            i_r = 0
            for temp_right in drawable.rcorner:
                temp_x = temp_right[0,0]
                temp_y = temp_right[0,1]
                right_mat.data[offset + i_r + dstride1*0] = temp_x
                right_mat.data[offset + i_r + dstride1*1] = temp_y
                i_r += 1

            self.right_pub.publish(right_mat)
                           
        else:
            print()
            print("No RIGHT corner coordinates")
            print()
           
        # tryp to publisht the intrinsic matrix for left and right camera
        # if self.c.is_mono:
        #     pass
        # else:
        #     stereo_cam_info = self.c.as_message()

        #     left_response = self.set_left_camera_info_service(stereo_cam_info[0])
        #     left_intrinsic = left_response.camera_info.K

        #     right_response = self.set_right_camera_info_service(stereo_cam_info[1])
        #     right_intrinsic = right_response.camera_info.K

        #     camera_intrinsics_msg.left_intirnsic = left_intrinsic
        #     camera_intrinsics_msg.right_intirnsic = right_intrinsic

        #     self.intrinsic_pub.publish(camera_intrinsics_msg)

        rospy.sleep(0.1)         
      
    # end of the handle_strero

    def check_set_camera_info(self, response):
        if response.success:
            return True

        for i in range(10):
            print("!" * 80)
        print()
        print("Attempt to set camera info failed: " + response.status_message)
        print()
        for i in range(10):
            print("!" * 80)
        print()
        rospy.logerr('Unable to set camera info for calibration. Failure message: %s' % response.status_message)
        return False

    def do_upload(self):
        self.c.report()
        print(self.c.ost())
        info = self.c.as_message()

        rv = True
        if self.c.is_mono:
            response = self.set_camera_info_service(info)
            rv = self.check_set_camera_info(response)
        else:
            response = self.set_left_camera_info_service(info[0])
            rv = rv and self.check_set_camera_info(response)
            response = self.set_right_camera_info_service(info[1])
            rv = rv and self.check_set_camera_info(response)
        return rv


class OpenCVCalibrationNode(CalibrationNode):
    """ Calibration node with an OpenCV Gui """
    FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SCALE = 0.6
    FONT_THICKNESS = 2

    def __init__(self, *args, **kwargs):

        CalibrationNode.__init__(self, *args, **kwargs)

        self.queue_display = deque([], 1)
        self.display_thread = DisplayThread(self.queue_display, self)
        self.display_thread.setDaemon(True)
        self.display_thread.start()

    @classmethod
    def putText(cls, img, text, org, color = (0,0,0)):
        cv2.putText(img, text, org, cls.FONT_FACE, cls.FONT_SCALE, color, thickness = cls.FONT_THICKNESS)

    @classmethod
    def getTextSize(cls, text):
        return cv2.getTextSize(text, cls.FONT_FACE, cls.FONT_SCALE, cls.FONT_THICKNESS)[0]

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.displaywidth < x:
            if self.c.goodenough:
                if 180 <= y < 280:
                    self.c.do_calibration()
            if self.c.calibrated:
                if 280 <= y < 380:
                    self.c.do_save()
                elif 380 <= y < 480:
                    # Only shut down if we set camera info correctly, #3993
                    if self.do_upload():
                        rospy.signal_shutdown('Quit')

    def on_scale(self, scalevalue):
        if self.c.calibrated:
            self.c.set_alpha(scalevalue / 100.0)

    def button(self, dst, label, enable):
        dst.fill(255)
        size = (dst.shape[1], dst.shape[0])
        if enable:
            color = (155, 155, 80)
        else:
            color = (224, 224, 224)
        cv2.circle(dst, (size[0] / 2, size[1] / 2), min(size) / 2, color, -1)
        (w, h) = self.getTextSize(label)
        self.putText(dst, label, ((size[0] - w) / 2, (size[1] + h) / 2), (255,255,255))

    def buttons(self, display):
        x = self.displaywidth
        self.button(display[180:280,x:x+100], "CALIBRATE", self.c.goodenough)
        self.button(display[280:380,x:x+100], "SAVE", self.c.calibrated)
        self.button(display[380:480,x:x+100], "COMMIT", self.c.calibrated)

    def y(self, i):
        """Set up right-size images"""
        return 30 + 40 * i
        
    def screendump(self, im):
        i = 0
        while os.access("/tmp/dump%d.png" % i, os.R_OK):
            i += 1
        cv2.imwrite("/tmp/dump%d.png" % i, im)

    def redraw_monocular(self, drawable):
        height = drawable.scrib.shape[0]
        width = drawable.scrib.shape[1]

        display = numpy.zeros((max(480, height), width + 100, 3), dtype=numpy.uint8)
        display[0:height, 0:width,:] = drawable.scrib
        display[0:height, width:width+100,:].fill(255)


        self.buttons(display)
        if not self.c.calibrated:
            if drawable.params:
                 for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w,_) = self.getTextSize(label)
                    self.putText(display, label, (width + (100 - w) / 2, self.y(i)))
                    color = (0,255,0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv2.line(display,
                            (int(width + lo * 100), self.y(i) + 20),
                            (int(width + hi * 100), self.y(i) + 20),
                            color, 4)

        else:
            self.putText(display, "lin.", (width, self.y(0)))
            linerror = drawable.linear_error
            if linerror < 0:
                msg = "?"
            else:
                msg = "%.2f" % linerror
                #print "linear", linerror
            self.putText(display, msg, (width, self.y(1)))

        self.queue_display.append(display)

    def redraw_stereo(self, drawable):
        height = drawable.lscrib.shape[0]
        width = drawable.lscrib.shape[1]

        display = numpy.zeros((max(480, height), 2 * width + 100, 3), dtype=numpy.uint8)
        display[0:height, 0:width,:] = drawable.lscrib
        display[0:height, width:2*width,:] = drawable.rscrib
        display[0:height, 2*width:2*width+100,:].fill(255)

        self.buttons(display)

        if not self.c.calibrated:
            if drawable.params:
                for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w,_) = self.getTextSize(label)
                    self.putText(display, label, (2 * width + (100 - w) / 2, self.y(i)))
                    color = (0,255,0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv2.line(display,
                            (int(2 * width + lo * 100), self.y(i) + 20),
                            (int(2 * width + hi * 100), self.y(i) + 20),
                            color, 4)

        else:
            self.putText(display, "epi.", (2 * width, self.y(0)))
            if drawable.epierror == -1:
                msg = "?"
            else:
                msg = "%.2f" % drawable.epierror
            self.putText(display, msg, (2 * width, self.y(1)))
            # TODO dim is never set anywhere. Supposed to be observed chessboard size?
            if drawable.dim != -1:
                self.putText(display, "dim", (2 * width, self.y(2)))
                self.putText(display, "%.3f" % drawable.dim, (2 * width, self.y(3)))

        self.queue_display.append(display)


def main():
    from optparse import OptionParser, OptionGroup
    parser = OptionParser("%prog --size SIZE1 --square SQUARE1 [ --size SIZE2 --square SQUARE2 ]",
                          description=None)
    parser.add_option("-c", "--camera_name",
                     type="string", default='narrow_stereo',
                     help="name of the camera to appear in the calibration file")
    group = OptionGroup(parser, "Chessboard Options",
                        "You must specify one or more chessboards as pairs of --size and --square options.")
    group.add_option("-p", "--pattern",
                     type="string", default="chessboard",
                     help="calibration pattern to detect - 'chessboard', 'circles', 'acircles'")
    group.add_option("-s", "--size",
                     action="append", default=[],
                     help="chessboard size as NxM, counting interior corners (e.g. a standard chessboard is 7x7)")
    group.add_option("-q", "--square",
                     action="append", default=[],
                     help="chessboard square size in meters")
    parser.add_option_group(group)
    group = OptionGroup(parser, "ROS Communication Options")
    group.add_option("--approximate",
                     type="float", default=0.0,
                     help="allow specified slop (in seconds) when pairing images from unsynchronized stereo cameras")
    group.add_option("--no-service-check",
                     action="store_false", dest="service_check", default=True,
                     help="disable check for set_camera_info services at startup")
    parser.add_option_group(group)
    group = OptionGroup(parser, "Calibration Optimizer Options")
    group.add_option("--fix-principal-point",
                     action="store_true", default=False,
                     help="fix the principal point at the image center")
    group.add_option("--fix-aspect-ratio",
                     action="store_true", default=False,
                     help="enforce focal lengths (fx, fy) are equal")
    group.add_option("--zero-tangent-dist",
                     action="store_true", default=False,
                     help="set tangential distortion coefficients (p1, p2) to zero")
    group.add_option("-k", "--k-coefficients",
                     type="int", default=2, metavar="NUM_COEFFS",
                     help="number of radial distortion coefficients to use (up to 6, default %default)")
    group.add_option("--disable_calib_cb_fast_check", action='store_true', default=False,
                     help="uses the CALIB_CB_FAST_CHECK flag for findChessboardCorners")
    parser.add_option_group(group)
    options, args = parser.parse_args()

    if len(options.size) != len(options.square):
        parser.error("Number of size and square inputs must be the same!")
    
    if not options.square:
        options.square.append("0.108")
        options.size.append("8x6")

    boards = []
    for (sz, sq) in zip(options.size, options.square):
        size = tuple([int(c) for c in sz.split('x')])
        boards.append(ChessboardInfo(size[0], size[1], float(sq)))

    if options.approximate == 0.0:
        sync = message_filters.TimeSynchronizer
    else:
        sync = functools.partial(ApproximateTimeSynchronizer, slop=options.approximate)

    num_ks = options.k_coefficients

    calib_flags = 0
    if options.fix_principal_point:
        calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if options.fix_aspect_ratio:
        calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
    if options.zero_tangent_dist:
        calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if (num_ks > 3):
        calib_flags |= cv2.CALIB_RATIONAL_MODEL
    if (num_ks < 6):
        calib_flags |= cv2.CALIB_FIX_K6
    if (num_ks < 5):
        calib_flags |= cv2.CALIB_FIX_K5
    if (num_ks < 4):
        calib_flags |= cv2.CALIB_FIX_K4
    if (num_ks < 3):
        calib_flags |= cv2.CALIB_FIX_K3
    if (num_ks < 2):
        calib_flags |= cv2.CALIB_FIX_K2
    if (num_ks < 1):
        calib_flags |= cv2.CALIB_FIX_K1

    pattern = Patterns.Chessboard
    if options.pattern == 'circles':
        pattern = Patterns.Circles
    elif options.pattern == 'acircles':
        pattern = Patterns.ACircles
    elif options.pattern != 'chessboard':
        print('Unrecognized pattern %s, defaulting to chessboard' % options.pattern)

    if options.disable_calib_cb_fast_check:
        checkerboard_flags = 0
    else:
        checkerboard_flags = cv2.CALIB_CB_FAST_CHECK

    rospy.init_node('cameracalibrator')
    node = OpenCVCalibrationNode(boards, options.service_check, sync, calib_flags, pattern, options.camera_name,
                                 checkerboard_flags=checkerboard_flags)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
