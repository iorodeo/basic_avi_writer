#!/usr/bin/env python
"""
basic_avi_writer.py

A basic ROS node for recording AVI files.

Usage:  rosrun basic_avi_writer avi_writer_node <image_topic> <avi filename>

"""
from __future__ import print_function
import roslib
roslib.load_manifest('basic_avi_writer')
import rospy
import sys
import os
import os.path
import cv
import threading
import math

# Messages
from sensor_msgs.msg import Image
from cv_bridge.cv_bridge import CvBridge 
from cv_bridge.cv_bridge import CvBridgeError

class AVI_Writer(object):

    def __init__(self,image_topic,filename):

        self.image_topic = image_topic
        self.filename = filename
        self.frame_count = 0
        self.recording = False
        self.writer = None 
        self.cv_image_size = None
        self.framerate = None 
        self.last_stamp = None

        self.lock = threading.Lock()
        self.bridge = CvBridge()
        rospy.init_node('avi_writer')
        rospy.on_shutdown(self.stop_video_writer)

        # Subscribe to image topic 
        self.image_sub = rospy.Subscriber(self.image_topic,Image,self.image_handler)


    def image_handler(self,data): 
        """
        Writes frames to avi file.
        """
        if self.framerate is not None:
            # Convert to opencv image and then to ipl_image
            cv_image = self.bridge.imgmsg_to_cv(data,'bgr8')
            ipl_image = cv.GetImage(cv_image)

            cv_image_size = cv.GetSize(cv_image)
            if self.frame_count == 0:
                self.cv_image_size = cv_image_size
                self.start_video_writer()

            if cv_image_size != self.cv_image_size:
                # Abort recording - image size changed.
                pass

            if self.recording:
                # Write video frame
                cv.WriteFrame(self.writer,ipl_image)
                self.frame_count += 1
        else:
            stamp = data.header.stamp
            if self.last_stamp is not None:
                diff_stamp = stamp - self.last_stamp
                dt = diff_stamp.to_sec()
                self.framerate = 1/dt
            self.last_stamp = stamp



    def start_video_writer(self): 
        self.writer = cv.CreateVideoWriter(
               self.filename,
               cv.CV_FOURCC('D','I','V','X'),
               self.framerate,
               self.cv_image_size,
               ) 
        if self.writer is None:
            raise IOError, 'unable to create video writer'
        self.recording = True

    def stop_video_writer(self):
        with self.lock:
            self.recording = False
            del self.writer
            self.writer = None

    def run(self):
        rospy.spin()
       
# -----------------------------------------------------------------------------
if __name__ == '__main__':

    image_topic = sys.argv[1]
    filename = sys.argv[2]

    node = AVI_Writer(image_topic, filename)
    node.run()



