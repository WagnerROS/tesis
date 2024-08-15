#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import threading

class ImageDepthPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/points", PointCloud2, self.depth_callback)
        self.image_pub = rospy.Publisher("/processed/rgb/image_raw", Image, queue_size=10)
        self.depth_pub = rospy.Publisher("/processed/depth/points", PointCloud2, queue_size=10)
        self.current_image = None
        self.current_depth = None
        self.lock = threading.Lock()
        self.capture_rate = rospy.Rate(1)  # 1 Hz

        self.capture_thread = threading.Thread(target=self.publish_captures)
        self.capture_thread.start()

    def image_callback(self, data):
        with self.lock:
            self.current_image = data

    def depth_callback(self, data):
        with self.lock:
            self.current_depth = data

    def publish_captures(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.current_image is not None:
                    self.image_pub.publish(self.current_image)
                if self.current_depth is not None:
                    self.depth_pub.publish(self.current_depth)
            self.capture_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('image_depth_publisher', anonymous=True)
    idp = ImageDepthPublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
