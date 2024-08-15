#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import threading

class PersonDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/processed/rgb/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/processed/depth/points", PointCloud2, self.depth_callback)
        self.coord_sub = rospy.Subscriber("/people_detector/detection_coordinates", Point, self.coord_callback)
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        self.points = None
        self.center_coords = None
        self.timer = threading.Timer(1.0, self.clear_marker)
        self.timer.start()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #cv2.imshow('Image', cv_image)
            #cv2.waitKey(1)
        except Exception as e:
            print(e)

    def depth_callback(self, data):
        self.points = data

    def coord_callback(self, data):
        self.center_coords = data
        if self.points is not None and self.center_coords is not None:
            point = self.get_depth_from_point_cloud(int(self.center_coords.x), int(self.center_coords.y))
            if point:
                self.publish_marker(point[0], point[1], point[2])
                self.reset_timer()

    def get_depth_from_point_cloud(self, x, y):
        # Obtener la profundidad del punto (x, y) de la nube de puntos
        gen = point_cloud2.read_points(self.points, field_names=("x", "y", "z"), skip_nans=True)
        closest_points = []
        for p in gen:
            # Aquí podemos buscar puntos alrededor del centro de la detección
            u = int(p[0] * 800.0 / p[2] + 319.5)
            v = int(p[1] * 800.0 / p[2] + 239.5)
            if abs(u - x) <= 2 and abs(v - y) <= 2:
                closest_points.append(p)

        if closest_points:
            avg_x = sum(p[0] for p in closest_points) / len(closest_points)
            avg_y = sum(p[1] for p in closest_points) / len(closest_points)
            avg_z = sum(p[2] for p in closest_points) / len(closest_points)
            return (avg_x, avg_y, avg_z)
        return None

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "person_detector"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = -x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        print(f"Persona detectada a una distancia de {-x} y {y} metros")
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def clear_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "person_detector"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)
        self.reset_timer()

    def reset_timer(self):
        if self.timer.is_alive():
            self.timer.cancel()
        self.timer = threading.Timer(1.0, self.clear_marker)
        self.timer.start()

if __name__ == '__main__':
    rospy.init_node('person_detector', anonymous=True)
    pd = PersonDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
