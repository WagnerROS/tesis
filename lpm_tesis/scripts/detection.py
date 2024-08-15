#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2

class PersonDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/processed/rgb/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/processed/depth/points", PointCloud2, self.depth_callback)
        self.person_cascade = cv2.CascadeClassifier('/home/wagner/KRONOS_PRUEBAS/src/lpm_robot/detect/haarcascade_lowerbody.xml')
        self.points = None

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            persons = self.person_cascade.detectMultiScale(gray, 1.1, 4)
            
            for (x, y, w, h) in persons:
                # Filtrar detecciones para centrarse desde el torso hacia abajo
                if y > cv_image.shape[0] // 3:
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    if self.points is not None:
                        # Obtener la distancia a la persona
                        point = self.get_depth_from_point_cloud(x + w//2, y + h//2)
                        if point:
                            print(f"Persona detectada a una distancia de {point[2]} metros")

            cv2.imshow('Image', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            print(e)

    def depth_callback(self, data):
        self.points = data

    def get_depth_from_point_cloud(self, x, y):
        # Obtener la profundidad del punto (x, y) de la nube de puntos
        gen = point_cloud2.read_points(self.points, field_names=("x", "y", "z"), skip_nans=True)
        closest_point = None
        min_dist = float('inf')
        for p in gen:
            # Aquí podemos buscar el punto más cercano al centro de la detección
            dist = (p[0] - x)**2 + (p[1] - y)**2
            if dist < min_dist:
                min_dist = dist
                closest_point = p
        return closest_point

if __name__ == '__main__':
    rospy.init_node('person_detector', anonymous=True)
    pd = PersonDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
