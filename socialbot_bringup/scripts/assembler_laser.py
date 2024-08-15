#!/usr/bin/env python
import rospy
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud2

rospy.init_node("assembler_client")
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub = rospy.Publisher("/laser_pointcloud",PointCloud2, queue_size=1)
r = rospy.Rate(30)

while not rospy.is_shutdown():
    try:
        now = rospy.get_rostime()
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        pub.publish (resp.cloud)

    except rospy.ServiceException, e:
        print "Service call fail"

    r.sleep()