#!/usr/bin/env python


import rospy
#import roslib
#roslib.load_manifest('differential_drive')
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Range
#############################################################################
class SonarRing:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("SonarRing")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = rospy.get_param('~rate',30.0)  # the rate at which to publish the transform
        self.min_range = 0.03
        self.max_range = 0.89
        self.field_of_view = 0.26
        
        # message init
        self.dist = Int16MultiArray() 
        self.dist.data = [400, 400, 400, 400, 400, 400, 400, 400]
                
        self.f_ultra = Range()
        self.f_ultra.header.frame_id = "base_us_front"
        self.f_ultra.radiation_type = 0
        self.f_ultra.field_of_view = self.field_of_view
        self.f_ultra.min_range = self.min_range    
        self.f_ultra.max_range = self.max_range

        self.fr_ultra = Range()
        self.fr_ultra.header.frame_id = "base_us_front_right"
        self.fr_ultra.radiation_type = 0
        self.fr_ultra.field_of_view = self.field_of_view
        self.fr_ultra.min_range = self.min_range    
        self.fr_ultra.max_range = self.max_range

        self.r_ultra = Range()
        self.r_ultra.header.frame_id = "base_us_right"
        self.r_ultra.radiation_type = 0
        self.r_ultra.field_of_view = self.field_of_view
        self.r_ultra.min_range = self.min_range    
        self.r_ultra.max_range = self.max_range

        self.br_ultra = Range()
        self.br_ultra.header.frame_id = "base_us_back_right"
        self.br_ultra.radiation_type = 0
        self.br_ultra.field_of_view = self.field_of_view
        self.br_ultra.min_range = self.min_range    
        self.br_ultra.max_range = self.max_range

        self.b_ultra = Range()
        self.b_ultra.header.frame_id = "base_us_back"
        self.b_ultra.radiation_type = 0
        self.b_ultra.field_of_view = self.field_of_view
        self.b_ultra.min_range = self.min_range    
        self.b_ultra.max_range = self.max_range

        self.bl_ultra = Range()
        self.bl_ultra.header.frame_id = "base_us_back_left"
        self.bl_ultra.radiation_type = 0
        self.bl_ultra.field_of_view = self.field_of_view
        self.bl_ultra.min_range = self.min_range    
        self.bl_ultra.max_range = self.max_range

        self.l_ultra = Range()
        self.l_ultra.header.frame_id = "base_us_left"
        self.l_ultra.radiation_type = 0
        self.l_ultra.field_of_view = self.field_of_view
        self.l_ultra.min_range = self.min_range    
        self.l_ultra.max_range = self.max_range

        
        self.fl_ultra = Range()
        self.fl_ultra.header.frame_id = "base_us_front_left"
        self.fl_ultra.radiation_type = 0
        self.fl_ultra.field_of_view = self.field_of_view
        self.fl_ultra.min_range = self.min_range    
        self.fl_ultra.max_range = self.max_range

        # subscriptions
        rospy.Subscriber("/sonar", Int16MultiArray, self.Callback)
        # publishers
        self.front = rospy.Publisher("f_ultra", Range, queue_size=10) #original queue_size=10 , demasiado para arduino si se reduce el valor se pierde presicion
        self.front_right = rospy.Publisher("fr_ultra", Range, queue_size=10) #original queue_size=10
        self.right = rospy.Publisher("r_ultra", Range, queue_size=10) #original queue_size=10
        self.back_right = rospy.Publisher("br_ultra", Range, queue_size=10) #original queue_size=10
        self.back = rospy.Publisher("b_ultra", Range, queue_size=10) #original queue_size=10
        self.back_left = rospy.Publisher("bl_ultra", Range, queue_size=10) #original queue_size=10
        self.left = rospy.Publisher("l_ultra", Range, queue_size=10) #original queue_size=10
        self.front_left = rospy.Publisher("fl_ultra", Range, queue_size=10) #original queue_size=10
       
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################n        
        
        self.f_ultra.header.stamp = rospy.Time.now()
        self.f_ultra.range = self.dist.data[0]/100.00

        self.front.publish(self.f_ultra)
        
        self.fr_ultra.header.stamp = rospy.Time.now()
        self.fr_ultra.range = self.dist.data[1]/100.00
        
        self.front_right.publish(self.fr_ultra)

        self.r_ultra.header.stamp = rospy.Time.now()
        self.r_ultra.range = self.dist.data[2]/100.00

        self.right.publish(self.r_ultra)

        self.br_ultra.header.stamp = rospy.Time.now()
        self.br_ultra.range = self.dist.data[3]/100.00

        self.back_right.publish(self.br_ultra)

        self.b_ultra.header.stamp = rospy.Time.now()
        self.b_ultra.range = self.dist.data[4]/100.00

        self.back.publish(self.b_ultra)

        self.bl_ultra.header.stamp = rospy.Time.now()
        self.bl_ultra.range = self.dist.data[5]/100.00

        self.back_left.publish(self.bl_ultra)

        self.l_ultra.header.stamp = rospy.Time.now()
        self.l_ultra.range = self.dist.data[6]/100.00

        self.left.publish(self.l_ultra)

        self.fl_ultra.header.stamp = rospy.Time.now()
        self.fl_ultra.range = self.dist.data[7]/100.00
        
        self.front_left.publish(self.fl_ultra)
           
    #############################################################################
    def Callback(self, msg):
    #############################################################################
        self.dist = msg                                 
        
    #############################################################################

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    UltraSound = SonarRing()
    UltraSound.spin()
    
    
   
