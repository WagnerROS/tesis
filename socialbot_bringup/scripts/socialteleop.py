#!/usr/bin/python3
#general imports

from __future__ import print_function
import threading
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from std_msgs.msg import String

#first message of the program
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

#regular keys getted
moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'k':(0,0,0,0),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0)
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

#subscriber class
class SubscribeThread(threading.Thread):
    def __init__(self, rate):
        super(SubscribeThread, self).__init__()
        self.subscriber = rospy.Subscriber("data_teleop", String, self.callback)
        self.key = " "
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 10.0 / rate
        else:
            self.timeout = None
        self.start()

    def callback(self, data1):
        self.key = data1.data
        #print(data1.data)
        

    #update function
    def update (self, data):
        self.condition.acquire()
        self.key = 0 
        #notify publish thread that we have a new message
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0)
        self.join()
    
    def run(self):
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)
            self.condition.release()
            #Publish
            self.subscriber


#general publish class 
class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/cmd_vel_teleop', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 10.0 / rate
        else:
            self.timeout = None

        self.start()

    #function to wait until cmd_vel connect
    def wait_for_subscribers(self):
        i=0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.25)
            i += 1
            i = i % 5
        
        if rospy.is_shutdown():
            raise Exception("Got shutdown before subscribers connect")

    #update function
    def update (self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        #notify publish thread that we have a new message
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()
    
    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)

            #copy the state into twist message form
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            #Publish
            if sub_thread.key != "0" :
                self.publisher.publish(twist)

        
        #Publish a stop message when thread exits
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


if __name__ == "__main__":
    global mensaje, key
    key = " "
    rospy.init_node('teleop_remote')
    speed = rospy.get_param("~speed", 0.15)
    turn = rospy.get_param("~turn", 0.35)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    
    if key_timeout == 0.0:
        key_timeout = None
    
    pub_thread = PublishThread(repeat)
    sub_thread = SubscribeThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    key = 0

    try:
        pub_thread.wait_for_subscribers()
        sub_thread.update(key)
        pub_thread.update(x, y, z, th, speed, turn)
        print(msg)
        print(vels(speed,turn))
        while not rospy.is_shutdown():
            print(sub_thread.key) 
            if sub_thread.key in moveBindings.keys():               
                x = moveBindings[sub_thread.key][0]
                y = moveBindings[sub_thread.key][1]
                z = moveBindings[sub_thread.key][2]
                th = moveBindings[sub_thread.key][3]
                pub_thread.update(x, y, z, th, speed, turn)
                #print(x)
            elif sub_thread.key in speedBindings.keys():
                speed = speed * speedBindings[sub_thread.key][0]
                turn = turn * speedBindings[sub_thread.key][1]
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15 
            else:
                #skip updating cmd_vel if the key timeout and robot already stopped
                if sub_thread.key == ' ' and x == 0 and y ==0 and z ==0 and th ==0:
                    continue
                
                x = 0
                y = 0
                z = 0
                th = 0
            
    except Exception as e:
        print(e)
    
    finally:
        sub_thread.stop()
        pub_thread.stop()

        