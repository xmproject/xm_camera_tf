#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, RegionOfInterest, CameraInfo
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64
from math import radians
import thread
from people_msgs.msg import *
from xm_msgs.msg import *
from xm_msgs.srv import *
import math

class position():
    def __init__(self):
        rospy.init_node("position_test")
        rospy.on_shutdown(self.shutdown)

        rate = rospy.get_param("~rate", 100)
        r = rospy.Rate(rate)

        self.head_pan_joint = rospy.get_param('~head_pan_joint', 'head_pan_joint')

        self.joints = [self.head_pan_joint]

        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.2)

        self.joint_state = JointState()
        rospy.Subscriber('joint_states', JointState, self.update_joint_state)

        while not rospy.is_shutdown():
            rospy.sleep(1)

            rospy.loginfo("waiting for joint_states")
            rospy.wait_for_message('joint_states', JointState)

            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

            print 'A'
            print current_pan
            print 'B'
            print current_pan * 180 / math.pi

    def update_joint_state(self, msg):
        self.joint_state = msg

    def shutdown(self):
        rospy.loginfo("over le")

if __name__ == '__main__':
    position()
    rospy.spin()