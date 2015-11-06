#!/usr/bin/env python
from xm_msgs.srv import xm_Pan
import rospy
from dynamixel_controllers.srv import *
from sensor_msgs.msg import JointState, RegionOfInterest
import math
from std_msgs.msg import Float64

class xm_pan_start():

    def __init__(self):
        self.pan_srv = rospy.Service('pan_srv', xm_Pan, self.pan_srv_handle)
        rospy.loginfo("init have completed")
        self.pan_pos_pub = rospy.Publisher('head_pan_joint/command', Float64, queue_size=10)

    def pan_srv_handle(self, request):
        if request.start == True:
            servo_position = 0.0
            self.pan_pos_pub.publish(servo_position)
            rospy.loginfo("send the pan position to the topic successfully")


if __name__ == '__main__':
    try:
        rospy.init_node('pan_server')
        xm_pan_start()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("hehe!!!!")
        pass





