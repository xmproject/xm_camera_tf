#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState, RegionOfInterest, CameraInfo
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64
from math import radians
import thread
from people_msgs.msg import *
from xm_msgs.msg import xm_Angle, xm_Task
import math

class tldTracker():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)

        rate = rospy.get_param("~rate", 100)
        r = rospy.Rate(rate)
        tick = 1.0 / rate

        speed_update_rate = rospy.get_param("~speed_update_rate", 40)
        speed_update_interval = 1.0 / speed_update_rate

        self.speed_update_threshold = rospy.get_param("~speed_update_threshold", 0.01)
        self.head_pan_joint = rospy.get_param('~head_pan_joint', 'head_pan_joint')
#        self.head_tilt_joint = rospy.get_param('~head_tilt_joint', 'head_tilt_joint')

        self.joints = [self.head_pan_joint]

        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.3)
        self.max_joint_speed = rospy.get_param('~max_joint_speed', 0.5)

        self.lead_target_angle = rospy.get_param('~lead_target_angle', 1.0)

        self.pan_threshold = rospy.get_param('~pan_threshold', 0.03)
        self.tilt_threshold = rospy.get_param('~tilt_threshold', 0.05)

        self.gain_pan = rospy.get_param("~gain_pan", 2.0)
        self.gain_tilt = rospy.get_param('~gain_tilt', 1.0)

        self.max_pan = rospy.get_param("~max_pan", radians(145))
        self.min_pan = rospy.get_param('~min_pan', radians(-145))
        self.max_tilt = rospy.get_param("~max_tilt", radians(90))
        self.min_tilt = rospy.get_param("~min_tilt", radians(-90))

        self.recenter_timeout = rospy.get_param('~recenter_timeout', 5)
        self.joint_state = JointState()
        rospy.Subscriber('joint_states', JointState, self.update_joint_state)
        self.pan_pub = rospy.Publisher('pan_angle', xm_Angle, queue_size=1000)
        while self.joint_state == JointState():
            rospy.sleep(1)

            self.init_servos()
            self.center_head_servos()

            self.target_visible = False

            target_lost_timer = 0.0

            speed_update_timer = 0.0

            pan_speed = tilt_speed = 0.0

            self.lock = thread.allocate_lock()

            rospy.loginfo("waiting for joint_states , tld topics")
            rospy.wait_for_message('joint_states', JointState)
            rospy.wait_for_message('people_position_estimation', PositionMeasurement)

            #    rospy.loginfo(self.image)
            #    rospy.sleep(1)

            rospy.tld_subscriber = rospy.Subscriber('people_position_estimation', PositionMeasurement, self.set_joint_cmd)

            rospy.loginfo("Ready to track target")

            while not rospy.is_shutdown():
                self.lock.acquire()

                try:
                    if not self.target_visible:
                        self.pan_speed = 0.0
#                        self.tilt_speed = 0.0

                        target_lost_timer += tick
                    else:
                        self.target_visible = False
                        target_lost_timer = 0.0

                    if target_lost_timer > self.recenter_timeout:
                        rospy.loginfo("cannot find target")
                        self.center_head_servos()
                        target_lost_timer = 0.0
                    else:
                        if speed_update_timer > speed_update_interval:
                            if abs(self.last_pan_speed - self.pan_speed) > self.speed_update_threshold:
                                self.set_servo_speed(self.head_pan_joint, self.pan_speed)
                                self.last_pan_speed = self.pan_speed

                            speed_update_timer = 0.0

                        if self.last_pan_position != self.pan_position:
                            self.set_servo_position(self.head_pan_joint, self.pan_position)
                            self.last_pan_position = self.last_pan_position

                    speed_update_timer += tick

                finally:
                    self.lock.release()

                r.sleep()

    def set_joint_cmd(self, msg):
        self.lock.acquire()
        try:
            if msg.pos is None:
                self.target_visible = False
                return

            self.target_visible = True

            pan_angle_ = xm_Angle()
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
            pan_angle_.xm_angle = current_pan
            self.pan_pub.publish(pan_angle_.xm_angle)

            if abs(msg.pos.y) > self.pan_threshold:
                self.pan_speed = min(self.max_joint_speed, max(0,self.gain_pan * abs(math.atan2(msg.pos.y, msg.pos.x))))

                if msg.pos.y > 0:
                    self.pan_position = math.atan2(msg.pos.y, msg.pos.x)
                else:
                    self.pan_position = math.atan2(msg.pos.y, msg.pos.x)
            else:
                self.pan_speed = 0
                self.pan_position = current_pan
        finally:
            self.lock.release()

    def center_head_servos(self):
        rospy.loginfo("centering servos.")

        self.servo_speed[self.head_pan_joint](self.default_joint_speed)
#        self.servo_speed[self.head_tilt_joint](self.default_joint_speed)

#        current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
        current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

        while abs(current_pan) > 0.05:
            self.servo_position[self.head_pan_joint].publish(0)
#            self.servo_position[self.head_tilt_joint].publish(0)

            rospy.sleep(0.1)

#            current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

        self.servo_speed[self.head_pan_joint](0.0)
 #       self.servo_speed[self.head_tilt_joint](0.0)

    def init_servos(self):
        self.servo_speed = dict()
        self.servo_position = dict()
        self.torque_enable = dict()

        rospy.loginfo("waiting for joint controller servicce")

        for joint in sorted(self.joints):
            set_speed_service = '/' + joint + '/set_speed'
            rospy.wait_for_service(set_speed_service)
            self.servo_speed[joint] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent=True)

            self.servo_speed[joint](self.default_joint_speed)
            self.servo_position[joint] = rospy.Publisher('/' + joint + '/command', Float64, queue_size=5)

            torque_enable = '/' + joint + '/torque_enable'
            rospy.wait_for_service(torque_enable)
            self.torque_enable[joint] = rospy.ServiceProxy(torque_enable, TorqueEnable)
            self.torque_enable[joint](False)

        self.pan_position = 0
#        self.tilt_position = 0
        self.pan_speed = 0
#        self.tilt_speed = 0

        self.last_pan_position = 0
#        self.last_tilt_position = 0
#        self.last_tilt_speed = 0
        self.last_pan_speed = 0

    def set_servo_speed(self, servo, speed):
        if speed == 0:
            speed = 0.01
        self.servo_speed[servo](speed)

    def set_servo_position(self, servo, position):
        self.servo_position[servo].publish(position)

    def update_joint_state(self, msg):
        self.joint_state = msg

    def shutdown(self):
        rospy.loginfo("shutting down head tracking node")
        try:
            self.tld_subscriber.unregister()
        except:
            pass

        self.center_head_servos()

        rospy.sleep(2)

        rospy.loginfo("relaxing pan and tilt servos")

        for servo in self.joints:
            self.torque_enable[servo](False)


class start_tracker():
    def __init__(self):
        rospy.loginfo("start tracker!!!")
        rospy.Subscriber("task_comming", xm_Task, self.task_handle)

    def task_handle(self, msg):
        if msg.task == 0x05:
            tldTracker()
            rospy.spin()
        else:
            pass

if __name__ == '__main__':
    try:
        rospy.init_node("tld_tracker")
        start_tracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("tld tracking node terminated.")
