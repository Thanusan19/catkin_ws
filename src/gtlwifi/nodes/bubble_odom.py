#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import message_filters
from math import atan2, hypot, pi, cos, sin, fmod

def norm_angle(x):
    return (x+pi)%(2*pi)-pi


class BubbleOdom:
    def __init__(self):
        rospy.init_node('bubble_odom')
        rospy.loginfo("Starting bubble rob odometry")
        self.base_line = rospy.get_param("~base_line",0.2)
        self.wheel_radius = rospy.get_param("~wheel_radius",0.1)
        self.body_frame = rospy.get_param("~body_frame","/base_link")
        self.odom_frame = rospy.get_param("~odom_frame","/odom")
        self.vrep_prefix = rospy.get_param("~vrep_prefix","/vrep")
        self.last_cmd = rospy.Time.now()
        self.broadcaster = tf.TransformBroadcaster()
        self.drive_sub={}
        self.ready = False
        self.connected = False
        self.left_prev = None
        self.right_prev = None
        self.pose = {"x":0, "y":0, "theta":0}
        print "Inter wheel and radius : %f %f" % (self.base_line,self.wheel_radius)

        self.drive_sub = rospy.Subscriber(self.vrep_prefix+"/wheelEncoder", JointState,self.sync_odo_cb,queue_size=1)

    def sync_odo_cb(self,joint_state):
        if not self.connected:
            self.js_prev = dict(zip(joint_state.name,joint_state.position))
            self.connected = True
            return
        js = dict(zip(joint_state.name,joint_state.position))
        dleft = norm_angle(js["left"] - self.js_prev["left"])
        dright = norm_angle(js["right"] - self.js_prev["right"])
        if dleft > pi:
            dleft -= 2*pi
        elif dleft < -pi:
            dleft += 2*pi
        if dright > pi:
            dright -= 2*pi
        elif dright < -pi:
            dright += 2*pi
        dleft *= self.wheel_radius
        dright *= self.wheel_radius
        dx = (dleft + dright)/2.0
        dy = 0
        dtheta = (dright - dleft)/self.base_line
        # print [dleft, dright, dx, dy ,dtheta]
        self.pose["x"] += dx * cos(self.pose["theta"]) - dy * sin(self.pose["theta"])
        self.pose["y"] += dx * sin(self.pose["theta"]) + dy * cos(self.pose["theta"])
        self.pose["theta"] += dtheta
        self.broadcaster.sendTransform((self.pose["x"], self.pose["y"], 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.pose["theta"]),
                     joint_state.header.stamp, self.body_frame, self.odom_frame)
        self.js_prev = js




    def run(self):
        rospy.spin()


if __name__=="__main__":
    demo = BubbleOdom()
    demo.run()
