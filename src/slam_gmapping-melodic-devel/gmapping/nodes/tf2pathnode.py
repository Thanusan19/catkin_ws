#!/usr/bin/env python  

import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import Path, Odometry

def callback(msgodom):
    pose =msgodom.pose.pose
    poseStamped.append(PoseStamped(pose=pose))
    p=Path(poses=poseStamped)
    p.header.frame_id = 'map'
    pub.publish(p)

def init():
    global pub
    global poseStamped
    poseStamped = []
    rospy.init_node('nodePath')
    pub = rospy.Publisher('/path', Path, queue_size = 10 )
    rospy.Subscriber("/odom",Odometry,callback)
    rospy.spin()

if __name__ == '__main__':
    init()