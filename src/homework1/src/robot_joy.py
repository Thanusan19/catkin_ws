#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
	twist = Twist()
	twist.linear.x = 4*data.axes[1]
	twist.angular.z = 4*data.axes[0]
	pub.publish(twist)

# Intializes 
def start():
	global pub
	pub = rospy.Publisher('vrep/twistCommand', Twist)
	# subscribed to "joy" topic 
	rospy.Subscriber("joy", Joy, callback)
	# starts the node
	rospy.init_node('robot_joy')
	rospy.spin()

if __name__ == '__main__':
	start()
