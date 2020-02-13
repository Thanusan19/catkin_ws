#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

scale=1.5
vel=1
angle=1.5

tc.WaitForAuto()
try:
    tc.GoToPose(goal_x=3,goal_y=3,max_velocity=vel,goal_angle=1,dumbSmart=False,k_v=3,k_alpha=8,k_gamma=-1.5,Holonomic=True)
    tc.Wait(duration=1.0)
    
    # tc.GoToPose(goal_x=scale,goal_y=-scale,max_velocity=0.5,goal_angle=0,dumbSmart=False,k_v=3,k_alpha=8,k_gamma=-1.5,Holonomic=True)
    # tc.Wait(duration=1.0)
    
    # tc.GoToPose(goal_x=-scale,goal_y=scale,max_velocity=0.5,goal_angle=3,dumbSmart=False,k_v=3,k_alpha=8,k_gamma=-1.5,Holonomic=True)
    # tc.Wait(duration=1.0)
    
    #tc.GoToPose(goal_x=-scale,goal_y=scale,max_velocity=vel,goal_angle=0,dumbSmart=True,k_v=3,k_alpha=8,k_gamma=-0.5)
    #tc.Wait(duration=1.0)


except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")
