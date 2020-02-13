#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
import math
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.05)
tc = TaskClient(server_node,default_period)
rospy.loginfo("Mission connected to server: " + server_node)

#scale=2.0
#vel=0.5

tc.WaitForAuto()
while True:
    tc.clearConditions() 
    w4roi = tc.WaitForFace(foreground=False) # q t on besoin d'argument
    tc.addCondition(ConditionIsCompleted("ROI detector",tc,w4roi)) # c'est quoi
    try:
        tc.Wander()
        tc.clearConditions()
    except TaskConditionException, e:
        tc.StareAtFace()
        tc.Wait(duration=5.)
        tc.SetHeading(relative=True,target=math.pi)
        
        #tc.StareAtFace(foreground=False)
        
        #rospy.logerr(" Good one: " + str(e))
    #tc.StareAtFace() # When the condition above is true so the robot  stare at the face
    #tc.Wait(duration=1.)
    #tc.SetHeading(relative)# quelle argument faut il passer 


if not rospy.core.is_shutdown():
    tc.SetManual()



rospy.loginfo("Mission completed")
