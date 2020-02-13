#!/usr/bin/env python
import roslib; roslib.load_manifest('rover_driver_base')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin, fmod

prefix=["FL","FR","CL","CR","RL","RR"]

class RoverMotors:
    def __init__(self):
        self.steering={}
        self.drive={}
        for k in prefix:
            self.steering[k]=0.0
            self.drive[k]=0.0
    def copy(self,value):
        for k in prefix:
            self.steering[k]=value.steering[k]
            self.drive[k]=value.drive[k]

class DriveConfiguration:
    def __init__(self,radius,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self):
        self.X = numpy.asmatrix(numpy.zeros((3,1)))
        self.motor_state = RoverMotors()
        self.first_run = True

    def twist_to_motors(self, twist, drive_cfg, skidsteer=False):
        motors = RoverMotors()
        if skidsteer:
            for k in drive_cfg.keys():
                # Insert here the steering and velocity of 
                # each wheel in skid-steer mode
                motors.steering[k] = 0
                
                if (twist.linear.x>0):
					drive = hypot(twist.linear.x,twist.linear.y)/drive_cfg[k].radius
                else:
					drive = -hypot(twist.linear.x,twist.linear.y)/drive_cfg[k].radius
                
                motors.drive["FL"] = -twist.angular.z*drive_cfg[k].x + drive
                motors.drive["CL"] = -twist.angular.z*drive_cfg[k].x + drive
                motors.drive["RL"] = -twist.angular.z*drive_cfg[k].x + drive
                
                motors.drive["FR"] = twist.angular.z*drive_cfg[k].x + drive
                motors.drive["CR"] = twist.angular.z*drive_cfg[k].x + drive 
                motors.drive["RR"] = twist.angular.z*drive_cfg[k].x + drive
                
                
        else:
            for k in drive_cfg.keys():
                # Insert here the steering and velocity of 
                # each wheel in rolling-without-slipping mode
                motors.steering[k] = atan2(twist.linear.y+twist.angular.z*drive_cfg[k].x , twist.linear.x-twist.angular.z*drive_cfg[k].y)
                motors.drive[k] = hypot(twist.linear.x-twist.angular.z*drive_cfg[k].y,twist.linear.y+twist.angular.z*drive_cfg[k].x)/drive_cfg[k].radius
                
        return motors

    def integrate_odometry(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            return self.X
        # Insert here your odometry code
        A = numpy.zeros((12,3))
        B = numpy.zeros((12,1))
        Xr = numpy.zeros((3,1))
        Xi = numpy.zeros((3,1))
        R= numpy.zeros((3,3))
        i=0

        test = (motor_state.drive['FL']-self.motor_state.drive['FL'])
        
        for k in prefix:
            angle = (motor_state.drive[k]-self.motor_state.drive[k])
            if angle < -pi:
                angle=angle+2*pi
                                    #fmod((motor_state.drive[k]-self.motor_state.drive[k])+pi, 2 * pi) - pi
            s = drive_cfg[k].radius * angle
            A[i,:]=[1,0,- drive_cfg[k].y ]        
            A[i+1,:]=[0,1,drive_cfg[k].x]                  # x et y sont la position de la roue indice k 
            B[i,0] = s * cos(motor_state.steering[k])       # Steering correspond a l'angle de la roue (Beta)
            B[i+1,0] = s * sin(motor_state.steering[k])       #(drive_cfg.radius(k) * (motor_state[k].drive-self.motor_state[k].drive))
            i=i+2
            
        Xr=numpy.matmul(pinv(A),B)

        R[0,:] = [cos(self.X[2,0]),sin(self.X[2,0]),0]
        R[1,:] = [-sin(self.X[2,0]),cos(self.X[2,0]),0]
        R[2,:] = [0,0,1]

        Xi = numpy.matmul(pinv(R),Xr)

        self.X[0,0] += Xi[0,0] #0.0
        self.X[1,0] += Xi[1,0] #0.0
        self.X[2,0] += Xi[2,0] #0.0
        self.motor_state.copy(motor_state)
        return self.X



