import roslib; roslib.load_manifest('ar_mapping_base')
import rospy
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
import tf
import threading

import rover_driver_base
from rover_driver_base.rover_kinematics import *

class Landmark:
    def __init__(self, Z, X , R):
        # Initialise a landmark based on measurement Z, 
        # current position X and uncertainty R
        # TODO
        theta = X[2,0]
        Rot = self.getRotation(theta)
        # Xpos = mat([[X[0,0]],[X[1,0]]])

        self.L = vstack([0,0])   
        self.L = numpy.matmul(Rot,Z) + X[0:2] #Xpos

        R2 = R*R
        
       
        self.P = mat([[R2,R2],[R2,R2]])



    def update(self,Z, X, R):
        # Update the landmark based on measurement Z, 
        # current position X and uncertainty R
        # TODO
        P=self.P
        L=self.L
        
        theta = X[2,0]
        H= self.getRotation(-theta)
   
       

        HT=transpose(H)
        R=vstack(([R,0],[0,R]))
        
        #Compute Kalman Gain
        Exp1=matmul(P,HT)
        Exp2=matmul(matmul(H,P),HT)
        Sum=Exp2 + R

        KG=matmul(Exp1,linalg.inv(Sum))

        #Update estimate with measurement zk
        diffMeasureEstimate = Z - matmul(H,L-X[0:2])
        measurementWeight=matmul(KG,diffMeasureEstimate)

        self.L= L + measurementWeight

        #Update the error covariance
        covarianceWeight= identity(2)-matmul(KG,H)
        self.P= matmul(covarianceWeight,P)

        print(X[2,0])
      

        return

    def getRotation(self, theta):
        R = mat(zeros((2,2)))
        R[0,0] = cos(theta); R[0,1] = -sin(theta)
        R[1,0] = sin(theta); R[1,1] = cos(theta)
        return R
        


class MappingKF:
    def __init__(self):
        self.lock = threading.Lock()
        self.marker_list = {}
        self.marker_pub = rospy.Publisher("~landmarks",MarkerArray,queue_size=1)

    def update_ar(self, Z, X, Id, uncertainty):
        self.lock.acquire()
        print "Update: Z="+str(Z.T)+" X="+str(X.T)+" Id="+str(Id)
        R = mat(diag([uncertainty,uncertainty]))
        # print(X.shape)
        # print(X[0:2].shape)
        print(uncertainty)

        if Id in self.marker_list:
            self.marker_list[Id].update(Z=Z,X=X,R=uncertainty)
        else:
            self.marker_list[Id]=Landmark(Z=Z,X=X,R=uncertainty)
        
        # Take care of the landmark Id observed as Z from X
        # self.marker_list is expected to be a dictionary of Landmark
        # such that current landmark can be retrieved as self.marker_list[Id] 
        # At initialisation, self.marker_list is empty
        # TODO

        self.lock.release()


    def publish(self, target_frame, timestamp):
        ma = MarkerArray()
        for id in self.marker_list:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = max(3*sqrt(Lkf.P[0,0]),0.05)
            marker.scale.y = max(3*sqrt(Lkf.P[1,1]),0.05)
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime.secs=3.0;
            ma.markers.append(marker)
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = 1000+id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0,0]
            marker.pose.position.y = Lkf.L[1,0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime.secs=3.0;
            ma.markers.append(marker)
        self.marker_pub.publish(ma)

