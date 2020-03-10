import roslib; roslib.load_manifest('ar_mapping_base')
import rospy
from numpy import *
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
import tf
import threading

import rover_driver_base
from rover_driver_base.rover_kinematics import *

class MappingKF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        RoverKinematics.__init__(self)
        self.lock = threading.Lock()
        self.X = mat(vstack(initial_pose))
        self.P = mat(diag(initial_uncertainty))
        self.idx = {}
        self.pose_pub = rospy.Publisher("~pose",PoseStamped,queue_size=1)
        self.marker_pub = rospy.Publisher("~landmarks",MarkerArray,queue_size=1)
        self.counter=3

    def getRotation(self, theta):
        R = mat(zeros((2,2)))
        R[0,0] = cos(theta); R[0,1] = -sin(theta)
        R[1,0] = sin(theta); R[1,1] = cos(theta)
        return R

    def prepare_inversion_matrix(self,drive_cfg):
        W = numpy.asmatrix(numpy.zeros((len(prefix)*2,3)))
        for i in range(len(prefix)):
            k = prefix[i]
            # prepare the least-square matrices
            W[2*i+0,0] = 1; W[2*i+0,1] = 0; W[2*i+0,2] = -drive_cfg[k].y; 
            W[2*i+1,0] = 0; W[2*i+1,1] = 1; W[2*i+1,2] = +drive_cfg[k].x; 
        return pinv(W)


    def prepare_displacement_matrix(self, motor_state_t1, motor_state_t2, drive_cfg):
        # then compute odometry using least square
        S = numpy.asmatrix(numpy.zeros((len(prefix)*2,1)))
        for i in range(len(prefix)):
            k = prefix[i]
            # compute differentials
            beta = (motor_state_t1.steering[k]+motor_state_t2.steering[k])/2
            ds = (motor_state_t2.drive[k] - motor_state_t1.drive[k]) % (2*pi)
            if ds>pi:
                ds -= 2*pi
            if ds<-pi:
                ds += 2*pi
            ds *= drive_cfg[k].radius
            S[2*i+0,0] = ds*cos(beta)
            S[2*i+1,0] = ds*sin(beta)
        return S

    
    def predict(self, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return (self.X, self.P)
        # print "-"*32
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state,motor_state,drive_cfg)
        self.motor_state.copy(motor_state)
        
        # Implement Kalman prediction here
        theta = self.X[2,0]
        Rtheta = mat([[cos(theta), -sin(theta), 0], 
                      [sin(theta),  cos(theta), 0],
                      [         0,           0, 1]]);
        DeltaX = iW*S
        # Update the state using odometry (same code as for the localisation
        # homework), but we only need to deal with a subset of the state:
        # TODO
        X= self.X[0:3,0]
        P= self.P[0:3,0:3]
        odoRobotFrame=matmul(iW,S)
        A= mat([[1,0,-sin(theta)*odoRobotFrame[0,0]-cos(theta)*odoRobotFrame[1,0]],[0,1,cos(theta)*odoRobotFrame[0,0]-sin(theta)*odoRobotFrame[1,0]],[0,0,1]])
        AT=transpose(A)

        B=matmul(Rtheta,iW)
        BT=transpose(B)
        

        # Qu=matmul(S,transpose(S))
        Qu = pow(encoder_precision,2)*identity(12)
        Q= pow(10,-4)*identity(3)
        # Q= pow(encoder_precision,2)*identity(3)
     
        odoWorldFrame=matmul(Rtheta,odoRobotFrame)
        
        X = X + odoWorldFrame
        P = matmul(A,matmul(P,AT)) + matmul(B,matmul(Qu,BT)) + Q

        self.X[0:3,0] = X[0:3,0]
        self.P[0:3,0:3] = P[0:3,0:3]

        self.lock.release()
        return (self.X,self.P)


    def update_ar(self, Z, id, uncertainty):
        # Landmark id has been observed as Z with a given uncertainty
        self.lock.acquire()
        print "Update: Z="+str(Z.T)+" X="+str(self.X.T)+" Id="+str(id)
        # Update the full state self.X and self.P based on landmark id
        # be careful that this might be the first time that id is observed
        # TODO

        
    

        if id in self.idx:
            #self.idx[id].update(Z=Z,X=self.X,R=uncertainty)
            # self.idx[id]=0
            n = len(self.idx)
            l=self.idx[id]
            P=self.P
            theta = self.X[2,0]
            uncertainty=5
            R=pow(uncertainty,2)*identity(2)
            L=self.X[l:l+2,0]
            print(L)

            
            H = mat(zeros((2,2*len(self.idx)+3)))
            LX=L-self.X[0:2,0]
            H[0,0] = -cos(theta) 
            H[0,1] = -sin(theta)
            H[0,2] = -sin(theta)*LX[0,0]+cos(theta)*LX[1,0]
            H[1,0] = sin(theta)
            H[1,1] = -cos(theta)
            H[1,2] = -cos(theta)*LX[0,0]-sin(theta)*LX[1,0]
            H[0:2,l:l+2] = self.getRotation(-theta)


            # Hrobot = mat([[-cos(theta), -sin(theta), (-sin(theta)*(L[0,0]-self.X[0,0])+cos(theta)*(L[1,0]-self.X[1,0]))], 
            #          [sin(theta),  -cos(theta), (-cos(theta)*(L[0,0]-self.X[0,0])-sin(theta)*(L[1,0]-self.X[1,0]))]])
            # Hland= mat([[cos(theta),sin(theta)],[-sin(theta),cos(theta)]])
            # Hzeros=numpy.zeros((2,(self.idx[id]-3)))
            # HzerosEnd=numpy.zeros((2,(3+2*len(self.idx))-(3+(self.idx[id]-3)+2)))
            # H=hstack((Hrobot,Hzeros,Hland,HzerosEnd))

            #Kalman Filter
            # summ=matmul(H,matmul(self.P,transpose(H))) + R
            # sumInv=linalg.inv(summ)
            # KG=matmul(self.P,matmul(transpose(H),sumInv))
            KG=self.P*H.T*linalg.inv(H*self.P*H.T + R)

            #Calculation of covariance matrix
            #self.P=matmul((identity(self.P.shape[0])-matmul(KG,H)),self.P)
            self.P=(identity(self.P.shape[0])-KG*H)*self.P

            #Calculation of X
            #Zpred= matmul(self.getRotation(-theta),(L-self.X[0:2,0]))
            #self.X= self.X + matmul(KG,(Z-Zpred))
            Zpred=self.getRotation(-theta)*(L-self.X[0:2,0])
            self.X= self.X + KG*(Z-Zpred)



            print(KG)
            print(H)


        else:
            theta = self.X[2,0]
            
            #Coordinate of the new landmark into the initial frame
            #L= matmul(self.getRotation(theta),Z) + self.X[0:2,0]
            L= self.X[0:2,0] + self.getRotation(theta)*Z
            

            #Jacobien --> uncertaincy of our robot state  
            # n= len(self.idx)
            # Jrobot=mat([[1,0,-sin(theta)*Z[0,0]-cos(theta)*Z[1,0]],[0,1,cos(theta)*Z[0,0]-sin(theta)*Z[1,0]]])
            # Jzeros=zeros((2,n*2))
            # J=hstack((Jrobot,Jzeros))
            n = len(self.idx)
            J = mat(zeros((2,3+2*n)))
            J[0,0] = 1 
            J[0,1] = 0
            J[0,2] = -sin(theta)*Z[0,0]-cos(theta)*Z[1,0]
            J[1,0] = 0
            J[1,1] = 1
            J[1,2] = cos(theta)*Z[0,0]-sin(theta)*Z[1,0]
            J[0:2,3:3+n*2] = zeros((2,n*2))


            #Jacobien --> uncertaincy bring by the measure
            Rtheta=self.getRotation(theta)
            R=pow(uncertainty,2)*identity(2)
 
            S= matmul(Rtheta,matmul(R,transpose(Rtheta))) + matmul(J,matmul(self.P,transpose(J))) 

            #Covariance matrix of our global state
            Ptmp=zeros((self.P.shape[0]+2,self.P.shape[1]+2))
            Ptmp[0:self.P.shape[0],0:self.P.shape[1]]=self.P
            Ptmp[self.P.shape[0]:,self.P.shape[1]:]=S
            
            #Add a landmark position to our State vector X
            #X= vstack((X,L)) 
            self.idx[id]=self.counter
            self.counter+=2
            self.X=vstack((self.X,L))
            self.P=Ptmp       
     
        self.lock.release()
        return (self.X,self.P)

    def update_compass(self, Z, uncertainty):
        self.lock.acquire()
        print "Update: S="+str(Z)+" X="+str(self.X.T)
        # Update the full state self.X and self.P based on compass measurement
        # TODO
        # self.X = ...
        # self.P = ...
        self.lock.release()
        return (self.X,self.P)


    def publish(self, target_frame, timestamp):
        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = timestamp
        pose.pose.position.x = self.X[0,0]
        pose.pose.position.y = self.X[1,0]
        pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2,0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        self.pose_pub.publish(pose)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        marker.scale.x = 3*sqrt(self.P[0,0])
        marker.scale.y = 3*sqrt(self.P[1,1]);
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        ma.markers.append(marker)
        for id in self.idx.iterkeys():
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            l = self.idx[id]
            marker.pose.position.x = self.X[l,0]
            marker.pose.position.y = self.X[l+1,0]
            marker.pose.position.z = -0.1
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = 3*sqrt(self.P[l,l])
            marker.scale.y = 3*sqrt(self.P[l+1,l+1]);
            marker.scale.z = 0.1;
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
            marker.pose.position.x = self.X[l+0,0]
            marker.pose.position.y = self.X[l+1,0]
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




