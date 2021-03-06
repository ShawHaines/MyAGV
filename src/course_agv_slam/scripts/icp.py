#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Quaternion,Point
from std_msgs.msg import Header
import numpy as np
import math
from course_agv_slam.srv import Odometry_srv
import sys

def toArray(p):
    return np.array([p.x,p.y,p.z])
def toList(q):
    return [q.x,q.y,q.z,q.w]

class NeighBor:
    def __init__(self):
        self.distances = []
        self.src_indices = []
        self.tar_indices = []

class Localization(object):
    '''
    The basic Localizing function class. Father of all sorts of Odometry class like ICP.
    Includes several utility functions.
    nodeName is also frame name.
    '''
    def __init__(self,nodeName):
        self.nodeName=nodeName
        # State Vector [x y yaw].T, column vector.
        self.xEst=np.zeros((3,1))
        self.odom_pub = rospy.Publisher(nodeName,Odometry,queue_size=1)
        self.odom_broadcaster=tf.TransformBroadcaster()

    def publishResult(self):
        self.odom_pub.publish(self.statusToOdometry(self.xEst))

        # flatten out
        s=np.array(self.xEst).reshape(-1)
        print("sensor-global  : {}".format(s))

        # tf broadcast
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),tuple(q),
                            rospy.Time.now(),self.nodeName,"world_base")

    def translateResult(self,T):
        # what exactly is this T? T is a affine transformation matrix of 1 higher order.
        # print("T: {}".format(T))
        delta_yaw = math.atan2(T[1,0],T[0,0])
        # [[cos(theta),-sin(theta)],[sin(theta),cos(theta)]]
        # print("sensor-delta-xyt:[{},{},{}]".format(T[0,2],T[1,2],delta_yaw))
        # improved readability
        # rotation=T[0:2,0:2]
        translation=T[0:2,2].reshape(2,1)
        x,y,theta=self.xEst
        theta+=delta_yaw
        # the position of theta+=delta_yaw needs consideration.
        eulerMatrix=tf.transformations.euler_matrix(0,0,theta)[0:2,0:2]
        x,y=np.array([x,y])+np.dot(eulerMatrix,translation)
        self.xEst=np.array([x,y,theta])
        
        # s = self.sensor_sta
        # self.sensor_sta[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        # self.sensor_sta[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        # self.sensor_sta[2] = s[2] + delta_yaw
    
    def statusToOdometry(self,sensorStatus):
        # flatten out
        s=np.array(sensorStatus).reshape(-1)
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        # odom topic publish
        odom = Odometry(header=Header(0,rospy.Time.now(),"world_base"))
        # odom.header.stamp = rospy.Time.now()
        # odom.header.frame_id = "world_base"

        # odom.pose.pose.position.x = s[0]
        # odom.pose.pose.position.y = s[1]
        # odom.pose.pose.position.z = 0
        odom.pose.pose.position=Point(s[0],s[1],0)
        # odom.pose.pose.orientation.x = q[0]
        # odom.pose.pose.orientation.y = q[1]
        # odom.pose.pose.orientation.z = q[2]
        # odom.pose.pose.orientation.w = q[3]
        
        # very elegant grammar!
        odom.pose.pose.orientation=Quaternion(*q)

        return odom

    def calcDist(self,a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)

    def T2u(self,t):
        '''
        translate relative transform matrix T to relative u.
        Note that relative u is according to self frame.
        x=x+[[R,0],[0,1]] u
        '''
        dw = math.atan2(t[1,0],t[0,0])
        u = np.array([[t[0,2],t[1,2],dw]])
        # .T can be viewed as transpose in 2 dimentional matrix.
        return u.T

    def x2T(self,x):
        T=tf.transformations.euler_matrix(0,0,x[2,0])[0:3,0:3]
        T[0:2,2]=x[0:2,0]

class ICPBase(Localization):
    def __init__(self,nodeName="icp_odom"):

        self.inf=1e6
        
        super(ICPBase,self).__init__(nodeName)

        # robot init states
        self.robot_x = float(rospy.get_param('/icp/robot_x',0))
        self.robot_y = float(rospy.get_param('/icp/robot_y',0))
        self.robot_theta = float(rospy.get_param('/icp/robot_theta',0))     
        
        # sensor's estimation for current state = robot_x_y_theta, overrides Localization.
        self.xEst = np.array([[self.robot_x,self.robot_y,self.robot_theta]]).T
        # if is the first scan, set as the map/target
        self.isFirstScan = True
        # src point cloud matrix
        self.src_pc=None
        # target point cloud matrix
        self.tar_pc=None

        

        self.laser_count  = 0
        # interval should not be set too small(like 1). Recommend that process once every 5 laser frames
        self.laser_interval= 5
        
        # max iterations
        self.max_iter = int(rospy.get_param('/icp/max_iter',10))
        # distance threshold for matching points
        self.dis_th = float(rospy.get_param('/icp/dis_th',0.5))
        # tolerance to stop icp
        self.tolerance = float(rospy.get_param('/icp/tolerance',0))

        # for wheel odometry.
        self.estimatedPose=Pose(position=Point(0,0,0),orientation=Quaternion(0,0,0,1))

    def processICP(self,source,target,initialT=None):
        '''
        Process the fitting between source and target.
        Returns T the transformation matrix.
        source and target is guarenteed not to be accidentally changed.
        initial argument pass in the initial value of T.
        If set to None, use Wheel odometry reference
        '''
        # init some variables
        src=np.copy(source)
        tar=np.copy(target)
        if initialT is None:
            try:
                # wheel auxilary. If you wnat to avoid it, pass in an identity(3) as initial.
                rospy.wait_for_service("/course_agv/odometry",timeout=0.1)
                newPose=rospy.ServiceProxy("/course_agv/odometry",Odometry_srv)()
                newPose=newPose.pose
                # print("New Pose(Wheel):{}".format(newPose))
                # print("type: {}".format(type(newPose)))
                # print("Original Pose(Wheel):{}".format(self.estimatedPose))
                # print("type: {}".format(type(self.estimatedPose)))
                translation=toArray(newPose.position)-toArray(self.estimatedPose.position)
                translation=translation[0:2]
                newEuler=tf.transformations.euler_from_quaternion(toList(newPose.orientation))
                oldEuler=tf.transformations.euler_from_quaternion(toList(self.estimatedPose.orientation))
                rotation=tf.transformations.euler_matrix(0,0,newEuler[2]-oldEuler[2])[0:2,0:2]
                # change to self frame
                
                translation=np.dot(tf.transformations.euler_matrix
                        (oldEuler[0],oldEuler[1],-oldEuler[2])[0:2,0:2],translation)
                self.estimatedPose=newPose
            except rospy.ROSException, e:
                print("wheel odometry failed: {}".format(e))
                # fallback to no reference.
                rotation = np.identity(2)
                translation=np.zeros(2)
        else:
            rotation = initialT[0:2,0:2]
            translation=initialT[0:2,2]

        # print("initial rotation:\n{} \ntranslation:{}".format(rotation,translation))
        # FIXME: learn about relativity! the laser in the frame moves opposite
        translation=-translation
        rotation=np.transpose(rotation)
        iterations=0
        # temp=np.copy(tar)
        lastDeviation=self.inf
        
        # don't move src_pc, adjust tar_pc to fit src.
        for _ in range(self.max_iter): # I haven't seen this grammar before...
            # transform tar_pc:
            iterations += 1
            # elegant matrix multiplying
            temp=np.dot(rotation,tar)+translation.reshape((2,1))
            neighbour=self.findNearest(src,temp)
            deviation=np.sum(neighbour.distances)
            # print("d= {} (iterations{})".format(deviation,iterations))
            if lastDeviation==deviation or deviation<self.tolerance*len(neighbour.src_indices):
                break
            lastDeviation=deviation
            # change the pairing rule
            tempTar=tar[:,neighbour.tar_indices] # elegant and pythonic!
            tempSrc=src[:,neighbour.src_indices]
            rotation,translation=self.getTransform(tempSrc,tempTar)
                  
        # print("--------------------------------------")
        # print("total iterations: {}".format(iterations))
        # print("total deviation: {}".format(np.sum(neighbour.distances)))
        T=np.identity(3)
        # because of the relative relation between frames, R and T should reverse
        T[0:2,0:2]=np.transpose(rotation)
        T[0:2,2]=-translation
        # error dealing.
        if np.any(np.isnan(T)):
            return np.identity(3)
        return T

    def findNearest(self,src,tar):
        '''
        guarantees that src and tar won't change.
        '''
        if np.size(tar)==0:
            return NeighBor()
        # find the pairing strategy between src and tar.
        neighbour = NeighBor()
        length=np.size(src,1)
        # allows one tar to multiple src pair
        for i in range(length):
            # np parallel computing is much faster than looping. Compare the code commented out and this!
            temp=np.linalg.norm(tar-src[:,i].reshape(2,1),axis=0) # distance
            # temp=[np.linalg.norm(tar[:,j]-src[:,i]) for j in range(length)]
            index=np.argmin(temp)
            # filter out the non-matching point pairs
            if temp[index]<self.dis_th:
                neighbour.src_indices.append(i)
                neighbour.tar_indices.append(index)
                neighbour.distances.append(temp[index])
        
        # print("neighbour:\n\ttar_indices:{}".format(neighbour.tar_indices))
        # print("\tdistances:{}".format(neighbour.distances))
        return neighbour

    def getTransform(self,src,tar):
        '''
        guarantees that src and tar won't change.
        '''
        # be very careful that the arguments are passed in as references.
        if np.size(src,1)!= np.size(tar,1):
            print("error in length!")
            # skipping
            return(np.identity(2),np.array([0,0]))
        length=np.size(src,1)
        srcCenter=np.mean(src,axis=1)
        tarCenter=np.mean(tar,axis=1)
        
        # print("srcCenter:{}".format(srcCenter))
        # print("tarCenter:{}".format(tarCenter))
        
        # ELEGANT!
        srcPrime=src-srcCenter.reshape((2,1))
        tarPrime=tar-tarCenter.reshape((2,1))
        W=np.dot(tarPrime,srcPrime.T)
        # print("W={}".format(W))
        U,S,V=np.linalg.svd(W)
        # FIXME: notice the difference of svd decomposing declaration! W=U*diag(S)*V
        rotation=np.transpose(np.dot(U,V))
        translation=srcCenter-np.dot(rotation,tarCenter)
        # print("rotation:{}".format(rotation))
        # print("translation:{}".format(translation))
        return (rotation,translation)
       
    def laserToNumpy(self,msg):
        # the x,y coordinates are in robot's frame?
        # pc = np.ones([3,total_num])
        total_num = len(msg.ranges)
        range_l = np.array(msg.ranges)
        # there COULD be inf! Need to discard them!
        valid=~np.isinf(range_l)
        range_l=range_l[valid]
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)[valid]
        # ufunc, high performance
        pc = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc

class ICP(ICPBase):
    def __init__(self,nodeName="icp_odom"):
        super(ICP,self).__init__(nodeName)

        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback,queue_size=self.laser_interval)
    
    def laserCallback(self,msg):
        # process and fit laser pointcloud data. 
        # callback is a little messy.
        print('------seq:  ',msg.header.seq)
        if not np.any(self.tar_pc):
            self.tar_pc = self.laserToNumpy(msg)
            # self.isFirstScan = False
            self.laser_count = 0
            return
        
        # process once every 5 laser scan because laser fps is too high
        self.laser_count += 1
        if self.laser_count < self.laser_interval:
            return
        self.laser_count = 0
        time_0 = rospy.Time.now()
        self.src_pc = self.laserToNumpy(msg)
        # print('input cnt: ',self.src_pc.shape[1])

        T=self.processICP(self.src_pc,self.tar_pc,initialT=np.identity(3))
        self.tar_pc = np.copy(self.src_pc) # moving the target to src
        self.translateResult(T)
        self.publishResult()
        duration=rospy.Time.now()-time_0
        print("time_cost: {} s".format(duration.to_sec()))
        pass

# used in localization.py
class SubICP(ICPBase):
    '''
    The management of firstScan etc. are moved to the Localization 
    object to handle.
    '''
    def __init__(self):
        super(SubICP,self).__init__()
    
    def laserCallback(self,msg):
        '''
        laser ICP odometry.
        '''
        time_0 = rospy.Time.now()
        self.src_pc = self.laserToNumpy(msg)
        # print('input cnt: ',self.src_pc.shape[1])

        T=self.processICP(self.src_pc,self.tar_pc,initialT=np.identity(3))
        self.tar_pc = np.copy(self.src_pc) # moving the target to src
        self.translateResult(T)
        self.publishResult()
        duration=rospy.Time.now()-time_0
        print("time_cost: {} s".format(duration.to_sec()))
        return self.T2u(T)

class LandmarkICP(ICPBase):
    def __init__(self):
        super(LandmarkICP,self).__init__()
    
    def laserCallback(self,msg):
        '''
        laser ICP odometry.
        '''
        time_0 = rospy.Time.now()
        self.src_pc = msg
        # print('input cnt: ',self.src_pc.shape[1])

        T=self.processICP(self.src_pc,self.tar_pc,initialT=np.identity(3))
        self.tar_pc = np.copy(self.src_pc) # moving the target to src
        self.translateResult(T)
        self.publishResult()
        duration=rospy.Time.now()-time_0
        print("time_cost: {} s".format(duration.to_sec()))
        return self.T2u(T)

def main():
    rospy.init_node('icp_node')
    icp = ICP()
    rospy.spin()

if __name__ == '__main__':
    main()