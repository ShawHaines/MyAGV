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

def toArray(p):
    return np.array([p.x,p.y,p.z])
def toList(q):
    return [q.x,q.y,q.z,q.w]
class NeighBor:
    def __init__(self):
        self.distances = []
        self.src_indices = []
        self.tar_indices = []

class OdometryLocation(object):
    def __init__(self):
        # robot init states
        self.robot_x = float(rospy.get_param('/icp/robot_x',0))
        self.robot_y = float(rospy.get_param('/icp/robot_y',0))
        self.robot_theta = float(rospy.get_param('/icp/robot_theta',0))     
        
        # sensor's estimation for current state = robot_x_y_theta
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        # if is the first scan, set as the map/target
        self.isFirstScan = True
        # src point cloud matrix
        self.src_pc=None
        # target point cloud matrix
        self.tar_pc=None
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
        pass

    def publishResult(self,T):
        # what exactly is this T? T is a affine transformation matrix of 1 higher order.
        print("T: {}".format(T))
        delta_yaw = math.atan2(T[1,0],T[0,0])
        # [[cos(theta),-sin(theta)],[sin(theta),cos(theta)]]
        print("sensor-delta-xyt:[{},{},{}]".format(T[0,2],T[1,2],delta_yaw))
        # improved readability
        # rotation=T[0:2,0:2]
        translation=T[0:2,2]
        x,y,theta=self.sensor_sta
        theta+=delta_yaw
        # the position of theta+=delta_yaw needs consideration.
        eulerMatrix=tf.transformations.euler_matrix(0,0,theta)[0:2,0:2]
        x,y=np.array([x,y])+np.dot(eulerMatrix,translation)
        self.sensor_sta=[x,y,theta]
        # s = self.sensor_sta
        # self.sensor_sta[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        # self.sensor_sta[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        # self.sensor_sta[2] = s[2] + delta_yaw
        print("sensor-global: {}".format(self.sensor_sta))

        # tf broadcast
        s = self.sensor_sta
        q = tf.transformations.quaternion_from_euler(0,0,self.sensor_sta[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"icp_odom","world_base")

        # odom topic publish
        # FIXME: the code can be more concise...
        odom = Odometry(header=Header(0,rospy.Time.now(),"world_base"))
        # odom.header.stamp = rospy.Time.now()
        # odom.header.frame_id = "world_base"

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)
        
    def laserToNumpy(self,msg):
        # the x,y coordinates are in robot's frame?
        total_num = len(msg.ranges)
        # pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        # ufunc, high performance
        pc = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        # print("Numpy pc:{}".format(pc))
        return pc

    def calcDist(self,a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)

class ICPBase(OdometryLocation):
    inf=1e6
    def __init__(self):
        super(ICPBase,self).__init__()
        self.laser_count  = 0
        # process once every 5 laser frames
        self.laser_inteval= 5
        
        # max iterations
        self.max_iter = int(rospy.get_param('/icp/max_iter',10))
        # distance threshold for filter the matching points
        self.dis_th = float(rospy.get_param('/icp/dis_th',0.5))
        # tolerance to stop icp
        self.tolerance = float(rospy.get_param('/icp/tolerance',10))
        
        self.estimatedPose=Pose(position=Point(0,0,0),orientation=Quaternion(0,0,0,1))
    
    def processICP(self,source,target):
        '''
        Process the fitting between source and target.
        Returns T the transformation matrix.
        source and target is guarenteed not to be accidentally changed.
        '''
        # init some variables
        src=np.copy(source)
        tar=np.copy(target)
        try:
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
            # no reference from wheel.
            rotation = np.identity(2)
            translation=np.zeros(2)

        print("initial rotation:\n{} \ntranslation:{}".format(rotation,translation))
        # FIXME: learn about relativity! the laser in the frame moves opposite
        translation=-translation
        rotation=np.transpose(rotation)
        
        iterations=0
        temp=np.copy(tar)
        lastDeviation=0
        
        # don't move src_pc, adjust tar_pc to fit src.
        for _ in range(self.max_iter): # I haven't seen this grammar before...
            # transform tar_pc:
            for i in range(np.size(tar,1)):
                # FIXME: You CAN'T change the target!
                temp[:,i]=np.dot(rotation,tar[:,i])+translation
            iterations += 1

            neighbour=self.findNearest(src,temp)
            deviation=np.sum(neighbour.distances)
            if lastDeviation==deviation or deviation<self.tolerance:
                break
            lastDeviation=deviation
            # change the pairing rule
            for i in range(np.size(tar,1)):
                temp[:,i]=tar[:,neighbour.tar_indices[i]]
            # FIXME: again the SHALLOW COPY!
            tar=np.copy(temp)
            rotation,translation=self.getTransform(src,tar)
            
            print("d= {} (iterations{})".format(deviation,iterations))
        
        print("--------------------------------------")
        print("total iterations: {}".format(iterations))
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
        # find the pairing strategy between src and tar.
        neighbour = NeighBor()
        length=np.size(src,1)
        # or you can use .tolist() method, since there's only one dimension, we can stick with this.
        neighbour.src_indices=list(np.zeros(length))
        neighbour.tar_indices=list(np.zeros(length)) 
        neighbour.distances=list(np.zeros(length))
        
        temp=[np.linalg.norm(src[:,i]-tar[:,j]) for i in range(length) for j in range(length)]
        temp=np.reshape(temp,(length,length))
        # print("distance matrix:{}".format(temp))
        for i in range(length):
            index=np.argmin(temp)
            row,column=index//length,index%length
            neighbour.src_indices[i]=row
            neighbour.tar_indices[i]=column
            neighbour.distances[i]  =temp[row,column]
            # removing this point pair...
            temp[row,:]=self.inf
            temp[:,column]=self.inf
        # very pythonic and elegent use of reordering! sort according to the src_indices in ascending order.
        ordering=np.argsort(neighbour.src_indices)
        # only array supports such operations...
        neighbour.src_indices=list(np.array(neighbour.src_indices)[ordering])
        neighbour.tar_indices=list(np.array(neighbour.tar_indices)[ordering])
        neighbour.distances  =list(np.array(neighbour.distances)[ordering])
        
        # print("neighbour:\n\ttar_indices:{}".format(neighbour.tar_indices))
        # print("\tdistances:{}".format(neighbour.distances))
        return neighbour

    def getTransform(self,src,tar):
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
        # TODO: There's still improving space.
        q_all=[np.dot(np.reshape(tar[:,i]-tarCenter,(2,1)),np.reshape(src[:,i]-srcCenter,(1,2))) for i in range(length)]
        # print(q_all)
        W=np.sum(q_all,axis=0)
        # print("W={}".format(W))
        U,S,V=np.linalg.svd(W)
        # FIXME: notice the difference of svd decomposing declaration!
        rotation=np.transpose(np.dot(U,V))
        translation=srcCenter-np.dot(rotation,tarCenter)
        # print("rotation:{}".format(rotation))
        # print("translation:{}".format(translation))
        return (rotation,translation)

class ICP(ICPBase):
    def __init__(self):
        super(ICP,self).__init__()

        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback,queue_size=self.laser_inteval)
    
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
        if self.laser_count < self.laser_inteval:
            return
        self.laser_count = 0
        time_0 = rospy.Time.now()
        self.src_pc = self.laserToNumpy(msg)
        # print('input cnt: ',self.src_pc.shape[1])

        T=self.processICP(self.src_pc,self.tar_pc)
        self.tar_pc = np.copy(self.src_pc) # moving the target to src

        self.publishResult(T)
        duration=rospy.Time.now()-time_0
        print("time_cost: {} s".format(duration.to_sec()))
        pass

def main():
    rospy.init_node('icp_node')
    icp = ICP()
    rospy.spin()

if __name__ == '__main__':
    main()