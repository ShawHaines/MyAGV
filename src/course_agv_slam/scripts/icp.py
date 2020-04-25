#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
import numpy as np
import math
class NeighBor:
    def __init__(self):
        self.distances = []
        self.src_indices = []
        self.tar_indices = []

class ICP:
    inf=1e6
    def __init__(self):
        self.laser_count  = 0
        # process once every 5 laser frames
        self.laser_inteval= 5
        # robot init states
        self.robot_x = float(rospy.get_param('/icp/robot_x',0))
        self.robot_y = float(rospy.get_param('/icp/robot_y',0))
        self.robot_theta = float(rospy.get_param('/icp/robot_theta',0))
        # sensor's estimation for current state = robot_x_y_theta
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]

        # max iterations
        self.max_iter = int(rospy.get_param('/icp/max_iter',30))
        # distance threshold for filter the matching points
        self.dis_th = float(rospy.get_param('/icp/dis_th',5))
        # tolerance to stop icp
        self.tolerance = float(rospy.get_param('/icp/tolerance',0))
        # if is the first scan, set as the map/target
        self.isFirstScan = True
            # src point cloud matrix
        self.src_pc=None
        # target point cloud matrix
        self.tar_pc=None


        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()
    def laserCallback(self,msg):
        # process and fit laser pointcloud data.
        print('------seq:  ',msg.header.seq)
        if not np.any(self.tar_pc):
            self.tar_pc = self.laserToNumpy(msg)
            # self.isFirstScan = False
            self.laser_count = 0
            return
        
        # process once every 5 laser scan because laser fps is too high
        self.laser_count += 1
        if self.laser_count <= self.laser_inteval:
            return
        self.laser_count = 0
        time_0 = rospy.Time.now()
        self.src_pc = self.laserToNumpy(msg)
        print('input cnt: ',self.src_pc.shape[1])

        # init some variables
        rotation = np.identity(2)
        translation=np.zeros(2)
        iterations=0
        
        # don't move src_pc, adjust tar_pc to fit src.
        for _ in range(self.max_iter): # I haven't seen this grammar...
            neighbour=self.findNearest()
            if np.sum(neighbour.distances)<self.tolerance:
                break
            temp=np.zeros_like(self.tar_pc)
            # change the pairing rule
            for i in range(np.size(self.tar_pc,1)):
                temp[:,i]=self.tar_pc[:,neighbour.tar_indices[i]]
            self.tar_pc=temp
            rotation,translation=self.getTransform()
            # transform tar_pc:
            for i in range(np.size(self.tar_pc,1)):
                self.tar_pc[:,i]=np.dot(rotation,self.tar_pc[:,i])+translation
            iterations += 1
        
        print("total iterations: {}".format(iterations))
        self.tar_pc = self.src_pc # what is this for? fitting is between the adjacent frames..
        T=np.identity(3)
        T[0:2,0:2]=rotation
        T[0:2,2]=translation
        self.publishResult(T)
        time_1 = rospy.Time.now()
        duration=time_1-time_0
        print("time_cost: {} s".format(duration.to_sec()))
        pass
    def findNearest(self):
        # find the pairing strategy between src and tar.
        neighbour = NeighBor()
        length=np.size(self.src_pc,1)
        neighbour.src_indices=range(length)
        # or you can use .tolist() method, since there's only one dimension, we can stick with this.
        neighbour.tar_indices=list(np.zeros(length)) 
        neighbour.distances=list(np.zeros(length))
        # temp=self.tar_pc # FIXME: shallow copy!
        temp=np.copy(self.tar_pc)
        
        for i in range(length):
            # one formula solves everything.
            dist=np.linalg.norm(np.subtract(temp,np.reshape(self.src_pc[:,i],(2,1))),axis=0)
            index=np.argmin(dist)
            if dist[index]>=self.dis_th:
                print("The fitting may be not good")
            neighbour.tar_indices[i]=index
            neighbour.distances[i]=dist[index]
            # remove this point, move it to inf
            temp[:,index]=np.array([self.inf,0])

        print("neighbour:\n\ttar_indices:{}".format(neighbour.tar_indices))
        print("\tdistances:{}".format(neighbour.distances))
        return neighbour

    def getTransform(self):
        # be very careful that the arguments are passed in as references.
        src=self.src_pc
        tar=self.tar_pc
        # those are references, or alias
        length=np.size(src,1)
        srcCenter=np.mean(src,axis=1)
        tarCenter=np.mean(tar,axis=1)
        
        print("srcCenter:{}".format(srcCenter))
        print("tarCenter:{}".format(tarCenter))
        # TODO: There's still improving space.
        q_all=[np.dot(np.reshape(src[:,i]-srcCenter,(2,1)),np.reshape(tar[:,i]-tarCenter,(1,2))) for i in range(length)]
        # print(q_all)
        W=np.sum(q_all,axis=0)
        print("W={}".format(W))
        U,S,V=np.linalg.svd(W)
        rotation=np.dot(V,np.transpose(U))
        translation=srcCenter-np.dot(rotation,tarCenter)
        print("rotation:{}".format(rotation))
        print("translation:{}".format(translation))
        return (rotation,translation)

    def publishResult(self,T):
        # what exactly is this T? T is a affine transformation matrix of 1 higher order.
        print("T: {}".format(T))
        delta_yaw = math.atan2(T[1,0],T[0,0])
        print("sensor-delta-xyt: ",T[0,2],T[1,2],delta_yaw)
        # improved readability
        rotation=T[0:2,0:2]
        translation=T[0:2,2]
        x,y,theta=self.sensor_sta
        x,y=np.array([x,y])+np.dot(rotation,translation)
        x+=math.cos(theta)*T[0,2]-math.sin(theta)*T[1,2]
        y+=math.sin(theta)*T[0,2]+math.cos(theta)*T[1,2]
        theta+=delta_yaw
        self.sensor_sta=[x,y,theta]
        # s = self.sensor_sta
        # self.sensor_sta[0] = s[0] + math.cos(s[2])*T[0,2] - math.sin(s[2])*T[1,2]
        # self.sensor_sta[1] = s[1] + math.sin(s[2])*T[0,2] + math.cos(s[2])*T[1,2]
        # self.sensor_sta[2] = s[2] + delta_yaw
        print("sensor-global: ",self.sensor_sta)

        # tf broadcast
        s = self.sensor_sta
        q = tf.transformations.quaternion_from_euler(0,0,self.sensor_sta[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"icp_odom","world_base")

        # odom topic publish
        # FIXME: the code can be more concise...
        odom = Odometry(header=Header(0,rospy.Time.now(),"world_base"))
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_base"

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
        odom.pose.pose.position.z = 0.001
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)
        pass
    def laserToNumpy(self,msg):
        # the x,y coordinates are in robot's frame?
        total_num = len(msg.ranges)
        # pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        # ufunc, high performance
        pc = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc

    def calcDist(self,a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)

def main():
    rospy.init_node('icp_node')
    icp = ICP()
    rospy.spin()

if __name__ == '__main__':
    main()