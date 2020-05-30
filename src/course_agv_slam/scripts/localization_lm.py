#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray,Marker
import numpy as np
from icp import ICP
from ekf import EKF
from extraction import LandMarkSet,Extraction
from localization import ICPLocalization
# import sys

class LandmarkLocalization(ICPLocalization):
    def __init__(self):
        self.icp = ICP()
        self.ekf = EKF()
        self.extraction = Extraction()

        # odom robot init states
        self.robot_x = rospy.get_param('/icp/robot_x',0)
        self.robot_y = rospy.get_param('/icp/robot_y',0)
        self.robot_theta = rospy.get_param('/icp/robot_theta',0)
        self.sensor_sta = [self.robot_x,self.robot_y,self.robot_theta]
        self.isFirstScan = True
        self.src_pc = []
        self.tar_pc = []

        # State Vector [x y yaw]
        self.xOdom = np.zeros((3,1))
        self.xEst = np.zeros((3,1))
        self.PEst = np.eye(3)
        
        # map observation
        self.obstacle = []
        # radius
        self.obstacle_r = 10

        # init map
        self.updateMap()
        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        self.odom_pub = rospy.Publisher('icp_odom',Odometry,queue_size=3)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.landMark_pub = rospy.Publisher('/landmarks',MarkerArray,queue_size=1)

    # There's something different.
    def laserCallback(self,msg):
        u = self.calc_odometry(msg)
        z = self.ekf.odom_model(self.xEst,self.calc_map_observation(self.icp.laserToNumpy(msg)))
        self.xEst,self.PEst = self.ekf.estimate(self.xEst,self.PEst,z,u)
        self.publishResult()
        pass

    def calc_map_observation(self,msg):
        landmarks_src = self.extraction.process(msg,True)
        landmarks_tar = self.extraction.process(self.laserEstimation(),True)
        print("lm : ",len(landmarks_src.id),len(landmarks_tar.id))
        self.publishLandMark(landmarks_src,landmarks_tar)
        src_pc = self.lm2pc(landmarks_src)
        tar_pc = self.lm2pc(landmarks_tar)
        transform_acc = self.icp.processICP(tar_pc,src_pc)
        return self.T2z(transform_acc)

    def calc_odometry(self,msg):
        if self.isFirstScan:
            self.tar_pc = self.laserToNumpy(msg)
            self.isFirstScan = False
            return np.array([[0.0,0.0,0.0]]).T
        self.src_pc = self.icp.laserToNumpy(msg)
        transform_acc = self.icp.processICP(self.tar_pc,self.src_pc)
        self.tar_pc = np.copy(self.src_pc)
        return self.T2z(transform_acc)

    def lm2pc(self,lm):
        total_num = len(lm.id)
        dx = lm.position_x
        dy = lm.position_y
        range_l = np.hypot(dy,dx)
        angle_l = np.arctan2(dy,dx)
        pc = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        print("total length= {}".format(total_num))
        return pc

    def publishLandMark(self,msg,msg2):
        # msg = LandMarkSet()
        if len(msg.id) <= 0:
            return
        
        landMark_array_msg = MarkerArray()
        for i in range(len(msg.id)):
            marker = Marker()
            marker.header.frame_id = "course_agv__hokuyo__link"
            marker.header.stamp = rospy.Time(0)
            marker.ns = "lm"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = msg.position_x[i]
            marker.pose.position.y = msg.position_y[i]
            marker.pose.position.z = 0 # 2D
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 0.5 # Don't forget to set the alpha
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            landMark_array_msg.markers.append(marker)
        for i in range(len(msg2.id)):
            marker = Marker()
            marker.header.frame_id = "course_agv__hokuyo__link"
            marker.header.stamp = rospy.Time(0)
            marker.ns = "lm"
            marker.id = i+len(msg.id)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = msg2.position_x[i]
            marker.pose.position.y = msg2.position_y[i]
            marker.pose.position.z = 0 # 2D
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 0.5 # Don't forget to set the alpha
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            landMark_array_msg.markers.append(marker)
        self.landMark_pub.publish(landMark_array_msg)

def main():
    rospy.init_node('localization_node')
    l = Localization()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
