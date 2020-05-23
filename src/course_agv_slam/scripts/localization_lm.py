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
import sys
class Localization():
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
        self.landMark_pub = rospy.Publisher('/landMarks',MarkerArray,queue_size=1)

    def updateMap(self):
        print("debug: try update map obstacle")
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map',GetMap)
            self.map = getMap().map
            # print(self.map)
        except:
            e = sys.exc_info()[0]
            print('Service call failed: %s'%e)
        # Update for planning algorithm
        map_data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose()
        tx,ty = np.nonzero((map_data > 20)|(map_data < -0.5))
        ox = (tx*self.map.info.resolution+self.map.info.origin.position.x)*1.0
        oy = (ty*self.map.info.resolution+self.map.info.origin.position.y)*1.0
        self.obstacle = np.vstack((ox,oy)).transpose()
        self.obstacle_r = self.map.info.resolution
        print("debug: update map obstacle success! ")

    def laserCallback(self,msg):
        u = self.calc_odometry(msg)
        z = self.ekf.odom_model(self.xEst,self.calc_map_observation(self.laserToNumpy(msg)))
        self.xEst,self.PEst = self.ekf.estimate(self.xEst,self.PEst,z,u)
        self.publishResult()
        pass

    def laserEstimation(self):
        # TODO
        return pc

    def calc_map_observation(self,msg):
        landmarks_src = self.extraction.process(msg,True)
        landmarks_tar = self.extraction.process(self.laserEstimation(),True)
        print("lm : ",len(landmarks_src.id),len(landmarks_tar.id))
        self.publishLandMark(landmarks_src,landmarks_tar)
        src_pc = self.lm2pc(landmarks_src)
        tar_pc = self.lm2pc(landmarks_tar)
        transform_acc = self.icp.process(tar_pc,src_pc)
        return self.T2u(transform_acc)

    def calc_odometry(self,msg):
        if self.isFirstScan:
            self.tar_pc = self.laserToNumpy(msg)
            self.isFirstScan = False
            return np.array([[0.0,0.0,0.0]]).T
        self.src_pc = self.laserToNumpy(msg)
        transform_acc = self.icp.process(self.tar_pc,self.src_pc)
        self.tar_pc = self.laserToNumpy(msg)
        return self.T2u(transform_acc)

    def lm2pc(self,lm):
        total_num = len(lm.id)
        dy = lm.position_y
        dx = lm.position_x
        range_l = np.hypot(dy,dx)
        angle_l = np.arctan2(dy,dx)
        pc = np.ones((3,total_num))
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        print("mdbg ",total_num)
        return pc
    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc
    def T2u(self,t):
        dw = math.atan2(t[1,0],t[0,0])
        u = np.array([[t[0,2],t[1,2],dw]])
        return u.T

    def publishResult(self):
        # tf
        s = self.xEst.reshape(-1)
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        self.odom_broadcaster.sendTransform((s[0],s[1],0.001),(q[0],q[1],q[2],q[3]),
                            rospy.Time.now(),"ekf_location","world_base")
        # odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_base"

        odom.pose.pose.position.x = s[0]
        odom.pose.pose.position.y = s[1]
        odom.pose.pose.position.z = 0.001
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.location_pub.publish(odom)

        s = self.xOdom
        q = tf.transformations.quaternion_from_euler(0,0,s[2])
        # odom
        odom = Odometry()
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

        # self.laser_pub.publish(self.target_laser)
        pass

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
