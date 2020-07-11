#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray,Marker
from nav_msgs.msg import OccupancyGrid
import numpy as np
from icp import LandmarkICP,SubICP
from localization_lm import LandmarkLocalization
from ekf_lm import EKF_Landmark,STATE_SIZE,LM_SIZE
from extraction import Extraction
from mapping import Mapping
# import sys

MAX_LASER_RANGE = 30

class SLAM_EKF(LandmarkLocalization,EKF_Landmark):
    alpha=3.0  # factor in estimating covariance.
    def __init__(self,nodeName="slam_ekf"):
        super(SLAM_EKF,self).__init__(nodeName)

        self.icp = SubICP()
        self.extraction = Extraction()
        self.mapping = Mapping(self.map_cellx_width,self.map_celly_width,self.map_reso)

        self.src_pc = None
        self.isFirstScan = True
        self.laser_count=0
        # interval
        self.laser_interval=5
        # State Vector [x y yaw].T, column vector.
        # self.xOdom = np.zeros((STATE_SIZE,1))
        self.xEst = np.zeros((STATE_SIZE,1))
        # Covariance.
        self.PEst = np.eye(STATE_SIZE)

        # FIXME: What are those?
        # map observation
        self.obstacle = []
        # radius
        self.obstacle_r = 10

        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.landMark_pub = rospy.Publisher('/landmarks',MarkerArray,queue_size=3)
        # self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        self.map_pub = rospy.Publisher('/slam_map',OccupancyGrid,queue_size=1)

        # ros parameters
        self.robot_x = float(rospy.get_param('/slam/robot_x',0))
        self.robot_y = float(rospy.get_param('/slam/robot_y',0))
        self.robot_theta = float(rospy.get_param('/slam/robot_theta',0))
        ## mapping parameters
        self.map_x_width = float(rospy.get_param('/slam/map_width'))
        self.map_y_width = float(rospy.get_param('/slam/map_height'))
        self.map_reso = float(rospy.get_param('/slam/map_resolution'))
        self.map_cellx_width = int(round(self.map_x_width/self.map_reso))
        self.map_celly_width = int(round(self.map_y_width/self.map_reso))
        ## localization parameters 
        # minimum landmark matches to update.
        self.min_match = int(rospy.get_param('/slam/min_match',2))
        # minimum number of points for a landmark cluster
        self.extraction.landMark_min_pt = int(rospy.get_param('/slam/landMark_min_pt',2))
        # maximum radius to be identified as landmark
        self.extraction.radius_max_th = float(rospy.get_param('/slam/radius_max_th',0.4))

    # feed icp landmark instead of laser.
    def laserCallback(self,msg):
        np_msg = self.laserToNumpy(msg)
        lm = self.extraction.process(np_msg)
        # u = self.calc_odometry(self.lm2pc(lm))
        u = self.calc_odometry(np_msg)
        z = self.observation(lm)
        self.xEst,self.PEst = self.ekf.estimate(self.xEst,self.PEst,z,u)

        # FIXME
        obs = self.u2T(self.xEst[0:3]).dot(np_msg)
        pmap = self.mapping.update(obs[0], obs[1], self.xEst[0], self.xEst[1])

        self.publishMap(pmap)
        self.publishLandMark(lm)
        self.publishResult()
        pass

    def observation(self,lm):
        landmarks = lm
        z = np.zeros((0, 3)) # 0 length...
        for i in range(len(landmarks.id)):
            dx = landmarks.position_x[i]
            dy = landmarks.position_y[i]
            d = math.hypot(dx, dy)
            angle = self.ekf.pi_2_pi(math.atan2(dy, dx))
            zi = np.array([d, angle, i])
            z = np.vstack((z, zi))
        return z

    def calc_odometry(self,np_msg):
        if self.isFirstScan:
            self.tar_pc = np_msg
            self.isFirstScan = False
            return np.array([[0.0,0.0,0.0]]).T
        self.src_pc = np_msg
        transform_acc = self.icp.process(self.tar_pc,self.src_pc)
        self.tar_pc = np_msg
        return self.T2u(transform_acc)

    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        range_l[range_l == np.inf] = MAX_LASER_RANGE
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
        return pc
        
    def publishMap(self,pmap):
        map_msg = OccupancyGrid()
        map_msg.header.seq = 1
        map_msg.header.stamp = rospy.Time().now()
        map_msg.header.frame_id = "map"

        map_msg.info.map_load_time = rospy.Time().now()
        map_msg.info.resolution = self.map_reso
        map_msg.info.width = self.map_cellx_width
        map_msg.info.height = self.map_celly_width
        map_msg.info.origin.position.x = -self.map_cellx_width*self.map_reso/2.0
        map_msg.info.origin.position.y = -self.map_celly_width*self.map_reso/2.0
        map_msg.info.origin.position.z = 0
        map_msg.info.origin.orientation.x = 0
        map_msg.info.origin.orientation.y = 0
        map_msg.info.origin.orientation.z = 0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = list(pmap.T.reshape(-1)*100)
        
        self.map_pub.publish(map_msg)

def main():
    rospy.init_node('slam_node')
    s = SLAM_EKF()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
