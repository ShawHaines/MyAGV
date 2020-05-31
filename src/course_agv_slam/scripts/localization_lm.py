#!/usr/bin/env python
import rospy
import tf
import math
import sys
from std_msgs.msg import Header,ColorRGBA
from geometry_msgs.msg import Point,Quaternion,Vector3
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray,Marker
import numpy as np
from icp import ICP,Localization
from ekf import EKF
from extraction import LandMarkSet,Extraction
# from localization import ICPLocalization

class LandmarkLocalization(Localization):
    def __init__(self,nodeName="ekf_lm"):
        super(LandmarkLocalization,self).__init__(nodeName)

        self.icp = ICP()
        self.extraction = Extraction()

        self.src_pc = None
        self.isFirstScan = True
        self.laser_count=0
        # interval
        self.laser_interval=5

        # State Vector [x y yaw].T, column vector.
        # self.xOdom = np.zeros((3,1))
        self.xEst = np.zeros((3,1))
        # Covariance.
        self.PEst = np.eye(3)
        
        # init map
        # map observation
        self.tar_pc = None
        self.updateMap()

        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.landMark_pub = rospy.Publisher('/landmarks',MarkerArray,queue_size=1)
        # self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)

    def updateMap(self):
        '''
        get all the isolated pixels in the map as landmarks,
        then extract the positions of landmarks into self.tar_pc
        '''
        print("debug: try update map obstacle")
        rospy.wait_for_service('/static_map')
        try:
            getMap = rospy.ServiceProxy('/static_map',GetMap)
            self.map = getMap().map
            # print(self.map)
        except:
            e = sys.exc_info()[0]
            print('Service call failed: %s'%e)
        
        # transposed (row,column)-> (x,y)
        self.map.data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose() 
        tx,ty = np.nonzero((self.map.data > 20)|(self.map.data < -0.5))
        landmarkList=[]
        for (x,y) in zip(tx,ty):
            if self.isolated((x,y)):
                landmarkList.append((x,y))
        originPos=np.array([self.map.info.origin.position.x,self.map.info.origin.position.y])
        # numpy's broadcasting feature
        self.tar_pc=(np.array(landmarkList)*self.map.info.resolution+originPos).T
        print("landmark list:\n{}".format(self.tar_pc))
        print("debug: update map obstacle success! ")

    def isolated(self,pair):
        # direction: up, down, left, right
        directions=np.array([(0,1),(0,-1),(-1,0),(1,0)])
        pair=np.array(pair)
        for dr in directions:
            newPair=pair+dr
            # detects if the obstacle is on the boundary
            if newPair[0]<0 or newPair[0]>=self.map.info.width or newPair[1]<0 or newPair[1]>=self.map.info.height:
                continue
            if self.map.data[tuple(newPair)]<-0.5 or self.map.data[tuple(newPair)]>20:
                return False
        return True

    # There's something different.
    def laserCallback(self,msg):
        print('------seq:  ',msg.header.seq)
        if self.isFirstScan:
            self.icp.tar_pc=self.icp.laserToNumpy(msg)
            self.isFirstScan = False
            return
        
        # Update once every 5 laser scan because the we cannot distinguish rotation if interval is too small.
        self.laser_count += 1
        if self.laser_count < self.laser_interval:
            return
        
        # Updating process
        self.laser_count = 0
        self.calc_odometry(msg)
        
        # z is the absolute states.
        z=self.calc_map_observation(msg)
        # xEst is both predicted and updated in the ekf.
        self.xEst,self.PEst=self.estimate(self.xEst,self.PEst,z,u)
        self.publishResult()
        pass

        u = self.calc_odometry(msg)
        z = self.ekf.odom_model(self.xEst,self.calc_map_observation(self.icp.laserToNumpy(msg)))
        self.xEst,self.PEst = self.ekf.estimate(self.xEst,self.PEst,z,u)
        self.publishResult()
        pass

    def calc_odometry(self,msg):
        '''
        Let icp handle the odometry, returns a relative odometry u.
        '''
        state0=np.copy(self.icp.xEst)
        # laser callback is manually fed by its owner class.
        self.icp.laserCallback(msg)
        # relative state.
        u=self.icp.xEst-state0
        return u

    def calc_map_observation(self,msg):
        landmarks_src = self.extraction.process(msg,True)
        # landmarks_tar = self.extraction.process(self.laserEstimation(),True)

        self.publishLandMark(landmarks_src,landmarks_tar)
        src_pc = landmarks_src
        # tar_pc = landmarks_tar
        transform_acc = self.icp.processICP(tar_pc,src_pc)
        return self.T2z(transform_acc)

    def publishLandMark(self,msg,msg2):
        # msg = LandMarkSet()
        # TODO: change the input interface into pc 
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

    def toMarker(self,pair,id=0,color="b"):
        '''
        color can be "r","g","b"
        '''
        # order: R,G,B,Alpha. Don't forget to set the alpha.
        colorDict={
            "r":[1,0,0,0.5], "R":[1,0,0,0.5],
            "g":[0,1,0,0.5], "G":[0,1,0,0.5],
            "b":[0,0,1,0.5], "B":[0,0,1,0.5],
        }
        marker = Marker(header=Header(id,rospy.Time(0),"course_agv__hokuyo__link"))
        marker.ns = "lm"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position=Point(pair[0],pair[1],0)
        marker.pose.orientation=Quaternion(0,0,0,1)
        marker.scale=Vector3(0.2,0.2,0.2)
        # * is a very useful grammar when passing arguments! 
        # expands the list or tuple in order. 
        marker.color=ColorRGBA(*colorDict[color])
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
