#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
# wow!
from icp import OdometryLocation,ICP
from ekf import EKF
import sys
import copy

class Localization(OdometryLocation):
    # direction: up, down, left, right
    directions=np.array([(0,1),(0,-1),(-1,0),(1,0)])
    inf=1e6
    def __init__(self):
        
        super(Localization,self).__init__()
        
        # very impressive...
        # TODO: change it into service
        # self.icp = ICP()
        self.ekf = EKF()
        
        # State Vector [x y yaw]'
        self.xOdom = np.zeros((3,1))
        self.xEst = np.zeros((3,1))
        self.PEst = np.eye(3) # the same with identity
                
        # map observation
        self.obstacle = []
        # radius
        self.obstacle_r = 10
        
        # init map
        self.updateMap()
        # ros topic
        self.laser=None
        self.laser_count=0
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.laser_pub = rospy.Publisher('/target_laser',LaserScan,queue_size=3)
        self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        
   

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
        # transposed (row,column)-> (x,y)
        
        self.map.data = np.array(self.map.data).reshape((-1,self.map.info.height)).transpose() 
        tx,ty = np.nonzero((self.map.data > 20)|(self.map.data < -0.5))
        obstacleList=[]
        for (x,y) in zip(tx,ty):
            # filter out the 4-connected ones.
            if not self.fourConnected((x,y)):
                obstacleList.append((x,y))
        originPos=np.array([self.map.info.origin.position.x,self.map.info.origin.position.y])
        # broadcasting
        self.obstacle=np.array(obstacleList)*self.map.info.resolution+originPos
        # ox = (tx*self.map.info.resolution+self.map.info.origin.position.x)*1.0
        # oy = (ty*self.map.info.resolution+self.map.info.origin.position.y)*1.0
        # self.obstacle = np.vstack((ox,oy)).transpose()
        print("obstacle list:\n{}".format(self.obstacle))
        self.obstacle_r = self.map.info.resolution/2
        print("debug: update map obstacle success! ")

    def laserCallback(self,msg):
        # TODO
        print('------seq:  ',msg.header.seq)
        self.calc_odometry(msg)
        # process once every 5 laser scan because laser fps is too high
        self.laser_count += 1
        if self.laser_count < self.laser_inteval:
            return
        self.laser_count = 0
        pass
    
    def laserEstimation(self,msg,x):
        '''
        Simulate the laser data from the estimated position.
        '''
        # laser is defined by the laser subscription
        data=self.laser
        data.seq+=1
        data.ranges=list(np.zeros_like(self.laser.ranges,dtype=float)+self.inf)
        for each in self.obstacle:
            # points from xEst to x
            dr=each-np.array(self.xEst[0:2])
            distance=np.linalg.norm(dr)
            dtheta=math.atan(self.obstacle_r,distance)
            theta=math.atan2(dr[1],dr[0])-self.xEst[2]
            angleRange=[theta-dtheta,theta+dtheta]
            # the index of laser covered
            indexRange=(np.array(angleRange)+np.pi)/self.laser.angle_increment
            for index in range(math.floor(indexRange[0]),math.ceil(indexRange[1])+1):
                if index>indexRange[0] and x<indexRange[1]:
                    if data.ranges[index]>distance:
                        data.ranges[index]=distance
        return data

    def calc_map_observation(self,msg):
        '''
        Using ICP to calculate the fitting Transform matrix between current laser
        and map laser. 
        Return: 3*3 matrix T.
        '''
        # TODO
        return transform_acc

    def calc_odometry(self,msg):
        '''
        get the relative odometry from icp.
        '''
        if self.isFirstScan:
            self.tar_pc = self.laserToNumpy(self.laserEstimation(msg,self.xEst))
            # print("ddddddddddddddddddd ",self.tar_pc - self.laserToNumpy(msg))
            self.isFirstScan = False
            return np.identity(3)
        self.src_pc = self.laserToNumpy(msg)
        transform_acc = self.icp.process(self.tar_pc,self.src_pc)
        self.tar_pc = self.laserToNumpy(msg)
        return transform_acc
    def fourConnected(self,pair):
        pair=np.array(pair)
        for dr in self.directions:
            newPair=pair+dr
            # detects if the obstacle is on the boundary
            if newPair[0]<0 or newPair[0]>=self.map.info.width or newPair[1]<0 or newPair[1]>=self.map.info.height:
                return False
            if self.map.data[tuple(newPair)]>-0.5 and self.map.data[tuple(newPair)]<20:
                return False
        return True


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
