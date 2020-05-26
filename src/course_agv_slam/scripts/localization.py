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
from icp import OdometryLocation,ICPBase
from ekf import EKF
import sys
import copy

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

        T=self.processICP(self.src_pc,self.tar_pc)
        self.tar_pc = np.copy(self.src_pc) # moving the target to src

        self.publishResult(T)
        duration=rospy.Time.now()-time_0
        print("time_cost: {} s".format(duration.to_sec()))
        pass


class Localization(OdometryLocation):
    # direction: up, down, left, right
    directions=np.array([(0,1),(0,-1),(-1,0),(1,0)])
    inf=1e6
    def __init__(self):
        
        super(Localization,self).__init__()
        
        # very impressive...
        # ICP node is included to access its processICP() method directly.

        self.icp = SubICP()
        self.ekf = EKF()
        
        # State Vector [x y yaw]'
        self.xOdom = np.zeros(3)
        self.xEst = np.zeros(3)
        # What exactly is this PEst?
        self.PEst = np.eye(3) # the same with identity
                
        # map obstacle
        self.obstacle = []
        # radius
        self.obstacle_r = 10
        
        # init map
        self.updateMap()
        # ros topic
        # self.laser=None
        self.laser_count=0
        # self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
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
            # filter out the 4-connected ones. Extract only the edges.
            if not self.fourConnected((x,y)):
                obstacleList.append((x,y))
        originPos=np.array([self.map.info.origin.position.x,self.map.info.origin.position.y])
        # broadcasting
        self.obstacle=np.array(obstacleList)*self.map.info.resolution+originPos
        # ox = (tx*self.map.info.resolution+self.map.info.origin.position.x)*1.0
        # oy = (ty*self.map.info.resolution+self.map.info.origin.position.y)*1.0
        # self.obstacle = np.vstack((ox,oy)).transpose()
        print("obstacle list:\n{}".format(self.obstacle))
        # a conservative estimation.
        self.obstacle_r = self.map.info.resolution/1.4 # a little bigger than sqrt(2)/2
        print("debug: update map obstacle success! ")

    def laserCallback(self,msg):
        '''
        laser ICP odometry.
        '''
        print('------seq:  ',msg.header.seq)
        if self.isFirstScan:
            self.tar_pc = self.laserToNumpy(self.laserEstimation(msg,self.xEst))
            self.icp.tar_pc=msg
            # print("ddddddddddddddddddd ",self.tar_pc - self.laserToNumpy(msg))
            self.isFirstScan = False
            return
        
        # Update once every 5 laser scan because laser fps is too high
        self.laser_count += 1
        if self.laser_count < self.laser_inteval:
            print("skipped.")
            return
        
        # Updating process
        self.laser_count = 0
        # laser callback is manually fed by its owner class.
        state0=np.copy(self.icp.sensor_sta)
        self.icp.laserCallback(msg)
        # relative state.
        u=np.array(self.icp.sensor_sta)-state0
        T=self.icp.processICP(self.laserEstimation(msg,self.xEst),self.tar_pc)
        # z is the absolute states.
        z=self.T2u(T)
        self.ekf.estimate(self.xEst,self.PEst,z,u)
        # self.laser_pub.publish(self.laserEstimation(msg,self.xEst))
        pass
    
    def laserEstimation(self,msg,x):
        '''
        Simulate the laser data from the estimated position x. msg is the reference laser.
        '''
        # laser is defined by the laser subscription
        # short and elegent implementation!
        data=msg
        data.header.seq+=1
        data.ranges=list(np.zeros_like(msg.ranges,dtype=float)+self.inf)
        for each in self.obstacle:
            # points from x to each
            dr=each-np.array(x[0:2])
            distance=np.linalg.norm(dr)
            dtheta=math.asin(self.obstacle_r/distance)
            theta=math.atan2(dr[1],dr[0])-x[2]
            angleRange=[theta-dtheta,theta+dtheta]
            # the index of laser covered
            indexRange=(np.array(angleRange)+np.pi)/msg.angle_increment
            # print(indexRange)
            distance-=self.obstacle_r
            for index in range(int(indexRange[0]),int(math.ceil(indexRange[1])+1)):
                if index>indexRange[0] and index<indexRange[1]:
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
        # return transform_acc
        pass

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
    
    def T2u(self,t):
        dw = math.atan2(t[1,0],t[0,0])
        u = np.array([[t[0,2],t[1,2],dw]])
        # .T can be viewed as transpose in 2 dimentional matrix.
        # but here u is only 1-D..
        # return u.T
        return u

def main():
    rospy.init_node('localization_node')
    l = Localization()
    rospy.spin()
    pass

# def test():
#     pass

if __name__ == '__main__':
    main()
    # test()
