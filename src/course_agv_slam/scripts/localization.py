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
from icp import Localization,ICPBase
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

        T=self.processICP(self.src_pc,self.tar_pc,initialT=np.identity(3))
        self.tar_pc = np.copy(self.src_pc) # moving the target to src
        self.translateResult(T)
        self.publishResult()
        duration=rospy.Time.now()-time_0
        print("time_cost: {} s".format(duration.to_sec()))
        pass


class ICPLocalization(Localization,EKF):
    inf=1e2
    def __init__(self,nodeName="ekf_icp"):

        # it only calls the Localization __init__ method. but EKF doesn't need init.
        super(ICPLocalization,self).__init__(nodeName)
        # EKF.__init__(self)
        
        # In order to avoid IO and communication, 
        # ICP node is included to access its processICP() method directly.
        self.icp = SubICP()
        
        # something needed like icp.
        
        self.tar_pc=None
        self.isFirstScan=True
        self.laserTemplate=LaserScan()
        self.laser_count=0
        # interval
        self.laser_interval=5

        # State Vector [x y yaw].T, column vector.
        # self.xOdom = np.zeros((3,1))
        self.xEst = np.zeros((3,1))
        
        # P is the covariance.
        self.PEst = np.eye(3) # the same with identity
                
        # map obstacle
        self.obstacle = []
        # radius
        self.obstacle_r = 10
        
        # init map
        self.updateMap()
        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.laser_pub = rospy.Publisher('/target_laser',LaserScan,queue_size=3)
        # self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        
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
            self.laserTemplate=msg
            self.laserTemplate.header.frame_id=("ekf_icp")
            # self.tar_pc = self.icp.laserToNumpy(self.laserEstimation(self.xEst))
            self.tar_pc=self.icp.laserToNumpy(msg)
            self.icp.tar_pc=self.icp.laserToNumpy(msg)
            # print("ddddddddddddddddddd ",self.tar_pc - self.laserToNumpy(msg))
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
    
    def laserEstimation(self,x):
        '''
        Simulate the laser data from the estimated position x. msg is the reference laser.
        '''
        # laser is defined by the laser subscription
        # short and elegant implementation!
        print("\n\nlaserEstimation x=\n{}\n\n".format(x))
        self.laserTemplate.header.seq+=1
        data=self.laserTemplate
        data.ranges=[self.inf]*len(data.ranges)
        data.header.stamp=rospy.Time(0)
        for each in self.obstacle:
            # points from x to each
            dr=each-np.array(x[0:2,0])
            distance=np.linalg.norm(dr)
            if distance<self.obstacle_r:
                # it means our xEst has stepped into an obstacle, 
                # better pretend the obstacle isn't there... 
                # But it may break the thin obstacle edge.
                continue
            dtheta=math.asin(self.obstacle_r/distance)
            theta=math.atan2(dr[1],dr[0])-x[2,0]
            while theta<-np.pi:
                theta+=2*np.pi
            while theta>np.pi:
                theta-=2*np.pi
            angleRange=[theta-dtheta,theta+dtheta]
            # the index of laser covered
            indexRange=(np.array(angleRange)+np.pi)/data.angle_increment
            # print(indexRange)
            distance-=self.obstacle_r
            for index in range(int(indexRange[0]),int(math.ceil(indexRange[1])+1)):
                if index>indexRange[0] and index<indexRange[1]:
                    tempIndex=index%len(data.ranges)
                    if data.ranges[tempIndex]>distance:
                        data.ranges[tempIndex]=distance
        # for debugging
        self.laser_pub.publish(data)
        return data

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
        '''
        Using ICP to calculate the fitting Transform matrix between current laser
        and map laser. 
        Return: column vector z.
        '''
        # T=self.icp.processICP(self.icp.laserToNumpy(msg),self.tar_pc,initialT=self.x2T(self.xEst))
        T=self.icp.processICP(self.icp.laserToNumpy(msg),self.tar_pc,initialT=np.identity(3))
        z=self.T2u(T)
        return z

    def fourConnected(self,pair):
        # direction: up, down, left, right
        directions=np.array([(0,1),(0,-1),(-1,0),(1,0)])
        pair=np.array(pair)
        for dr in directions:
            newPair=pair+dr
            # detects if the obstacle is on the boundary
            if newPair[0]<0 or newPair[0]>=self.map.info.width or newPair[1]<0 or newPair[1]>=self.map.info.height:
                return False
            if self.map.data[tuple(newPair)]>-0.5 and self.map.data[tuple(newPair)]<20:
                return False
        return True
    
    # EKF virtual function.
    def observation_model(self,x):
        return self.calc_map_observation(self.laserEstimation(x))

def main():
    rospy.init_node('localization_node')
    l = ICPLocalization()
    rospy.spin()
    pass

# def test():
#     pass

if __name__ == '__main__':
    main()
    # test()
