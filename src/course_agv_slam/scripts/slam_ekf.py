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

        # ros parameters
        self.robot_x = float(rospy.get_param('/slam/robot_x',0))
        self.robot_y = float(rospy.get_param('/slam/robot_y',0))
        self.robot_theta = float(rospy.get_param('/slam/robot_theta',0))
        ## mapping parameters
        self.map_x_width = float(rospy.get_param('/slam/map_width',25))
        self.map_y_width = float(rospy.get_param('/slam/map_height',25))
        self.map_reso = float(rospy.get_param('/slam/map_resolution',0.1))
        self.map_cellx_width = int(round(self.map_x_width/self.map_reso))
        self.map_celly_width = int(round(self.map_y_width/self.map_reso))
        
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

        # init map
        # map observation
        self.tar_pc = None
        self.updateMap()

        # FIXME: What are those?
        # map observation
        self.obstacle = []
        # radius
        self.obstacle_r = 10

        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        # self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        self.map_pub = rospy.Publisher('/slam_map',OccupancyGrid,queue_size=1)
        
        ## localization parameters 
        # minimum landmark matches to update.
        self.min_match = int(rospy.get_param('/slam/min_match',2))
        # minimum number of points for a landmark cluster
        self.extraction.landMark_min_pt = int(rospy.get_param('/slam/landMark_min_pt',2))
        # maximum radius to be identified as landmark
        self.extraction.radius_max_th = float(rospy.get_param('/slam/radius_max_th',0.4))

    # feed icp landmark instead of laser.
    def laserCallback(self,msg):
        print('------seq:  ',msg.header.seq)
        if self.isFirstScan:
            # feed in landmarks.
            # self.icp.tar_pc=self.extraction.process(msg)
            self.icp.tar_pc=self.icp.laserToNumpy(msg)
            self.isFirstScan = False
            return
        
        # Update once every 5 laser scan because the we cannot distinguish rotation if interval is too small.
        self.laser_count += 1
        if self.laser_count < self.laser_interval:
            return

        # Updating process
        self.laser_count = 0
        landmarks=self.extraction.process(msg,True)
        self.publishLandMark(landmarks,"b")
        u=self.calc_odometry(msg)
        
        # z is the landmarks as a 2*n array.
        z=landmarks
        # xEst is both predicted and updated in the ekf.
        self.xEst,self.PEst=self.estimate(self.xEst,self.PEst,z,u)
        self.publishResult()

        # np_msg = self.laserToNumpy(msg)
        # lm = self.extraction.process(np_msg)
        # # u = self.calc_odometry(self.lm2pc(lm))
        # u = self.calc_odometry(np_msg)
        # z = self.observation(lm)
        # self.xEst,self.PEst = self.ekf.estimate(self.xEst,self.PEst,z,u)

        # # FIXME
        # pointCloud = self.u2T(self.xEst[0:3]).dot(np_msg)
        pointCloud=np.dot(tf.transformations.euler_matrix(0,0,self.xEst[2,0])[0:2,0:2],self.icp.laserToNumpy(msg))+self.xEst[0:2]
        pmap = self.mapping.update(pointCloud, self.xEst[0:2].reshape(-1))

        self.publishMap(pmap)
        pass

    def calc_odometry(self,msg):
        '''
        Let icp handle the odometry, returns a relative odometry u.
        '''
        state0=np.copy(self.icp.xEst)
        # laser callback is manually fed by its owner class.
        # self.icp.laserCallback(self.extraction.process(msg))
        self.icp.laserCallback(msg)
        # relative state.
        u=self.icp.xEst-state0
        return u

    # EKF virtual function.
    def observation_model(self,xEst):
        '''
        returns an 2*n column vector list.
        [z1,z2,....zn]
        '''
        rotation=tf.transformations.euler_matrix(0,0,xEst[2,0])[0:2,0:2]
        z=np.dot(rotation.T,self.tar_pc-xEst[0:2,0].reshape(2,1))
        return z

    def estimate(self,xEst,PEst,z,u):
        G,Fx=self.jacob_motion(xEst,u)
        covariance=np.dot(G.T,np.dot(PEst,G))+np.dot(Fx.T,np.dot(self.Cx,Fx))
        
        # Predict
        xPredict=self.odom_model(xEst,u)
        zEst=self.observation_model(xPredict)
        self.publishLandMark(zEst,color="r",namespace="estimate",frame=self.nodeName)

        neighbour=self.icp.findNearest(z,zEst)
        zPredict=zEst[:,neighbour.tar_indices]
        zPrime=z[:,neighbour.src_indices]

        length=len(neighbour.src_indices)
        variance=self.alpha/(length+self.alpha)
        print("\n\nlength: {} variance: {}".format(length,variance))
        if length<self.min_match:
            print("Matching points are too little to execute update.")
            #  only update according to the prediction stage.
            return xPredict, covariance

        self.publishLandMark(zPredict,color="g",namespace="paired",frame=self.nodeName)
        m=self.jacob_h(self.tar_pc,neighbour,xPredict)

        # z (2*n)array->(2n*1) array
        zPredict=np.vstack(np.hsplit(zPredict,np.size(zPredict,1)))
        zPrime  =np.vstack(np.hsplit(zPrime,np.size(zPrime,1)))
        print("delta z: \n{}\n\n".format(zPrime-zPredict))
        
        # Karman factor. Universal formula.
        K=np.dot(np.dot(covariance,m.T),np.linalg.inv(np.dot(m,np.dot(covariance,m.T))+np.diag([variance]*2*length)))

        # Update
        xEst=xPredict+np.dot(K,zPrime-zPredict)
        PEst=covariance-np.dot(K,np.dot(m,covariance))

        return xEst, PEst

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
        self.tar_pc=np.transpose(np.array(landmarkList)*self.map.info.resolution+originPos)
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
