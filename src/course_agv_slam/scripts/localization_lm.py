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
from icp import LandmarkICP,Localization
from ekf_lm import EKF_Landmark
from extraction import Extraction
# from localization import ICPLocalization

class LandmarkLocalization(Localization,EKF_Landmark):
    alpha=3.0
    def __init__(self,nodeName="ekf_lm"):
        super(LandmarkLocalization,self).__init__(nodeName)

        self.icp = LandmarkICP()
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

    # feed icp landmark instead of laser.
    def laserCallback(self,msg):
        print('------seq:  ',msg.header.seq)
        if self.isFirstScan:
            # feed in landmarks.
            self.icp.tar_pc=self.extraction.process(msg)
            self.isFirstScan = False
            return
        
        # Update once every 5 laser scan because the we cannot distinguish rotation if interval is too small.
        self.laser_count += 1
        if self.laser_count < self.laser_interval:
            return
        
        # Updating process
        self.laser_count = 0
        landmarks=self.extraction.process(msg,True)
        u=self.calc_odometry(landmarks)
        
        # z is the landmarks as a 2*n array.
        z=landmarks
        # xEst is both predicted and updated in the ekf.
        self.xEst,self.PEst=self.estimate(self.xEst,self.PEst,z,u)
        self.publishResult()
        return

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

    # def calc_map_observation(self,msg):
    #     landmarks_src = self.extraction.process(msg,True)
    #     # landmarks_tar = self.extraction.process(self.laserEstimation(),True)

    #     self.publishLandMark(landmarks_src,color="b")
    #     self.publishLandMark(landmarks_tar,color="g")
    #     src_pc = landmarks_src
    #     # tar_pc = landmarks_tar
    #     transform_acc = self.icp.processICP(tar_pc,src_pc)
    #     return self.T2u(transform_acc)
    #     pass

    def publishLandMark(self,msg,color="b"):
        '''
        msg=2*n np array.
        '''
        if len(msg.id) <= 0:
            return
        
        landMark_array = MarkerArray()
        landMark_array.markers=[self.toMarker(x,i,color) for i,x in enumerate(msg.T)]
        self.landMark_pub.publish(landMark_array)

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
        return marker

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
        neighbour=self.icp.findNearest(z,zEst)
        length=len(neighbour.src_indices)
        variance=self.alpha/(length+self.alpha)

        print("length: {} variance: {}".format(length,variance))

        zPredict=zEst[:,neighbour.tar_indices]

        m=self.jacob_h(self.tar_pc,neighbour,xPredict)
        # z (2*n)array->(2n*1) array
        zPredict=np.vstack(np.hsplit(zPredict,np.size(zPredict,1)))
        zPrime  =np.vstack(np.hsplit(z,np.size(z,1)))
        
        # Karman factor. Universal formula.
        K=np.dot(np.dot(covariance,m.T),np.linalg.inv(np.dot(m,np.dot(covariance,m.T))+np.diag([variance]*2*length)))

        # Update
        xEst=xPredict+np.dot(K,zPrime-zPredict)
        PEst=covariance-np.dot(K,np.dot(m,covariance))

        return xEst, PEst

def main():
    rospy.init_node('localization_node')
    l = LandmarkLocalization()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
