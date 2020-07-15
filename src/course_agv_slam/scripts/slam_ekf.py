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
from icp import LandmarkICP,SubICP,NeighBor
from localization_lm import LandmarkLocalization
from ekf_lm import EKF_SLAM,STATE_SIZE,LM_SIZE,Cx,INF,OBSTACLE_RADIUS
from extraction import Extraction
import scipy.linalg as scilinalg
# import sys

MAX_LASER_RANGE = 30

class SLAM_Localization(LandmarkLocalization,EKF_SLAM):
    alpha=3.0  # factor in estimating covariance.
    def __init__(self,nodeName="slam_ekf"):
        super(SLAM_Localization,self).__init__(nodeName)

        self.icp = SubICP()
        self.extraction = Extraction()

        self.isFirstScan = True
        self.laser_count=0
        # interval
        self.laser_interval=5
        # State Vector [x y yaw].T, column vector.
        # self.xOdom = np.zeros((STATE_SIZE,1))
        self.xEst = np.zeros((STATE_SIZE,1))
        # Covariance. Initial is very certain.
        self.PEst = np.zeros((STATE_SIZE,STATE_SIZE))
        # landMark Estimation. Like former self.tar_pc
        self.lEst = np.zeros((LM_SIZE,0)) # lEst should be of 2*N size

        # ros topic
        self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        # self.location_pub = rospy.Publisher('ekf_location',Odometry,queue_size=3)
        
        ## localization parameters 
        # minimum landmark matches to update.
        self.min_match = int(rospy.get_param('/slam/min_match',2))
        # minimum number of points for a landmark cluster
        self.extraction.landMark_min_pt = int(rospy.get_param('/slam/landMark_min_pt',1))
        # maximum radius to be identified as landmark
        self.extraction.radius_max_th = float(rospy.get_param('/slam/radius_max_th',0.4))

        OBSTACLE_RADIUS=0.35

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
        # z is the landmarks in self frame as a 2*n array.
        z=self.extraction.process(msg,True)
        self.publishLandMark(z,"b")
        # relative displacement
        u=self.calc_odometry(msg)
        
        # xEst,lEst,PEst is both predicted and updated in the ekf.
        self.xEst,self.lEst,self.PEst=self.estimate(self.xEst,self.lEst,self.PEst,z,u)

        self.publishResult()
        return

    def calc_odometry(self,msg):
        '''
        Let icp handle the odometry, returns a relative odometry u.
        '''
        # laser callback is manually fed by its owner class.
        return self.icp.laserCallback(msg)

    # EKF virtual function.
    def observation_model(self,xEst,lEst):
        '''
        returns an 2*n column vector list.
        [z1,z2,....zn]
        '''
        rotation=tf.transformations.euler_matrix(0,0,xEst[2,0])[0:2,0:2]
        z=np.dot(rotation.T,lEst-xEst[0:2])
        return z

    def estimate(self,xEst,lEst,PEst,z,u):

        G,Fx=self.jacob_motion(xEst,lEst,u)
        # FIXME: the G and Fx transpose is confused. Thanks god they are all symmetric ones...
        covariance=np.dot(G,np.dot(PEst,G.T))+np.dot(Fx,np.dot(Cx,Fx.T))
        
        # Predict
        xPredict=self.odom_model(xEst,u)

        zEst=self.observation_model(xPredict,lEst)
        self.publishLandMark(zEst,color="r",namespace="estimate",frame=self.nodeName)

        neighbour=self.icp.findNearest(z,zEst)
        # Predicted
        zPredict=zEst[:,neighbour.tar_indices]
        self.publishLandMark(zPredict,color="g",namespace="paired",frame=self.nodeName)

        lSize=np.size(lEst,1)
        zSize=np.size(z,1)
        # how many observed landmarks are missed, how many new ones will be added.
        missedIndices=sorted(list(set(range(zSize))-set(neighbour.src_indices)))
        paired=len(neighbour.src_indices)
        missed=len(missedIndices)

        neighbour.src_indices+=missedIndices
        # the newly-added landmarks
        neighbour.tar_indices+=range(lSize,missed+lSize)

        zObserved=z[:,neighbour.src_indices]
        # add new landmarks to newL (new lEst)
        # newL=np.repeat(xPredict[0:2],missed,axis=1)
        # delicately select the spawning position...
        newL=np.dot(tf.transformations.euler_matrix(0,0,xPredict[2,0])[0:2,0:2],z[:,missedIndices])+xPredict[0:2]
        newL=np.hstack((lEst,newL))
        zEst=self.observation_model(xPredict,newL) 
        zPredict=zEst[:,neighbour.tar_indices]
        # the newly-added landmarks' uncertainty is very large
        # block is so powerful!
        covariance=scilinalg.block_diag(covariance,np.diag([INF]*LM_SIZE*missed))
        yPredict=self.XLtoY(xPredict,newL)

        variance=self.alpha/(zSize+self.alpha)*OBSTACLE_RADIUS
        print("\n\npaired: {} variance: {} missed: {} landmarks: {}".format(paired,variance,missed,lSize+missed))
        if zSize<self.min_match:
            print("Observed landmarks are too little to execute update.")
            #  only update according to the prediction stage.
            return xPredict,newL,covariance

        m=self.jacob_h(newL,neighbour,xPredict)

        # z (2*n)array->(2n*1) array
        zPredict =np.vstack(np.hsplit(zPredict,np.size(zPredict,1)))
        zObserved=np.vstack(np.hsplit(zObserved,np.size(zObserved,1)))
        # print("delta z: \n{}\n".format(zObserved-zPredict))
        
        # print("covariance:\n{}\n".format(covariance))
        # Karman factor. Universal formula.
        K=np.dot(np.dot(covariance,m.T),np.linalg.inv(np.dot(m,np.dot(covariance,m.T))+np.diag([variance**2]*2*zSize)))

        # Update
        fix=np.dot(K,zObserved-zPredict)
        # print("fix: \n{}\n\n".format(fix))
        yEst=yPredict+fix
        PEst=covariance-np.dot(K,np.dot(m,covariance))
        xEst,lEst=self.YtoXL(yEst)
        return xEst,lEst,PEst

    def XLtoY(self,x,l):
        # split y (2*N) to 2N*1
        y=np.vstack(np.hsplit(l,np.size(l,1)))
        y=np.vstack((x,y))
        return y

    def YtoXL(self,y):
        x,l=np.vsplit(y,[STATE_SIZE])
        l=np.hstack(np.vsplit(l,np.size(l)/LM_SIZE))
        return x,l

def main():
    rospy.init_node('slam_node')
    s = SLAM_Localization()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()
