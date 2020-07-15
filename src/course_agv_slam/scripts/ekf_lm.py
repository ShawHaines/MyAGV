import math
import numpy as np
import tf
from ekf import EKF,STATE_SIZE,LM_SIZE,INF

M_DIST_TH = 0.6  # Threshold of Mahalanobis distance for data association.
# covariance of Odometry relative displacement u 
Cx = np.diag([0.35, 0.35, np.deg2rad(15.0)]) ** 2

class EKF_Landmark(EKF):
    def __init__(self):
        super(EKF_Landmark,self).__init__()

    def jacob_h(self, tar, neighbour, x):
        '''
        the Jacobian of observation model.
        '''
        length=len(neighbour.tar_indices)
        if length==0:
            print("Error: no matching points!")
            return 0
        rotation=tf.transformations.euler_matrix(0,0,x[2,0])[0:2,0:2]
        # the yaw(theta) derivative of rotation. [[-sin,cos],[-cos,-sin]]
        derivativeR=tf.transformations.euler_matrix(0,0,x[2,0]+math.pi/2)[0:2,0:2]
        z=tar[:,neighbour.tar_indices]
        partialTheta=np.dot(derivativeR.T,z-x[0:2])
        partialTheta=np.vstack(np.hsplit(partialTheta,length))
        H=np.repeat(-np.transpose(rotation),length,axis=0)
        H=np.hstack((H,partialTheta))
        return H

    def calc_landmark_position(self, x, z):
        zp = np.zeros((2, 1))

        zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
        zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

        return zp

    def calc_n_lm(self, x):
        n = int((len(x) - STATE_SIZE) / LM_SIZE)
        return n

    def get_landmark_position_from_state(self, x, ind):
        lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

        return lm

    def search_correspond_landmark_id(self, xAug, PAug, zi):
        """
        Landmark association with Mahalanobis distance
        """
        # don't know how to calculate Mahalanobis distance..
        # and it doesn't seem necessary either.
        nLM = self.calc_n_lm(xAug)

        min_dist = []

        ## TODO search and update min_dist

        min_dist.append(M_DIST_TH)  # new landmark

        min_id = min_dist.index(min(min_dist))

        return min_id

    def piRange(self, angle):
        '''
        normalize the angle in range [-pi,pi]
        '''
        return (angle + math.pi) % (2 * math.pi) - math.pi

class EKF_SLAM(EKF_Landmark):
    def __init__(self):
        super(EKF_SLAM,self).__init__()

    def jacob_motion(self,xEst,lEst,u):
        """
        Jacobian of Odom Model
        y = [x,y,w,...,xi,yi,...]T
        u = [dx,dy,dw]T
        returns (G,Fx)
        x_t=G*x_{t-1}+Fx*u
        """
        lSize=np.size(lEst,1)
        yLength=LM_SIZE*lSize+STATE_SIZE
        # the Jacobian for x
        G=np.identity(yLength)
        derivativeR=tf.transformations.euler_matrix(0,0,xEst[2,0]+np.pi/2)[0:3,0:3]
        G[0:3,2]=np.dot(derivativeR,u).reshape(-1)
        # the Jacobian for u
        Fx=np.zeros((yLength,STATE_SIZE))
        Fx[0:STATE_SIZE,0:STATE_SIZE]=tf.transformations.euler_matrix(0,0,xEst[2,0])[0:3,0:3]
        return (G,Fx)

    def jacob_h(self, landmark, neighbour, x):
        '''
        the Jacobian of observation model.
        '''
        zSize=len(neighbour.src_indices)
        lSize=np.size(landmark,1)
        if zSize==0:
            print("Error: no matching points!")
            return 0
        rotation=tf.transformations.euler_matrix(0,0,x[2,0])[0:2,0:2]
        # the yaw(theta) derivative of rotation. [[-sin,cos],[-cos,-sin]]
        derivativeR=tf.transformations.euler_matrix(0,0,x[2,0]+math.pi/2)[0:2,0:2]
        z=landmark[:,neighbour.tar_indices]
        partialTheta=np.dot(derivativeR.T,z-x[0:2])
        partialTheta=np.vstack(np.hsplit(partialTheta,zSize))
        H=np.repeat(-np.transpose(rotation),zSize,axis=0)
        H=np.hstack((H,partialTheta))
        # landmark part
        Hl=np.zeros((zSize,lSize))
        Hl[neighbour.src_indices,neighbour.tar_indices]=1
        # Excellent usage on kronecker multiplying!
        Hl=np.kron(Hl,rotation.T)
        # print("Hl=\n{}".format(Hl))
        return np.hstack((H,Hl))