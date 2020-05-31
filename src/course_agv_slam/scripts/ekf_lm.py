import math
import numpy as np
import tf
from ekf import EKF

M_DIST_TH = 0.6  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

class EKF_Landmark(EKF):
    # EKF state covariance
    Cx = np.diag([0.35, 0.35, np.deg2rad(15.0)]) ** 2

    def __init__(self):
        super(EKF_Landmark,self).__init__()

    def jacob_h(self, tar, neighbour, x):
        '''
        the Jacobian of observation model.
        '''
        length=len(neighbour.tar_indices)
        rotation=tf.transformations.euler_matrix(0,0,x[2,0])[0:2,0:2]
        # the yaw(theta) derivative of rotation. [[-sin,cos],[-cos,-sin]]
        derivativeR=tf.transformations.euler_matrix(0,0,x[2,0]+math.pi/2)[0:2,0:2]
        z=tar[:,neighbour.tar_indices]
        partialTheta=np.dot(derivativeR,z-x[0:2,0].reshape(2,1))
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
