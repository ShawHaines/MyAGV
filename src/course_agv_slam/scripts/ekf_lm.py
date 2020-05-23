import math
import numpy as np

# EKF state covariance
Cx = np.diag([0.35, 0.35, np.deg2rad(15.0)]) ** 2

M_DIST_TH = 0.6  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

class EKF():
    def __init__(self):
        pass
    def estimate(self, xEst, PEst, z, u):
        # Predict
        ## TODO

        # Update
        ## TODO

        return xEst, PEst

    def odom_model(self, x, u):
        """
            x = [x,y,w,...,xi,yi,...]T
            u = [ox,oy,ow]T
        """
        ## TODO
        return x

    def calc_n_lm(self, x):
        n = int((len(x) - STATE_SIZE) / LM_SIZE)
        return n

    def jacob_motion(self, x, u):
        """
        Jacobian of Odom Model
        x = [x,y,w,...,xi,yi,...]T
        u = [ox,oy,ow]T
        """
        ## TODO
        return G, Fx,


    def calc_landmark_position(self, x, z):
        zp = np.zeros((2, 1))

        zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
        zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

        return zp


    def get_landmark_position_from_state(self, x, ind):
        lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

        return lm


    def search_correspond_landmark_id(self, xAug, PAug, zi):
        """
        Landmark association with Mahalanobis distance
        """

        nLM = self.calc_n_lm(xAug)

        min_dist = []

        ## TODO search and update min_dist

        min_dist.append(M_DIST_TH)  # new landmark

        min_id = min_dist.index(min(min_dist))

        return min_id

    def jacob_h(self, q, delta, x, i):
        ## TODO
        return H

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
