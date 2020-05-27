import math
import numpy as np


# Covariance for EKF simulation
Q = np.diag([
    0.2,  # variance of location on x-axis
    0.2,  # variance of location on y-axis
    math.radians(3.0)  # variance of yaw angle
]) ** 2  # predict state covariance
R = np.diag([0.2, 0.2,math.radians(3.0)]) ** 2  # Observation x,y position covariance

class EKF(object):
    def __init__(self):
        pass
    def estimate(self, xEst, PEst, z, virtual_z):
        # Predict
        xEst=self.odom_model(xEst,u)
        # How can this one call icp?

        # Update
        ## TODO

        return xEst, PEst

    def odom_model(self, x, u):
        """
            x = [x,y,w,...,xi,yi,...]T
            u = [ox,oy,ow]T
        """
        x=x+u
        return x

    def jacob_motion(self, x, u):
        """
        Jacobian of Odom Model
        x = [x,y,w,...,xi,yi,...]T
        u = [ox,oy,ow]T
        """
        ## TODO
        return jH

    def observation_model(self,x):
        # TODO
        # This one needs to be overwritten.
        return x

    def jacob_h(self):
        '''
        the Jacobian of observation model.
        '''
        return np.identity(3)