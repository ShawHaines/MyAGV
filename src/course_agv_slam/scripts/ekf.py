import math
import numpy as np
from abc import ABCMeta, abstractmethod

# Covariance for EKF simulation
Q = np.diag([
    0.2,  # variance of location on x-axis
    0.2,  # variance of location on y-axis
    math.radians(3.0)  # variance of yaw angle
]) ** 2  # predict state covariance
R = np.diag([0.2, 0.2,math.radians(3.0)]) ** 2  # Observation x,y position covariance

class EKF(object):
    __metaclass__ = ABCMeta
    def __init__(self):
        pass
    def estimate(self, xEst, PEst, z, u):
        G,Fx=self.jacob_motion(xEst,u)
        GSquare=np.dot(G,G)
        FxSquare=np.dot(Fx,Fx)
        covariance=np.dot(GSquare,PEst)+np.dot(FxSquare,Q)
        m=self.jacob_h()

        # Predict
        xPredict=self.odom_model(xEst,u)
        zEst=self.observation_model(xPredict)

        # Karman factor. Universal formula.
        K=np.dot(np.dot(covariance,m.T),np.linalg.inv(np.dot(m,np.dot(covariance,m.T))+R))

        # Update
        xEst=xPredict+np.dot(K,z-zEst)
        PEst=covariance-np.dot(K,np.dot(m,covariance))

        return xEst, PEst

    def odom_model(self, x, u):
        """
            x = [x,y,w,...,xi,yi,...]T
            u = [ox,oy,ow]T
        """
        return x+u

    def jacob_motion(self, x, u):
        """
        Jacobian of Odom Model
        x = [x,y,w,...,xi,yi,...]T
        u = [ox,oy,ow]T
        returns (G,Fx)
        x_t=G*x_{t-1}+Fx*u
        """

        # the jacoby for x
        G=np.identity(3)
        # the jacoby for u
        Fx=np.identity(3)
        return (G,Fx)

    @abstractmethod # virtual.
    def observation_model(self,x):
        # TODO
        # This one needs to be overwritten.
        return x

    def jacob_h(self):
        '''
        the Jacobian of observation model.
        '''
        return np.identity(3)