#!/usr/bin/env python

import math
import numpy as np
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from mapping import MappingBase
from icp import toList,toArray

class Bayes_Mapping(MappingBase):
    def __init__(self):
        super(Bayes_Mapping,self).__init__()
        self.oddMap=np.zeros_like(self.pmap)
        self.transmit=0.3
        self.reflect=0.95

        self.transmitOdd=math.log(self.transmit/(1.0-self.transmit))
        self.reflectOdd =math.log(self.reflect/(1.0-self.reflect))
        
    def update(self, laserPC, center):
        """
        Use Bayes odds updating formula.
        """
        # change the points into integers
        start=list(np.round(np.array(center)/self.resolution).astype(int))
        for point in laserPC.T:
            end=list(np.round(point//self.resolution).astype(int))
            pointList=self.line(start,end)
            if np.size(pointList)==0:
                return
            # remove obstacle point.
            if list(pointList[0])==end:
                np.delete(pointList,0,axis=0)
            elif list(pointList[-1])==end:
                np.delete(pointList,-1,axis=0)
            # origin bias
            pointList+=self.origin
            # transmitted, decrease possibility
            self.oddMap[pointList[:,0],pointList[:,1]]+=self.transmitOdd
            # reflected on obstacles.
            self.oddMap[tuple(np.array(end)+self.origin)]+=self.reflectOdd
        self.pmap=1.0-1.0/(1+np.exp(self.oddMap))
        return

def main():
    rospy.init_node("mapping")
    bayesMapping=Bayes_Mapping()
    rospy.spin()

if __name__ == '__main__':
    main()
