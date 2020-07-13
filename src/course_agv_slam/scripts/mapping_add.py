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

class Mapping(MappingBase):
    def __init__(self):
        super(Mapping,self).__init__()
        # ray tracing update factor
        self.weight=0.02

    def update(self, laserPC, center):
        """
        Use adding rules updating formula.
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
            self.pmap[pointList[:,0],pointList[:,1]]-=self.weight
            # reflected on obstacles.
            self.pmap[tuple(np.array(end)+self.origin)]+=10*self.weight

        self.pmap[self.pmap>1]=1
        self.pmap[self.pmap<0]=0
        return

def main():
    rospy.init_node("mapping")
    mapping=Mapping()
    rospy.spin()

if __name__ == '__main__':
    main()
