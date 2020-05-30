#!/usr/bin/env python
from sensor_msgs.msg import LaserScan

import numpy as np
import rospy
class LandMarkSet():
    def __init__(self):
        self.position_x = []
        self.position_y = []
        self.id = []

class Extraction():
    def __init__(self):
        self.range_threshold = float(rospy.get_param('/extraction/range_threshold',1.0))
        self.radius_max_th = float(rospy.get_param('/extraction/radius_max_th',0.3))
        self.landMark_min_pt = int(rospy.get_param('"/extraction/landMark_min_pt',2))

    def process(self,msg,trust = False):
        labels = []
        # I don't know what trust are, just ignoring them...
        ranges=np.array(msg.ranges)
        # add a ranges[0] at the end so that diff won't subtract the length by 1
        np.append(ranges,ranges[0])
        delta=np.diff(ranges)
        
        jumpPos=np.nonzero(np.abs(delta)>self.range_threshold)[0]
        np.append(jumpPos,jumpPos[0]+np.size(delta))
        for i in range(np.size(jumpPos)):
            points=jumpPos[i+1]-jumpPos[i]
            # ensure enough points.
            if points>=self.landMark_min_pt:
                # ensure not too large radius. A rough estimation, radius*angle
                if ranges[jumpPos[i]]*msg.angle_increment*points<=self.radius_max_th:
                    # This is a landmark, use the average index
                    labels.append((jumpPos[i]+jumpPos[i+1])//2)
        return self.extractLandMark(msg,labels,trust)
        
    #  What's the difference between the two functions?
    def extractLandMark(self,msg,labels,trust):
        landmark=LandMarkSet()
        ranges=np.array(msg.ranges)
        theta =np.linspace(msg.angle_min,msg.angle_max,len(msg.ranges))

        ranges=ranges[labels]
        theta =theta[labels]
        landmark.id=labels
        landmark.position_x=list(np.multiply(np.cos(theta),ranges))
        landmark.position_y=list(np.multiply(np.sin(theta),ranges))
        return landmark
