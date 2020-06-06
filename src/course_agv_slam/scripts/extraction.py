#!/usr/bin/env python
from sensor_msgs.msg import LaserScan

import numpy as np
import rospy

class Extraction():
    def __init__(self):
        self.range_threshold = float(rospy.get_param('/extraction/range_threshold',1.0))
        self.radius_max_th = float(rospy.get_param('/extraction/radius_max_th',0.3))
        self.landMark_min_pt = int(rospy.get_param('"/extraction/landMark_min_pt',2))

    def process(self,msg,trust = False):
        labels = []
        # I don't know what trust are, just ignoring them...
        ranges=np.array(msg.ranges)
        length=np.size(ranges)
        # add a ranges[0] at the end so that diff won't subtract the length by 1
        np.append(ranges,ranges[0])
        delta=np.diff(ranges)
        # the jump is between jumpPos and jumpPos+1
        jumpPos=np.nonzero(np.abs(delta)>self.range_threshold)[0] # use [0] because return value is a tuple.
        for i in range(np.size(jumpPos)):
            if i<np.size(jumpPos)-1:
                points=jumpPos[i+1]-jumpPos[i]
            else:
                points=jumpPos[0]+length-jumpPos[i]
            # ensure enough points.
            if points>=self.landMark_min_pt:
                # ensure not too large radius. A rough estimation, radius*angle
                if ranges[jumpPos[i]]*msg.angle_increment*points<=self.radius_max_th*2:
                    # store a tuple into labels.
                    labels.append((jumpPos[i],jumpPos[i+1 if i+1<np.size(jumpPos) else 0]))
        return self.extractLandMark(msg,labels,trust)
        
    #  What's the difference between the two functions?
    def extractLandMark(self,msg,labels,trust):
        '''
        no longer use landMarkSet class. Use pointCloud (2*n array) as return value.
        '''
        ranges=np.array(msg.ranges)
        total_num=len(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        pc = np.vstack((np.multiply(np.cos(angle_l),ranges),np.multiply(np.sin(angle_l),ranges)))

        length=len(labels)
        landmark=np.zeros((2,length))
        for index,(i,j) in enumerate(labels):
            if i<j:
                landmark[:,index]=np.mean(pc[:,np.arange(i+1,j+1)],axis=1)
            else:
                landmark[:,index]=np.mean(pc[:,np.hstack((np.arange(i+1,total_num),np.arange(j+1)))],axis=1)
        return landmark
