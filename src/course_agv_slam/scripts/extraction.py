#!/usr/bin/env python
from sensor_msgs.msg import LaserScan

import numpy as np
import rospy

class Extraction():
    def __init__(self):
        self.range_threshold = float(rospy.get_param('/extraction/range_threshold',1.0))
        self.radius_max_th = float(rospy.get_param('/extraction/radius_max_th',0.4))
        self.landMark_min_pt = int(rospy.get_param('"/extraction/landMark_min_pt',2))

    def process(self,msg,trust = False):
        """
        returns pointcloud instead of Landmark objects.
        """
        labels = []
        # I don't know what trust are, just ignoring them...
        ranges=np.array(msg.ranges)

        theta =np.linspace(msg.angle_min,msg.angle_max,len(msg.ranges))
        pointCloud=np.vstack((np.multiply(np.cos(theta),ranges),np.multiply(np.sin(theta),ranges)))
        
        # add a index[0] at the end so that diff won't subtract the length by 1
        pointCloud=np.hstack((pointCloud,pointCloud[:,0].reshape(2,1)))
        delta=np.linalg.norm(np.diff(pointCloud),axis=0)
        # jump is between jumpPos and jumpPos+1
        jumpPos=np.nonzero(np.abs(delta)>self.range_threshold)[0]
        np.append(jumpPos,jumpPos[0]+np.size(delta))
        for i in range(np.size(jumpPos)-1):
            points=jumpPos[i+1]-jumpPos[i]
            # ensure enough points.
            if points>=self.landMark_min_pt:
                # ensure not too large radius. A rough estimation, radius*angle
                if ranges[jumpPos[i]]*msg.angle_increment*points<=self.radius_max_th*2:
                    # This is a landmark, use the average index
                    labels.append(((jumpPos[i]+jumpPos[i+1]+1)//2)%np.size(delta))
        return self.extractLandMark(msg,labels,trust)
        
    #  What's the difference between the two functions?
    def extractLandMark(self,msg,labels,trust):
        '''
        no longer use landMarkSet class. Use pointCloud (2*n array) as return value.
        '''
        ranges=np.array(msg.ranges)
        theta =np.linspace(msg.angle_min,msg.angle_max,len(msg.ranges))

        ranges=ranges[labels]
        theta =theta[labels]
        # id is not important...
        # landmark.id=labels
        landmark=np.vstack((np.multiply(np.cos(theta),ranges),np.multiply(np.sin(theta),ranges)))
        return landmark
