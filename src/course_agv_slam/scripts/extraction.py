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
        self.range_threshold = rospy.get_param('/extraction/range_threshold',1.0)
        self.radius_max_th = rospy.get_param('/extraction/radius_max_th',0.3)
        self.landMark_min_pt = rospy.get_param('"/extraction/landMark_min_pt',2)

    def process(self,msg,trust = False):
        labels = []
        # TODO
        landmarks_ = self.extractLandMark(laser_extr,labels,trust)
        return landmarks_

    def extractLandMark(self,msg,labels,trust):
        # TODO
        return landMarks
