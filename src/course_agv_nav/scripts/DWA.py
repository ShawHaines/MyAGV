#!/usr/bin/env python
#coding:utf-8
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Point,PointStamped,Quaternion
from std_msgs.msg import Header
from course_agv_nav.srv import Plan,PlanResponse
import numpy as np
import time

from local_planner import LocalPlanner

class DWAPlanner(LocalPlanner):

    def __init__(self):
        pass

    def dwa(self):
        window=self.getDynamicWindow()
        bestPair=None
        minCost=-1
        for pair in window:
            self.predictTrajectory()
            cost=self.cost(pair)
            if cost<minCost or minCost<0:
                minCost=cost
                bestPair=pair
        return bestPair

    def cost(self,pair):
        goalCost=dist(me,goal)*k_goal
        speedCost=(v_max-abs(self.v))*k_speed
        obstacleCost=self.nearestObstacle()*k_obstacle
        return goalCost+speedCost+obstacleCost

    def getDynamicWindow(self):
        pass
    

def main():
    rospy.init_node('local_planner',anonymous=False)
    lp = LocalPlanner()
    time.sleep(0.5)
    # gp.test()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
