#!/usr/bin/env python
#coding:utf-8
import rospy
import tf

from nav_msgs.srv import GetMap
from nav_msgs.msg import Path,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Point,PointStamped,Quaternion,Pose
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

import numpy as np
import time

from local_planner import LocalPlanner

class DWAPlanner(LocalPlanner):
    """ TODO:
        dwa()
        predictTrajectory()
        cost()
        getDynamicWindow()
    """
    # arguments
    v_max=0.3
    w_max=2.0
    v_step=0.001
    w_step=0.01
    # unit: cells per second^2
    a_max=0.04
    beta_max=0.5
    
    
    def __init__(self):
        super(DWAPlanner,self).__init__()
        self.deltaT=self.sleepTime
        self.laserListener=rospy.Subscriber("/course_agv/laser/scan",LaserScan,callback=updateLaser)
        self.laserInfo=LaserScan()

    def planOnce(self):
        self.vx,self.vw=self.dwa()
        self.publishVel()
        return
        
    def dwa(self):
        window=self.getDynamicWindow()
        bestPair=None
        minCost=-1
        for pair in window:
            # self.predictTrajectory(pair)
            cost=self.cost(pair)
            if cost<minCost or minCost<0:
                minCost=cost
                bestPair=pair
        return bestPair

    def cost(self,pair):
        goalCost=dist(me,goal)*k_goal
        speedCost=(self.v_max-abs(self.vx))*k_speed
        obstacleCost=self.nearestObstacle()*k_obstacle
        return goalCost+speedCost+obstacleCost

    def getDynamicWindow(self):
        # I think it's not necessary to use a lock here..
        # filter by maxium accelaration
        window=[]
        for vw in np.arange(self.vw-self.beta_max*self.deltaT,self.vw-self.beta_max*self.deltaT,self.w_step):
            for vx in np.arange(self.vx-self.a_max*self.deltaT,self.vx+self.a_max*self.deltaT,self.v_step):
                #filter by maxium speed
                if abs(vx)>self.vx or abs(vw)>self.w_max:
                    continue
                #filter by obstacle prediction
                obstacleDist=self.nearestObstacle(self.predictTrajectory((vx,vw)))
                # not consider the situation of near miss. 
                if 2*obstacleDist*self.a_max<self.vx**2:
                    if self.vx>0:
                        # pruning
                        break
                    else:
                        continue
                print("vx:{}\tvw:{}\tdist:{}".format(vx,vw,obstacleDist))
                window.append((vx,vw,obstacleDist))
        return window     

    def nearestObstacle(self,ps):
        # ps: PoseStamped
        theta=np.arange(self.laserInfo.angle_min,self.laserInfo.angle_min,self.laserInfo.angle_increment)              
        x=np.cos(theta)
        y=np.sin(theta)
        now=np.array(ps.pose.position.x,ps.pose.position.y)
        dist=[np.linalg.norm(now-np.array(pos)) for pos in zip (x,y)]
        return np.min(dist)

    def predictTrajectory(self,pair):
        # return a Posestamped
        # dtheta is positive in counterclockwise.
        dtheta=pair[1]*self.deltaT
        rho=pair[0]/pair[1]
        dx=rho*np.sin(dtheta)
        dy=rho*(1-np.cos(dtheta))
        newHeader=Header(0,rospy.Time.now(),"robot_base")
        newPose=Pose(position=Point(dx,dy,0),orientation=(1,0,0,0))
        newPoseStamped=PoseStamped(header=newHeader,pose=newPose)
        # probably buggy
        print("prediction:")
        print(newPoseStamped)
        return newPoseStamped

    def updateLaser(self,data):
        # TODO: identify and remove junk laser data (those that hit the ground by accident.)
        self.laserInfo=data
        return

def main():
    rospy.init_node('dwa_planner',anonymous=False)
    dwaAgent=DWAPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
