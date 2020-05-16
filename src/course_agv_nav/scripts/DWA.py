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
import math
import time

from local_planner import LocalPlanner

class DWAPlanner(LocalPlanner):
    # arguments
    safeDistance=0.4 # the lowest distance permitted.
    deltaT=0.5
    v_max=0.35
    w_max=0.6
    v_slice=5
    w_slice=10
   
    # cost arguments, all set to positive
    k_goal=1
    k_heading=3
    k_speed=0.1
    k_obstacle=1.2
    
    def __init__(self):
        super(DWAPlanner,self).__init__()
        self.laserListener=rospy.Subscriber("/course_agv/laser/scan",LaserScan,callback=self.updateLaser,queue_size=1)
        self.laserInfo=LaserScan()
        # self.sleepTime=self.deltaT
        self.sleepTime=0.25
        self.arrive_threshold=1.0
        # self.poseDebug=rospy.Publisher("/course_agv/poseDebug",PoseStamped,queue_size=0)

    def planOnce(self):
        self.lock.acquire()
        self.updateGlobalPose()
        self.vx,self.vw=self.dwa()
        self.publishVel()
        self.lock.release()
        return
        
    def dwa(self):
        window=self.getDynamicWindow()
        # print(window)
        # if window is empty
        if not window:
            return (self.vx,self.vw)
        cost=[self.cost(pair) for pair in window]
        bestPair=window[np.argmin(cost)]
        print("bestPair:{}".format(bestPair))
        # auxilary debug visualization
        # debugPose=self.tfListener.transformPose("map",self.predictTrajectory((bestPair[0],bestPair[1])))
        # self.poseDebug.publish(debugPose)
        # return bestPair
        return (bestPair[0],bestPair[1])

    def cost(self,pair):
        vx,vw,obstacleDist=pair
        me=self.predictTrajectory((vx,vw))
        goal=self.path.poses[self.goal_index]

        goalCost=self.distance(me,goal)*self.k_goal
        # permits backwards running
        headingCost=math.cos(self.headingAngle(me,goal))*np.sign(self.vx)*(-self.k_heading)
        speedCost=abs(vx)*-self.k_speed
        obstacleCost=self.potential(obstacleDist)*self.k_obstacle
        return goalCost+headingCost+speedCost+obstacleCost

    def potential(self,x):
        return x**(-0.6)-x

    def getDynamicWindow(self):
        # I think it's not necessary to use a lock here..
        # filter by maxium accelaration
        window=[]
        print("getting window...")
        # already filtered by maximum velocity and accelaration
        minVw=max(self.vw-self.beta_max*self.sleepTime,-self.w_max)
        maxVw=min(self.vw+self.beta_max*self.sleepTime,self.w_max)
        minVx=max(self.vx-self.a_max*self.sleepTime,-self.v_max)
        maxVx=min(self.vx+self.a_max*self.sleepTime,self.v_max)
        
        for vw in np.linspace(minVw,maxVw,self.w_slice):
            for vx in np.linspace(minVx,maxVx,self.v_slice):
                #filter by obstacle prediction
                obstacleDist=self.nearestObstacle(self.predictTrajectory((vx,vw)))
                # print("vx:{}\tvw:{}\tdist:{}".format(vx,vw,obstacleDist))

                # not consider the situation of near miss. 
                # FIXME: how can we avoid going directly through a point?
                if 2*obstacleDist*self.a_max<self.vx**2:
                    # not very convincing... rho WOULD change.
                    # if self.vx>0:
                    #     # pruning
                    #     break
                    # else:
                    #     continue
                    continue
                window.append((vx,vw,obstacleDist))
                
        return window

    def nearestObstacle(self,ps):
        # ps: PoseStamped

        # is there an easy way in tf to transform laserscan type? like pointcloud?
        theta=np.linspace(self.laserInfo.angle_min,self.laserInfo.angle_max,len(self.laserInfo.ranges))
        # good thing that hokuyo differs from base only in z coordinate. Otherwise it would have been much harder.
        x=np.cos(theta)*self.laserInfo.ranges
        y=np.sin(theta)*self.laserInfo.ranges
        now=np.array([ps.pose.position.x,ps.pose.position.y])
        dist=[np.linalg.norm(now-np.array(pos)) for pos in zip (x,y)]
        # print(dist)
        return np.min(dist)

    def predictTrajectory(self,pair):
        # return a Posestamped
        # dtheta is positive in counterclockwise.
        vx,vw=pair[0],pair[1]
        epsilon=1e-6
        if abs(vw)<epsilon:
            dx=vx*self.deltaT
            dy=0
            dtheta=0
        else:
            dtheta=vw*self.deltaT
            rho=vx/vw
            
            dx=rho*np.sin(dtheta) # under 4 combinations it is always correct
            # FIXME: tred carefully with sign problem!
            dy=rho*(1-np.cos(dtheta))
        # FIXME: DON'T use rospy.Time.now()! 
        # Time(0) is the latest stored moment, while Time.now() is NOW.
        # if you have to use Time.now(), make sure exception is 
        # catched and use it with waitForTransform().
        newHeader=Header(0,rospy.Time(0),"robot_base")
        quaternion=tf.transformations.quaternion_from_euler(0,0,dtheta)
        # messy interpretation
        newPose=Pose(position=Point(dx,dy,0),
                    orientation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        newPoseStamped=PoseStamped(header=newHeader,pose=newPose)
        # probably buggy
        # print("prediction:")
        # print(newPoseStamped)
        return newPoseStamped

    def updateLaser(self,data):
        # TODO: identify and remove junk laser data (those that hit the ground by accident.)
        # in order to avoid its change during DWA
        self.lock.acquire()
        # also requires queue_size to be 1, make sure it's latest
        self.laserInfo=data
        self.lock.release()
        return

    def distance(self,_psA,_psB):
        # convert the two PointStamped into the same frame, and calculate the distance.
        try:
            psA=self.tfListener.transformPose("map",_psA)
            psB=self.tfListener.transformPose("map",_psB)
        except (tf.LookupException,tf.ConnectivityException, tf.ExtrapolationException):
            print("get TF error!")
        pA=np.array([psA.pose.position.x,psA.pose.position.y,psA.pose.position.z])
        pB=np.array([psB.pose.position.x,psB.pose.position.y,psB.pose.position.z])        
        return np.linalg.norm(pA-pB)
    
    def headingAngle(self,me,goal):
        # convert the two PointStamped into the same frame, and calculate the angle.
        try:
            psA=self.tfListener.transformPose("map",me)
            psB=self.tfListener.transformPose("map",goal)
        except (tf.LookupException,tf.ConnectivityException, tf.ExtrapolationException):
            print("get TF error!")
        dx=psB.pose.position.x-psA.pose.position.x
        dy=psB.pose.position.y-psA.pose.position.y
        theta=math.atan2(dy,dx)
        euler=tf.transformations.euler_from_quaternion(
            [psA.pose.orientation.x,psA.pose.orientation.y,psA.pose.orientation.z,psA.pose.orientation.w])
        return euler[2]-theta

def main():
    rospy.init_node('dwa_planner',anonymous=False)
    dwaAgent=DWAPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
