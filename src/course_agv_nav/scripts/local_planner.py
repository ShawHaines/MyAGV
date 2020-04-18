#!/usr/bin/env python
#coding:utf-8
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import Header
from threading import Lock,Thread
import numpy as np
import math
import time

class LocalPlanner(object):# the object Base is necessary
    tracking_thread=None
    goal_index=0
    sleepTime=0.017  # 60Hz
    arrive_threshold = 0.5
    terminate_threshold=0.15
    vx = 0.0
    vw = 0.0
     # unit: cells per second^2
    a_max=0.05
    beta_max=0.3
    
    def __init__(self):
        self.lock = Lock()
        
        self.path = Path(header=Header(0,rospy.Time.now(),"map"))
        self.tfListener = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback,queue_size=1)
        self.vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.isTracking=False
        pass
    def updateGlobalPose(self):
        # update the AGV's current pose and the progress of mid_goal tracking
        try:
            # learn how to use these methods!
            self.tfListener.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tfListener.lookupTransform('/map','/robot_base',rospy.Time(0))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        # concise grammar!
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw
        p = self.path.poses[self.goal_index].pose.position
        dis = math.hypot(p.x-self.x,p.y-self.y)
        if dis < self.arrive_threshold and self.goal_index < len(self.path.poses)-1:
            self.goal_index += 1
        self.midpose_pub.publish(self.path.poses[self.goal_index])
        if self.goal_index==len(self.path.poses)-1 and dis<self.terminate_threshold:
            self.isTracking=False
            return
        
    def pathCallback(self,msg):
        print("get path msg!!!!!",msg)
        self.path = msg
        self.lock.acquire()
        self.initTracking()
        self.lock.release()
        self.isTracking=True
        if not self.tracking_thread:
            self.tracking_thread = Thread(target=self.trackThreadFunc)
            self.tracking_thread.start()
        pass
    def initTracking(self):
        self.goal_index = 0
        self.updateGlobalPose()
        pass
    def trackThreadFunc(self):
        print("running track thread!!")
        # while self.plan_lastIndex > self.plan_target_ind:
        while self.isTracking:
            self.planOnce()
            time.sleep(self.sleepTime)
        print("exit track thread!!")
        self.lock.acquire()
        self.publishVel(zero=True)
        self.vx,self.vw=0,0
        self.lock.release()
        self.tracking_thread = None
        return
    def planOnce(self):
        # it's just a virtual function, overload it in its derived classes
        pass
    def publishVel(self,zero = False):
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        if zero:
            # improved to gentle brake
            while abs(self.vx)>self.a_max*self.sleepTime or abs(self.vw)>self.beta_max*self.sleepTime:
                if abs(self.vx)>self.a_max*self.sleepTime:
                    self.vx-=np.sign(self.vx)*self.a_max*self.sleepTime
                    cmd.linear.x  = self.vx
                if abs(self.vw)>self.beta_max*self.sleepTime:
                    self.vw-=np.sign(self.vw)*self.beta_max*self.sleepTime
                    cmd.angular.z = self.vw
                self.vel_pub.publish(cmd)
                time.sleep(self.sleepTime)
            cmd.linear.x =0
            cmd.angular.z=0
        self.vel_pub.publish(cmd)


def main():
    rospy.init_node('local_planner',anonymous=False)
    lp = LocalPlanner()
    # time.sleep(0.5)
    # gp.test()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
