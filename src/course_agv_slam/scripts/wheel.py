#!/usr/bin/env python
# coding:utf-8
import rospy
import tf
from sensor_msgs.msg import LaserScan,JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped,Pose,PoseStamped,Point,Quaternion
from std_msgs.msg import Header
from threading import Lock,Thread
import numpy as np
import math
import time
from course_agv_slam.srv import Odometry_srv,Odometry_srvResponse

def toList(q):
    return [q.x,q.y,q.z,q.w]
def toQuaternion(l):
    return Quaternion(l[0],l[1],l[2],l[3])

class WheelOdometry(object):
    deltaT=0.25
    WheelRadius=0.08
    Width=0.227
    def __init__(self):
        self.wheelSubscriber=rospy.Subscriber("/course_agv/joint_states",JointState,callback=self.updateJointState,queue_size=1)
        self.lock=Lock()
        self.leftPosition=self.rightPosition=None
        # Odometry estimated by wheel odometry.
        self.odometry=Odometry(header=Header(0,rospy.Time(0),"world_base"))
        self.odometry.pose.pose=Pose(position=Point(0,0,0),orientation=Quaternion(0,0,0,1))
        
        self.odometryPublisher=rospy.Publisher("wheel_odom",Odometry,queue_size=1)
        self.service=rospy.Service("/course_agv/odometry",Odometry_srv,self.respondOdometry,buff_size=1)
        self.jointstates=None
        self.working=True
        self.loop()
        
    def updateJointState(self,data):
        self.lock.acquire()
        if not self.leftPosition:
            self.leftPosition =data.position[0]
            self.rightPosition=data.position[1]
        self.jointstates=data
        self.lock.release()
        return
    def publishOdometry(self):
        self.odometryPublisher.publish(self.odometry)
        return
    def positioning(self):
        if not self.jointstates:
            return False
        self.lock.acquire()
        # the distance covered by each wheel
        sl=(self.jointstates.position[0]-self.leftPosition) *self.WheelRadius
        sr=(self.jointstates.position[1]-self.rightPosition)*self.WheelRadius
        self.leftPosition =self.jointstates.position[0]
        self.rightPosition=self.jointstates.position[1]
        dtheta=(sr-sl)/self.Width
        dl=(sl+sr)/2
        epsilon=1e-10
        if abs(dtheta)>epsilon:
            rho=dl/dtheta
            dx=rho*math.sin(dtheta)
            dy=rho*(1-math.cos(dtheta))
        else:
            dx=dl
            dy=0
        # update pose
        # euler is much more convenient than quaternions.
        euler=list(tf.transformations.euler_from_quaternion(toList(self.odometry.pose.pose.orientation)))
        displace=np.array([dx,dy,0,1])
        displace=np.dot(tf.transformations.euler_matrix(euler[0],euler[1],euler[2]),displace)
        print("dx {} dy {} dtheta {}".format(dx,dy,dtheta))
        self.odometry.pose.pose.position.x+=displace[0]
        self.odometry.pose.pose.position.y+=displace[1]
        euler[2]+=dtheta
        self.odometry.pose.pose.orientation=toQuaternion(tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2]))
        print("position {}".format(self.odometry.pose.pose.position))
        print("orientation {}".format(euler))
        self.odometry.header.stamp=self.jointstates.header.stamp
        self.odometry.header.seq  =self.jointstates.header.seq
        self.lock.release()
        return True
    def respondOdometry(self,request):
        return self.odometry.pose.pose
    def loop(self):
        while self.working:
            success=self.positioning()
            if success:
                self.publishOdometry()
            time.sleep(self.deltaT)
            
        print("exit looping...")
        return
            
def main():
    rospy.init_node('wheel_odometry')
    wheel= WheelOdometry()
    rospy.spin()

if __name__ == "__main__":
    main() 