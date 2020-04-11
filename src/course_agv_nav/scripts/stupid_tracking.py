#!/usr/bin/env python
import rospy
#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import Header
from threading import Lock,Thread
import numpy as np
import math
import time
class Tracking:
    tracking_thread=None
    goal_index=0
    sleepTime=0.01
    arrive_threshold = 0.6
    vx = 0.0
    vw = 0.0
    # arguments of coefficients that need to be adjusted
    # linear control strategy version.
    k_rho=0.3 
    k_alpha=1.5
    k_beta=1.0
    
    def __init__(self):
        self.lock = Lock()
        
        self.path = Path(header=Header(0,rospy.Time.now(),"map"))
        self.tfListener = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
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

    def pathCallback(self,msg):
        print("get path msg!!!!!",msg)
        self.path = msg
        self.lock.acquire()
        self.initTracking()
        self.lock.release()
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
        while True:
            self.planOnce()
            time.sleep(self.sleepTime)
        print("exit track thread!!")
        self.lock.acquire()
        self.publishVel(True)
        self.lock.release()
        self.tracking_thread = None
        pass
    def planOnce(self):
        self.lock.acquire()

        self.updateGlobalPose()
        
        # if self.goal_index==len(self.path.poses)-1:


        target = self.path.poses[self.goal_index].pose.position

        dx = target.x - self.x
        dy = target.y - self.y
        # see the definition of alpha,beta,rho from the courseware. 
        # Notice that alpha is of the opposite sign.
        beta = math.atan2(dy, dx)
        alpha= beta-self.yaw
        rho=np.linalg.norm([dx,dy])

        self.vx = self.k_rho*rho
        self.vw = self.k_alpha*alpha+self.k_beta*beta
        # this threshold would only add to the instability. The angular velocity is not too sensitive.
        # if self.vw > 0.5:
        #     self.vw = 0.5
        # the limit should be 2 ways.
        # if self.vw > 0.2 or self.vw<-0.2:
        #     self.vx = 0

        self.publishVel()

        self.lock.release()
        pass
    def publishVel(self,zero = False):
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        if zero:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.vel_pub.publish(cmd)

def main():
    rospy.init_node('stupid_tracking')
    t = Tracking()
    rospy.spin()

if __name__ == '__main__':
    main()
