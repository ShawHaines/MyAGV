#!/usr/bin/env python
import rospy
import tf
# refactoring from LocalPlanner
from local_planner import LocalPlanner

# turns out you still need to import these..
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import Header
import numpy as np
import math
import time


class Tracking(LocalPlanner):
    # arguments of coefficients that need to be adjusted
    # graceful control strategy version.
    k1=0.8
    k2=3.0
    vMax=0.20
    mu=0.03
    order=1.50  # the order of v selector
    def __init__(self):
        super(Tracking,self).__init__()
    def planOnce(self):
        self.lock.acquire()

        self.updateGlobalPose()
                
        target = self.path.poses[self.goal_index].pose.position
        
        dx = target.x - self.x
        dy = target.y - self.y
        
        # see the definition of alpha,beta,rho from the courseware. 
        # Notice that alpha is of the opposite sign.
        beta = math.atan2(dy, dx)
        alpha= beta-self.yaw
        rho=np.linalg.norm([dx,dy])

        # bloody sign problem!
        kappa=1/rho*(self.k2*(alpha+math.atan(self.k1*beta))+math.sin(alpha)*(1+self.k1/(1+(self.k1*beta)**2)))
        print("kappa={}".format(kappa))
        self.vx=self.vMax/(1+self.mu*abs(kappa)**self.order)
        self.vw=self.vx*kappa
        print('\tvx={}'.format(self.vx))
        print('\tvw={}'.format(self.vw))

        self.publishVel()

        self.lock.release()
        pass
    
def main():
    rospy.init_node('path_tracking')
    t = Tracking()
    rospy.spin()

if __name__ == '__main__':
    main()
