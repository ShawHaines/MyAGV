#!/usr/bin/env python
import rospy
#!/usr/bin/env python
import rospy
import tf

# refactored from LocalPlanner class
from local_planner import LocalPlanner

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import Header
from threading import Lock,Thread
import numpy as np
import math
import time

class LinearTracking(LocalPlanner):
    # arguments of coefficients that need carefully adjusted
    # linear control strategy version.
    k_rho=0.2 
    k_alpha=1.5
    k_beta=1.0
    
    def __init__(self):
        super(LinearTracking,self).__init__()
    def planOnce(self):
        self.lock.acquire()

        self.updateGlobalPose()
        
        # if self.goal_index==len(self.path.poses)-1:


        target = self.path.poses[self.goal_index].pose.position

        dx = target.x - self.x
        dy = target.y - self.y

        # see the definition of alpha,beta,rho in the courseware. 
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
def main():
    rospy.init_node('stupid_tracking')
    lt = LinearTracking()
    rospy.spin()

if __name__ == '__main__':
    main()
