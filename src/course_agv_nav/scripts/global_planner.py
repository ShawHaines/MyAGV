#!/usr/bin/env python
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from course_agv_nav.srv import Plan,PlanResponse

import time

class GlobalPlanner:
    def __init__(self):
        # TODO  : 
        #   tf listener : 
        #   publisher   : /course_agv/global_path
        #   subscriver  : /course_agv/goal
        #   service     : /course_agv/global_plan
        #   initialize for other variable
        
        #example
        self.path_pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)

        pass
    def test(self):
        rx = [0,-1,-2,-3]
        ry = [0,1,0,1]
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(rx)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = rx[i]
            pose.pose.position.y = ry[i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)

def main():
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    time.sleep(0.5)
    gp.test()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
