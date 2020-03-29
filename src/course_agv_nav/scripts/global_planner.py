#!/usr/bin/env python
#coding:utf-8
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Point,PointStamped
from std_msgs.msg import Header
# from course_agv_nav.srv import Plan,PlanResponse

# import matplotlib as mpl
# mpl.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
# import cv2
import time
import heapq

def dilate(src, kernel):
    """
    dilate bin image src
    Args:
        bin_image: image with 0,1 pixel value
    Returns:
        dilated image
    Beautifully implemented
    """

    # print("kernel:",type(kernel))
    src = np.array(src)
    # print("src:",type(src))

    # if (kernel_size%2 == 0) or kernel_size<1:
    #     raise ValueError("kernel size must be odd and bigger than 1")
    # if (bin_image.max() != 1) or (bin_image.min() != 0):
    #     raise ValueError("input image's pixel value must be 0 or 1")
    newShape=[x+y for x,y in zip(src.shape,kernel.shape)]
    tempMap=np.zeros(shape=tuple(newShape),dtype=np.int8);
    tempMap[kernel.shape[0]//2:kernel.shape[0]//2+src.shape[0],
            kernel.shape[1]//2:kernel.shape[1]//2+src.shape[1]]=src
    result=np.zeros(shape=src.shape)
    for i in range(src.shape[0]):
        for j in range(src.shape[1]):
            result[i,j]=np.max(tempMap[i:i+kernel.shape[0],j:j+kernel.shape[1]]&kernel)
    return result

class MyPoint:
    def __init__(self,x=0,y=0):
        self.x=int(x)
        self.y=int(y)
        return

class GlobalPlanner:
    pathPublisher=None
    currentPose=None
    goal=None
    goalSubscriber=None
    tfListener=None
    mapGrid=[]
    mapInfo=None
    def loadMap(self,data):
        # unit: meter, it's actually diameter...
        ExpandRadius=0.4

        self.mapGrid=[0 if x==0 else 1 for x in data.data]
        # print(self.mapGrid)
        self.mapInfo=data.info
        self.mapGrid=np.reshape(self.mapGrid,
                                (self.mapInfo.height,self.mapInfo.width))
        
        # print(self.mapGrid)
        # plt.pcolormesh(self.mapGrid)
        # plt.savefig("mapGrid_src.png")
        cellCount=ExpandRadius//self.mapInfo.resolution+1
        kernel=np.ones((cellCount,cellCount),int)
        self.mapGrid=dilate(self.mapGrid,kernel)
        # plt.pcolormesh(self.mapGrid)
        # plt.savefig("expanded.png")
        # print("finished.")
        # useless openCV!
        # self.mapGrid=cv2.dilate(self.mapGrid,kernel)
        # cv2.imshow('result',self.mapGrid)
        
        # I can only write this in the callback function to ensure it's called after the map is loaded...
        self.goalSubscriber=rospy.Subscriber('/course_agv/goal',PoseStamped,callback=self.onUpdateGoal)
        return
                
    def onUpdateGoal(self,goal):
        self.goal = self.tfListener.transformPose("map", goal)
        # rospy.loginfo(self.goal.pose.position)
        self.currentPose = PoseStamped()
        
        # subtract a duration of 0.03 second to ensure the transform.
        self.currentPose.header.__init__(seq=1,stamp=rospy.Time.now()-rospy.Duration(secs=0.03),frame_id="robot_base")
        self.currentPose.pose.position=Point(0,0,0)
        self.currentPose.pose.orientation=tf.transformations.quaternion_from_euler(0,0,1)
        # rospy.loginfo(self.currentPoint)
        self.currentPose = self.tfListener.transformPoint('map',self.currentPose)
        # rospy.loginfo(self.currentPoint.point)

        # transform into the nearest grid, default origin is on the center.
        now=MyPoint()
        destination=MyPoint()
        now.x = self.currentPose.pose.position.x//self.mapInfo.resolution+self.mapInfo.width//2
        now.y = self.currentPose.pose.position.y//self.mapInfo.resolution+self.mapInfo.height//2
        destination.x=self.goal.pose.position.x//self.mapInfo.resolution+self.mapInfo.width //2
        destination.y=self.goal.pose.position.y//self.mapInfo.resolution+self.mapInfo.height//2
        
        rospy.loginfo((now.x,now.y))
        rospy.loginfo((destination.x,destination.y))
        
        self.AStarSearch(now,destination)
        
    def AStarSearch(self,now,destination):
        if now.x==destination.x and now.y==destination.y:
            self.path

    def __init__(self):
        # TODO :
        #   tf listener : 
        #   publisher   : /course_agv/global_path
        #   subscriber  : /course_agv/goal
        #   service     : /course_agv/global_plan
        #   initialize for other variable
        
        #example
        
        # self.pathPublisher = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)
        
        self.tfListener=tf.TransformListener() # plenty usage guides
        self.mapSubscriber=rospy.Subscriber('/map',OccupancyGrid,callback=self.loadMap,queue_size=1)


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
        self.pathPublisher.publish(path)

def main():
    rospy.init_node('global_planner',anonymous=False)
    gp = GlobalPlanner()
    time.sleep(0.5)
    # gp.test()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
