#!/usr/bin/env python
#coding:utf-8
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Point,PointStamped,Quaternion
from std_msgs.msg import Header
from course_agv_nav.srv import Plan,PlanResponse

# import matplotlib as mpl
# mpl.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
# import cv2
import time
import heapq

# order: counter-clockwise, from x axis
directions=np.array([(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)],dtype=int)

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
    # newShape=[x+y for x,y in zip(src.shape,kernel.shape)]
    newShape=np.add(src.shape,kernel.shape)
    tempMap=np.zeros(shape=tuple(newShape),dtype=np.int8)
    tempMap[kernel.shape[0]//2:kernel.shape[0]//2+src.shape[0],
            kernel.shape[1]//2:kernel.shape[1]//2+src.shape[1]]=src
    result=np.zeros(shape=src.shape)
    for i in range(src.shape[0]):
        for j in range(src.shape[1]):
            result[i,j]=np.max(tempMap[i:i+kernel.shape[0],j:j+kernel.shape[1]]&kernel)
    return result

def distance(a,b):
    "normal distance."
    return np.linalg.norm(np.array(a)-np.array(b))

class cell:
    lastPath=()
    cost=-1.0
    # the rest of length to get goal 
    rest=-1.0
    known=False
    def __init__(self):
        self.known=False

class GlobalPlanner:
    pathPublisher=None
    globalPlannerService=None
    currentPose=None
    goal=None
    goalSubscriber=None
    tfListener=None
    mapGrid=[]
    mapInfo=None
    cells=[]
    path=None

    def __init__(self):
        # TODO :
        #   tf listener : 
        #   publisher   : /course_agv/global_path
        #   subscriber  : /course_agv/goal
        #   service     : /course_agv/global_plan
        #   initialize for other variable
        
        #example
        
        self.pathPublisher = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)
        self.tfListener=tf.TransformListener() # plenty usage guides
        self.loadMap()
        self.globalPlannerService=rospy.Service("/course_agv/global_plan",Plan,self.updatePlan,buff_size=1)
        self.goalSubscriber=rospy.Subscriber('/course_agv/goal',PoseStamped,callback=self.onUpdateGoal)
        
        self.path=Path(header=Header(0,rospy.Time.now(),"map"))
        pass

    def loadMap(self):

        ExpandRadius=0.4
        rospy.wait_for_service('/static_map')        
        try:
            # very compact use, equal to the effect of the commented ones.
            data=rospy.ServiceProxy("/static_map",GetMap)().map
            # getMapHandle=rospy.ServiceProxy("/static_map",GetMap)
            # data=getMapHandle().map
            
        except rospy.ServiceException,e:
            rospy.loginfo("Map service call failed: {}".format(e))

        # print(data)
        self.mapGrid=[0 if x==0 else 1 for x in data.data]
        # print(self.mapGrid)
        self.mapInfo=data.info
        self.mapGrid=np.reshape(self.mapGrid,
                                (self.mapInfo.height,self.mapInfo.width))
        
        # plt can only work in main thread...
        # print(self.mapGrid)
        # plt.pcolormesh(self.mapGrid)
        # plt.show()
        # plt.savefig("mapGrid_src.png")

        cellCount=ExpandRadius*2//self.mapInfo.resolution+1
        kernel=np.ones((cellCount,cellCount),int)
        self.mapGrid=dilate(self.mapGrid,kernel)

        # plt.pcolormesh(self.mapGrid)
        # plt.savefig("expanded.png")
        # plt.show()
        print("finished.")
        # useless openCV!
        # self.mapGrid=cv2.dilate(self.mapGrid,kernel)
        # cv2.imshow('result',self.mapGrid)
        
        for i in range(self.mapInfo.width):
            for j in range(self.mapInfo.height):
                self.cells.append(cell())
        self.cells=np.reshape(self.cells,(self.mapInfo.height,self.mapInfo.width))

        return
                
    def onUpdateGoal(self,goal):
        # update Goal
        self.goal = self.tfListener.transformPose("map", goal)
        # rospy.loginfo(self.goal.pose.position)

        # call for planning service
        rospy.wait_for_service("/course_agv/global_plan")
        try:
            plannerService=rospy.ServiceProxy("/course_agv/global_plan",Plan)
            flag=plannerService()
            print(flag)
        except rospy.ServiceException, e:
            print("global Planning service failed: {}".format(e))
        
    def AStarSearch(self,now,destination):
        for i in range(self.mapInfo.width):
            for j in range(self.mapInfo.height):
                self.cells[i,j].cost=-1.0
                self.cells[i,j].rest=-1.0
                self.cells[i,j].lastPath=()
        
        self.cells[now].cost=0.0
        self.cells[now].rest=distance(now,destination)
        # current is a tuple
        current=now
        heap=[]
        while not self.cells[destination].lastPath:
            for dr in directions:
                temp=tuple(np.array(current)+dr)
                if (not self.mapGrid[temp]) and (self.cells[temp].cost<0):
                    self.cells[temp].cost=self.cells[current].cost+np.linalg.norm(dr)
                    self.cells[temp].rest=distance(current,destination)
                    self.cells[temp].lastPath=current
                    heapq.heappush(heap,(self.cells[temp].cost+self.cells[temp].rest, temp))

            # rospy.loginfo(heap)

            current=heapq.heappop(heap)[1]
        if self.cells[destination].lastPath:
            current=destination
            tempPath=[]
            # trace back
            while current!=now:
                tempPath.append(current)
                current=self.cells[current].lastPath
            tempPath.reverse()
            self.path.poses=[self.toPoseStamped(x) for x in tempPath]

            # rospy.loginfo(tempPath)
            # rospy.loginfo(self.path.poses)
            
            for i in range(len(tempPath)):
                self.path.poses[i].header.seq=i
                if i<len(tempPath)-1:
                    # calculate the inner product to find theta，
                    # it's really a clever way, instead of the diverging problem of arctan 
                    # and +- sign problem.
                    # Actually math.atan2 will take care of that...
                    
                    # you need to learn to use trListener.lookupTransform function
                    deltaR=np.array([self.path.poses[i+1].pose.position.x-self.path.poses[i].pose.position.x, self.path.poses[i+1].pose.position.y-self.path.poses[i].pose.position.y])

                    # rospy.loginfo(self.path.poses[i])
                    # rospy.loginfo(self.path.poses[i+1])
                    # rospy.loginfo(deltaR)

                    theta=np.arccos((deltaR*np.array([1,0])).sum()/np.linalg.norm(deltaR))

                    if (deltaR[1]<0):
                        theta=-theta
                    
                    # again the messy quaternion
                    quaternion=tf.transformations.quaternion_from_euler(0,0,theta)
                    self.path.poses[i].pose.orientation=Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
                else:
                    # deals with the goal point.
                    self.path.poses[i].pose=self.goal.pose
                    # quaternion=tf.transformations.quaternion_from_euler(0,0,0)
                    # self.path.poses[i].pose.orientation=Quaternion(x=quaternion[0],y=quaternion[1],z=quaternion[2],w=quaternion[3])
            self.path.header.seq+=1
            self.path.header.stamp=rospy.Time.now()
            # publish the path
            self.pathPublisher.publish(self.path)

            return True
        else:
            return False

    def updatePlan(self,request):
        # returns True if success. else return false.
        # Also, it publishes the plan to topic /course_agv/global_path        
        self.currentPose = PoseStamped()
        
        # subtract a duration of 0.03 second to ensure the transform.
        self.currentPose.header.__init__(seq=1,stamp=rospy.Time.now()-rospy.Duration(secs=0.03),frame_id="robot_base")
        self.currentPose.pose.position=Point(0,0,0)
        self.currentPose.pose.orientation=Quaternion(0,0,0,1)
        rospy.loginfo(self.currentPose)
        self.currentPose = self.tfListener.transformPose('map',self.currentPose)
        # rospy.loginfo(self.currentPoint.point)
        
        # transform into the nearest grid, default origin is on the center.
        # be aware that (x,y) is in the opposite order of (i,j)
        # and now, destination is in row-column order, just as self.mapGrid
        now=[]
        destination=[]
        now.append(int(self.currentPose.pose.position.y//self.mapInfo.resolution+self.mapInfo.height//2))
        now.append(int(self.currentPose.pose.position.x//self.mapInfo.resolution+self.mapInfo.width//2))
        destination.append(int(self.goal.pose.position.y//self.mapInfo.resolution+self.mapInfo.height//2))
        destination.append(int(self.goal.pose.position.x//self.mapInfo.resolution+self.mapInfo.width //2))
        # really messy...
        now=tuple(now)
        destination=tuple(destination)
        rospy.loginfo(now)
        rospy.loginfo(destination)
        
        return self.AStarSearch(now,destination)

    def toPoseStamped(self,pos):
        newPose=PoseStamped(header=Header(0,rospy.Time.now(),"map"))
        newPose.pose.position.z=0
        newPose.pose.position.x=(pos[1]-self.mapInfo.height//2)*self.mapInfo.resolution
        newPose.pose.position.y=(pos[0]-self.mapInfo.width //2)*self.mapInfo.resolution
        return newPose
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
    # plt.pcolormesh(gp.mapGrid)
    # plt.show()
    # gp.test()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()
