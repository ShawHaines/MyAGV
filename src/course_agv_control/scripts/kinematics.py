#!/usr/bin/env python

import geometry_msgs.msg
import std_msgs.msg
import rospy

# define the scaling
WheelRadius=0.08
Width=0.227

class Transmitter:

    # private member
    __leftVelocity=0
    __rightVelocity=0
    def onRecieve(self,data):
        # calculate the velocity of each wheel and publish.

        # rospy.loginfo(rospy.get_caller_id())
        velocity=data.linear.x
        omega=data.angular.z

        # rospy.loginfo("  v={:3} w={:3}".format(velocity,omega))
        # change to print message, in order to save some logging space.
        print("v={:3} w={:3}".format(velocity,omega))


        # first I'll guess the velocity command is for v...
        self.__leftVelocity=(velocity-omega*Width/2)/WheelRadius
        self.__rightVelocity=(velocity+omega*Width/2)/WheelRadius
        
        # rospy.loginfo(" Vr={:3} Vl={:3}".format(self.__rightVelocity,self.__leftVelocity))
        print("\tWr={:3} Wl={:3}".format(self.__rightVelocity,self.__leftVelocity))

        self.rightPublisher.publish(self.__rightVelocity)
        self.leftPublisher.publish(self.__leftVelocity)
        
    def __init__(self):
        self.subscriber=rospy.Subscriber('/course_agv/velocity',
            geometry_msgs.msg.Twist,callback=self.onRecieve)
        self.leftPublisher=rospy.Publisher("/course_agv/left_wheel_velocity_controller/command",std_msgs.msg.Float64,queue_size=1)
        self.rightPublisher=rospy.Publisher("/course_agv/right_wheel_velocity_controller/command",std_msgs.msg.Float64,queue_size=1)
        
def main():
    nodeName=rospy.init_node("transmitter",anonymous=True)
    print("Node Name: {}".format(nodeName))
    transmitter=Transmitter()
    rospy.spin()
    

if __name__ == "__main__":
    main()