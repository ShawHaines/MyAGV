#!/usr/bin/env python
import rospy
import gazebo_msgs.msg
import tf
# import roslib # not necessary since we don't have dependencies.


def onRecievePose(data):
    broadcaster=tf.TransformBroadcaster()
    # rospy.loginfo(data)
    position=data.pose[1].position
    orientation=data.pose[1].orientation
    # print("my_position: ",position)
    broadcaster.sendTransform((position.x,position.y,position.z),
                            (orientation.x,orientation.y,orientation.z,orientation.w),
                            rospy.Time.now(),
                            "robot_base","map")

if __name__ == "__main__":
    rospy.init_node("tf_subscriber",anonymous=True)
    rospy.Subscriber("/gazebo/model_states",gazebo_msgs.msg.ModelStates,callback=onRecievePose)
    rospy.spin()