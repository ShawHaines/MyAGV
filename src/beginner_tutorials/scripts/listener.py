#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("listener",anonymous=True)
    rospy.Subscriber("foobar",String,callback)
    rospy.spin()


# seems the position of the callback() is not a matter...
def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard "+data.data)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

