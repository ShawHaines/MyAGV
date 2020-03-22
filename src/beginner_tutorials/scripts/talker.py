#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node("talker",anonymous=True)
    rate=rospy.Rate(10)
    publisher=rospy.Publisher("foobar",String,queue_size=10)
    while not rospy.is_shutdown():
        message="Hello world! The time is {}s.".format(rospy.get_time())
        rospy.loginfo(message)
        publisher.publish(message)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass