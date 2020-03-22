#!/usr/bin/env python
import rospy
import sys
from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse

def add_two_ints_client(x,y):
    rospy.wait_for_service("add_two_ints")
    try:
        handleAdd=rospy.ServiceProxy("add_two_ints",AddTwoInts)
        response=handleAdd(x,y)
        return response.sum
    except rospy.ServiceException, e:
        print("Service call failed: {}".format(e))

def usage():
    return("correct usage:\n{} x y".format(sys.argv[0]))


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x=int(sys.argv[1])
        y=int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting [{} + {}]".format(x,y))
    print("{} + {} = {}".format(x,y,add_two_ints_client(x,y)))