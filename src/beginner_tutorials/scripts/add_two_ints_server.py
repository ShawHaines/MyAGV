#!/usr/bin/env python

from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy


def onAdd(request):
    print("Returning [{} + {} = {}]".format(request.a,request.b,(request.a+request.b)))
    return(AddTwoIntsResponse(request.a+request.b))

def add_two_ints_server():
    rospy.init_node("add_two_ints_server",anonymous=True)
    service=rospy.Service("add_two_ints",AddTwoInts,onAdd)
    print("Ready to add ...")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()