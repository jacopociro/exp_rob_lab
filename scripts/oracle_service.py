##! /usr/bin/env python2

from exp_rob_lab.srv import oracle, oracleResponse
import rospy

def oracle_handle(req):
    if req.id == 3:
        print("Hypothesis is correct!")
        return oracleResponse(0)
    else:
        print("Try again!")

def oracle_server():
    rospy.init_node('oracle')
    s = rospy.Service('oracle', oracle, oracle_handle)
    print("Oracle is ready to tell you the truth!")
    rospy.spin()

if __name__ == '__main__':
    try:
        oracle_server()
    except rospy.ROSInterruptException: pass