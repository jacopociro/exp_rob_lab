##! /usr/bin/env python2

from exp_rob_lab.srv import oracle, oracleResponse
import rospy

def oracle_handle(req):
    print('Hypothesis number %s is...'%req.id)
    if req.id == 'HP3':
        print("correct!")
        right = 0
    else:
        print("wrong! Try again!")
        right = 1
    return oracleResponse(right)

def oracle_server():
    rospy.init_node('oracle')
    s = rospy.Service('oracle', oracle, oracle_handle)
    print("Oracle is ready to tell you the truth!")
    rospy.spin()

if __name__ == '__main__':
    try:
        oracle_server()
    except rospy.ROSInterruptException: pass