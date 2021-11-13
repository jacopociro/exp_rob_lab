#! /usr/bin/env python2

from exp_rob_lab.srv import Hypothesis, HypothesisResponse
import rospy

def make_hypothesis(req):
    
    print(req)
    
    return HypothesisResponse(req)

def hypothesis_maker_server():

    rospy.init_node('server')

    
    s = rospy.Service('hypothesis_maker', Hypothesis, make_hypothesis)
    
    
    print(s)
    rospy.spin()

if __name__ == '__main__':
    try:
        hypothesis_maker_server()
    except rospy.ROSInterruptException: pass