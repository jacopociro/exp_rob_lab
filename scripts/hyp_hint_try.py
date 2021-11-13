#! /usr/bin/env python

import rospy
from rospy.client import wait_for_message
import smach
import random
import time
import smach_ros
from exp_rob_lab.msg import Hint
from exp_rob_lab.srv import *
from exp_rob_lab.srv import oracle, oracleResponse
sub = None
culo = Hint

def callback(msg):
    global culo
    print(msg)
    culo = msg
    rospy.loginfo(msg)
    return culo


def main():
    global sub, culo
    rospy.init_node('hyp_hint_try')
    
    #subscribes to hint_publisher, publishes to hypothesis_maker
  
    sub = rospy.Subscriber("hint", Hint, callback)

    wait_for_message('hint', Hint)
    print (culo)
    print (sub)
    rospy.wait_for_service('hypothesis_maker')
    try:
        hypothesis = rospy.ServiceProxy('hypothesis_maker', Hypothesis)
        
        id = culo.id
        name = culo.name
        class_id = culo.class_id
        
        
        request = HypothesisRequest(id, name, class_id)
        print(request)
        response = hypothesis(id, name, class_id)
        print (response)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.spin()
    



if __name__ == '__main__':
    main()