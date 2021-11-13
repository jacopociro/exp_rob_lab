#! /usr/bin/env python


import rospy
from exp_rob_lab.srv import *




def main():
    
    rospy.init_node('client')
    
    #subscribes to hint_publisher, publishes to hypothesis_maker
  
    
    
    #print (sub)
    rospy.wait_for_service('hypothesis_maker')
    try:
        hypothesis = rospy.ServiceProxy('hypothesis_maker', Hypothesis)
        #rospy.spin()
        #resp_hyp = hypothesis(sub.id, sub.name, sub.class_id)
        id = 'HP3'
        name = 'Jhon'
        class_id = 'who'
        
        request = HypothesisRequest(id, name, class_id)
        print(request)
        response = hypothesis(id, name, class_id)
        print (response)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)





if __name__ == '__main__':
    main()