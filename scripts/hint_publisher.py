#! /usr/bin/env python2


## @package expr_rob_lab
#
#  \file hint_publisher.py
#  \brief script for the hint publisher.
#
#  \author Jacopo Ciro Soncini
#  \version 1.0
#  \date 29/08/2022
#  \details
#  
#  Subscribes to: <BR>
#       None
#
#  Publishes to: <BR>
#	    /complete
#       /consistent
#       /marker_publisher/detected_id 
#
#  Services: <BR>
#       /hypothesis_maker
#       /oracle_hint
#
#  Client Services: <BR>
#       None
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#       This node handles the hint publisher, by publishing the hint from the aruco service.


import rospy
from exp_rob_lab.msg import Hint
from exp_rob_lab.srv import *
import random
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from diagnostic_msgs.msg import KeyValue
from exp_assignment3.srv import Marker, MarkerResponse
from std_msgs.msg import String, Int32, Bool
import time
import random
foundClues=[]

##
# \brief This function is the callback for the aruco subscriber   
# \param: the message receive from the subscriber, Int32
# \return: None
# This function is the callback for the aruco subscriber. It reads the variable, check if the clue is already saved and if not it calls the hypothesis_maker service
# and saves the clue in the global variable foundClues (array di clues). Also it publishes the clue on the /consistent and /complete publishers.

def Id(msg):
    global foundClues
    clue = msg.data
    check = True
    for x in foundClues:
        if x == clue:
            check = False
    if check:
        foundClues.append(clue)
        if clue > 10 and clue < 41:
            ht = orcl_srv(clue)
            print (ht)
            id = ht.oracle_hint.ID
            class_id = ht.oracle_hint.key
            name = ht.oracle_hint.value
            request = HypothesisRequest()
            request.id = str(id)
            request.class_id = str(class_id)
            request.name = str(name)
            print (request)
            resp_hyp = hyp(request)
            print("consistent?")
            print(resp_hyp)
            pub.publish(resp_hyp.consistent)
            if resp_hyp.consistent:
                pub2.publish(resp_hyp.id)

##
# \brief this function initilizes the node, the services and the publishers.    
# \param: None
# \return: None

def publisher():

    global hyp, orcl_srv, pub, pub2
    rospy.init_node('hint_publisher')
    hyp = rospy.ServiceProxy('/hypothesis_maker', Hypothesis)
    rospy.wait_for_service('/hypothesis_maker')
    orcl_srv = rospy.ServiceProxy('/oracle_hint', Marker)
    rospy.wait_for_service('/oracle_hint')
    rospy.Subscriber("/marker_publisher/detected_id", Int32, Id)
    pub = rospy.Publisher('/consistent', Bool, queue_size=1)   
    pub2 = rospy.Publisher('/complete', String, queue_size=1)
    rospy.spin()       

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException: pass