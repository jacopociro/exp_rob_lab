#! /usr/bin/env python2


## @package expr_rob_lab
#
#  \file hint_publisher.py
#  \brief script for the hint publisher.
#
#  \author Jacopo Ciro Soncini
#  \version 1.0
#  \date 15/11/2021
#  \details
#  
#  Subscribes to: <BR>
#       None
#
#  Publishes to: <BR>
#	    hint
#
#  Services: <BR>
#       None
#
#  Client Services: <BR>
#       None
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#       This node handles the hint publisher, by publishing random hints from a list.


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
    

def publisher():
##
# \brief this function initilizes the node and the publisher. it also defines the message we want to send.
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