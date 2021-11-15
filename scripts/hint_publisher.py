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
import random

hint = [['HP1','Miss_Scarlett', 'who'],['HP1','Candlestick', 'what'],['HP1','Kitchen', 'where'],
        ['HP2','Rev_Green', 'who'],['HP2','Dagger', 'what'],['HP2','Ballroom', 'where'],
        ['HP3','Col_Mustard', 'who'],['HP3','Lead_Pipe', 'what'],['HP3','Conservatory', 'where'],
        ['HP4','Prof_Plum', 'who'],['HP4','Revolver', 'what'],['HP4','Biliard_Room', 'where'],
        ['HP5','Mrs_Peacock', 'who'],['HP5','Rope', 'what'],['HP5','Library', 'where'],
        ['HP6','Mrs_White', 'who'],['HP6','Wrench', 'what'],['HP6','Lounge', 'where']]

def publisher():
##
# \brief this function initilizes the node and the publisher. it also defines the message we want to send.
    pub = rospy.Publisher('hint', Hint)
    rospy.init_node('hint_publisher')
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        num = random.randint(0,17)
    
        msg = Hint()
        msg.id = hint[num][0]
        msg.name = hint[num][1]
        msg.class_id = hint[num][2]
        
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException: pass