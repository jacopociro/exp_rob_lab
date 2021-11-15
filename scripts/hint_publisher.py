#! /usr/bin/env python2

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
    pub = rospy.Publisher('hint', Hint)
    rospy.init_node('hint_publisher')
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        #ricordati di cambiare il randint per includere tutti gli int
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