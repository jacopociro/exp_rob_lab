#! /usr/bin/env python

import rospy
import smach
import random
import time
import smach_ros
from exp_rob_lab.msg import Hint
from exp_rob_lab.srv import *

resp_hyp = HypothesisResponse
var = Hint

def room():
    return random.choice(['Kitchen', 'Ballroom', 'Conservatory', 'Dining_Room', 'Biliard_Room', 'Library', 'Lounge', 'Hall', 'Study'])

def callback(msg):
    global var
    #rospy.loginfo(msg)
    var = msg
    return var

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['clues'])
        
    def execute(self, userdata):
        place = room()
        rospy.loginfo('Moving to %s' %place)
        time.sleep(2)

        return 'clues'

class Clues(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'hypothesis'])
        

    def execute(self, userdata):
        global resp_hyp, var
        rospy.loginfo("Looking for clues...")
        
        #subscribes to hint_publisher, publishes to hypothesis_maker
        sub = rospy.Subscriber('hint', Hint, callback)
        rospy.wait_for_message('hint', Hint)

        rospy.wait_for_service('hypothesis_maker')
        try:
            hyp = rospy.ServiceProxy('hypothesis_maker', Hypothesis)
            
            id = var.id
            name = var.name
            class_id = var.class_id
          
            request = HypothesisRequest(id, name, class_id)
            resp_hyp = hyp(id, name, class_id)
            print('Hint collected:')
            print (resp_hyp)
            if resp_hyp.consistent == False:
                return 'move'
            else:
                return 'hypothesis'

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


        

        

class Hyp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'stop'])

    def execute(self, userdata):
        global resp_hyp
        
        rospy.loginfo('Formulating an hypothesis')
        rospy.loginfo('Moving to terminal')
        time.sleep(2)

        #subscribes to hypothesis maker and sends to oracle to check

        rospy.wait_for_service('oracle')
        try: 
            Oracle = rospy.ServiceProxy('oracle', oracle)
            oracle_res = Oracle(resp_hyp.id)
            if oracle_res.right == 0:
                print("Yay! I got the answer right!")
                return 'stop'
            else: 
                print("Mmh... I need more hints...")
                return 'move'
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
def main():
    
    rospy.init_node('state_machine')

    sm = smach.StateMachine(outcomes=['stop'])

    with sm:
        smach.StateMachine.add('Move', Move(), transitions={'clues':'Clues'})
        smach.StateMachine.add('Clues', Clues(), transitions={'move':'Move', 'hypothesis':'Hyp'})
        smach.StateMachine.add('Hyp', Hyp(), transitions={'move':'Move', 'stop':'stop'})

    sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()