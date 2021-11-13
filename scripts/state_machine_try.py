#! /usr/bin/env python


import rospy
import smach
import random
import time
import smach_ros
from exp_rob_lab.msg import Hint
from exp_rob_lab.srv import Hypothesis, HypothesisResponse
from exp_rob_lab.srv import oracle, oracleResponse

hypothesis = None
oracle_res = None
resp_hyp = None

def room():
    return random.choice(['Kitcher', 'Ballroom', 'Conservatory', 'Dining_Room', 'Biliard_Room', 'Library', 'Lounge', 'Hall', 'Study'])

def callback(msg):
    rospy.loginfo(msg)
    return msg

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
        global hypothesis, resp_hyp
        rospy.loginfo("Looking for clues...")
        #rospy.wait_for_service('hypothesis_maker')
        #subscribes to hint_publisher, publishes to hypothesis_maker
        #sub = rospy.Subscriber('hint', Hint, callback)
        #hypothesis = rospy.ServiceProxy('hypothesis_maker', Hypothesis)
        #resp_hyp = hypothesis(sub.id, sub.name, sub.class_id)
        #resp_hyp = 1
        resp_hyp = 2
        print (resp_hyp)
        
        if resp_hyp == 1:
            return 'move'
        else:
            return 'hypothesis'

class Hypothesis(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'stop'])

    def execute(self, userdata):
        global oracle_res, hypothesis, resp_hyp
        rospy.loginfo('Formulating an hypothesis')
        #subscribes to hypothesis maker and sends to oracle to check
        #Oracle = rospy.ServiceProxy('oracle', oracle)
        #oracle_res = oracle(resp_hyp.id)
        oracle_res = 1
        if oracle_res == 0:
            return 'stop'
        else: 
         return 'move'
        
def main():
    rospy.init_node('state_machine_try')

    sm = smach.StateMachine(outcomes=['stop'])

    with sm:
        smach.StateMachine.add('Move', Move(), transitions={'clues':'Clues'})
        smach.StateMachine.add('Clues', Clues(), transitions={'move':'Move', 'hypothesis':'Hypothesis'})
        smach.StateMachine.add('Hypothesis', Hypothesis(), transitions={'move':'Move', 'stop':'stop'})

    sis = smach_ros.IntrospectionServer('state_machine', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()