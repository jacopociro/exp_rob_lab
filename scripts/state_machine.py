#! /usr/bin/env python

## @package expr_rob_lab
#
#  \file state_machine.py
#  \brief This script handles the state machine's states calling services as he needs. This is the main program.
#
#  \author Jacopo Ciro Soncini
#  \version 1.0
#  \date 15/11/2021
#  \details
#  
#  Subscribes to: <BR>
#       hint
#
#  Publishes to: <BR>
#	    None
#
#  Services: <BR>
#       
#
#  Client Services: <BR>
#       hypothesis_maker
#       oracle
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#    The state machine has three differente states: Move, Clues and Hyp. Move handles the movement of the robot between the rooms.
#    This movement is fake and is implemented by a simple sleep that simulates the time taken to move. Clues handles the hints 
#    by subscribing to the hint publisher node and sending the information requested by the hypothesis_maker service. The response
#    is saved in a global variable so that it can be passed to the last state, Hyp, if the hypothesis is deemed consistent. 
#    Hyp is a state that simply reads the identifier of the hypothesis and sends it to the oracle service that knows if the hypothesis
#    is right.


import rospy
import smach
import random
import time
import smach_ros
from exp_rob_lab.msg import Hint
from exp_rob_lab.srv import *

resp_hyp = HypothesisResponse
var = Hint

##
# \brief this function return a random room for the robot to move in
# \param: None
# \return: room, string   
# This function simulates the move action
def room():

    return random.choice(['Kitchen', 'Ballroom', 'Conservatory', 'Dining_Room', 'Biliard_Room', 'Library', 'Lounge', 'Hall', 'Study'])

##
# \brief this function returns the message published from hint
# \param: msg, Hint
# \return: var, Hint
def callback(msg):

    global var
    #rospy.loginfo(msg)
    var = msg
    return var

#
# \brief this class defines the state Move, the execute is just a sleep to simulate movement. It returns only clues.
# \param: None
# \return: None  
# This state handles the movement, calling the move function
class Move(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['clues'])
        
    def execute(self, userdata):
        place = room()
        rospy.loginfo('Moving to %s' %place)
        time.sleep(2)

        return 'clues'

##
# \brief this class defines the state Clues, the execute subscribes to the hint publisher and handles the hypothesis_maker service.
#  It returns move or hypothesis to change states.
# \param: None
# \return: None  
# This state handles the looking for clues behaviour, calling the publisher and the service. Based on the response it move to different states.
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

##
# \brief this class defines the state Hyp, the execute handles the oracle service. It returns move if the hypothesis is mistaken or stop if the hypothesis is correct.
# \param: None
# \return: None  
# This state handles the checking the hypothesis behaviour, calling the oracle service.
# If the solution matches with the one from the service the state machines stops.
class Hyp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'stop'])

    def execute(self, userdata):
        global resp_hyp
        
        rospy.loginfo('Formulating an hypothesis')
        rospy.loginfo('Moving to terminal')
        time.sleep(2)

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

##
# \brief this is the main function, it calls the smach state machine, all of the states and a sis to see the state machine graph.
# \param: None
# \return: None   
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