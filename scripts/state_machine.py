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
#       hypothesis_make
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
from geometry_msgs.msg import Twist, Point, Pose
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_commander
import moveit_msgs.msg
from math import pi
from std_msgs.msg import String, Int32, Bool
from erl2.srv import Oracle
from exp_assignment3.srv import Marker, MarkerRequest
import math

resp_hyp = HypothesisResponse
var = Hint
def look_for_clues():
    speed = Twist()
    goal = group_cmd.get_current_joint_values()
    goal[1] = 0
    goal[2] = 0
    goal[3] = 0
    group_cmd.go(goal, wait=False)
    time.sleep(20)

    speed.angular.z = 0.5
    cmd_vel.publish(speed)
    
    time.sleep(40)
    speed.angular.z = 0
    cmd_vel.publish(speed)

    goal[1] = math.pi/4
    goal[2] = math.pi/4
    goal[3] = -math.pi/2 + 0.05
    group_cmd.go(goal, wait=False)
    time.sleep(20)
    
    speed.angular.z = -0.5
    cmd_vel.publish(speed)
    
    time.sleep(40)
    speed.angular.z = 0
    cmd_vel.publish(speed)

def move(a):
##
# \brief this function return a random room for the robot to move in
    goal = MoveBaseGoal()
    if a == 1:
        print("Going To Room")
        room = random.choice(['Kitchen', 'Ballroom', 'Conservatory', 'Dining_Room', 'Biliard_Room', 'Library'])
        Kitchen = Ballroom = Conservatory = Dining_Room = Biliard_Room = Library = True
        while True:
            if room == 'Kitchen' and Kitchen == True:
                x = -4
                y = -3
                Kitchen = False
                break
            elif room == 'Ballroom' and Ballroom == True:
                x = -4
                y = 2
                Ballroom = False
                break
            elif room == 'Conservatory' and Conservatory == True:
                x = -4
                y = 7
                Conservatory = False
                break
            elif room == 'Dining_Room' and Dining_Room == True:
                x = 5
                y = -7
                Dining_Room = False
                break
            elif room == 'Biliard_Room' and Biliard_Room == True:
                x = 5
                y = -3
                Biliard_Room = False
                break
            elif room == 'Library' and Library == True:
                x = 5
                y = 1
                Library = False
                break
            elif Kitchen == False and Ballroom == False and Conservatory == False and Dining_Room == False and Biliard_Room == False and Library == False:
                Library = True
                Biliard_Room = True
                Dining_Room = True
                Conservatory = True
                Ballroom = True
                Kitchen = True
    else:
        print("Going Home")
        x = 0
        y = -1

    goal.target_pose.pose.position.x= x
    goal.target_pose.pose.position.y= y
    print(goal)
    res = action.send_goal(goal)
    print(res)
    res = action.wait_for_result()
    print(res)
        #rospy.loginfo("Room Reached!")
    


def callback(msg):
##
# \brief this function returns the message published from hint
    global var
    #rospy.loginfo(msg)
    var = msg
    return var

class Move(smach.State):
##
# \class Move
# \brief this class defines the state Move, the execute is just a sleep to simulate movement.
# It returns only clues.
    def __init__(self):
        smach.State.__init__(self, outcomes=['clues'])
        
    def execute(self, userdata):
        place = move(1)
        rospy.loginfo('Moving to %s' %place)
        time.sleep(2)

        return 'clues'

class Clues(smach.State):
##
# \class Clues
# \brief this class defines the state Clues, the execute subscribes to the hint publisher and handles
# the hypothesis_maker service. It returns move or hypothesis to change states.
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'hypothesis'])
        

    def execute(self, userdata):
        global resp_hyp, var
        rospy.loginfo("Looking for clues...")
        look_for_clues()
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
##\class Hyp
# \brief this class defines the state Hyp, the execute handles the oracle service. It returns
# move if the hypothesis is mistaken or stop if the hypothesis is correct.
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'stop'])

    def execute(self, userdata):
        global resp_hyp
        
        rospy.loginfo('Formulating an hypothesis')
        rospy.loginfo('Moving to terminal')
        move(2)
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
        
def main():
##
# \brief this is the main function, it calls the smach state machine, all of the states and a sis to see the state machine graph
    global action, cmd_vel, arm, group_cmd
    rospy.init_node('state_machine')
    action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    arm = moveit_commander.RobotCommander()
    name = "arm"
    group_cmd = moveit_commander.MoveGroupCommander(name)
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