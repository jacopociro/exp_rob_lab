#! /usr/bin/env python

## @package expr_rob_lab
#
#  \file state_machine.py
#  \brief This script handles the state machine's states calling services as he needs. This is the main program.
#
#  \author Jacopo Ciro Soncini
#  \version 1.0
#  \date 29/08/2022
#  \details
#  
#  Subscribes to: <BR>
#       /complete
#
#  Publishes to: <BR>
#	    /cmd_vel
#
#  Services: <BR>
#       oracle_solution
#
#  Client Services: <BR>
#      None
#
#  Action Services: <BR>
#       Move_base
#
#  Description: <BR>
#    The state machine has three differente states: Move, Clues and Hyp. Move handles the movement of the robot between the rooms. Clues looks for the hints 
#    Hyp is a state that simply check the /oracle_solution service using the identifier of the hypothesis.


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

Kitchen = Ballroom = Conservatory = Dining_Room = Biliard_Room = Library = True
ID_complete = []
resp_hyp = HypothesisResponse
var = Hint

##
# \brief this function looks for clues
# \param: None
# \return: None   
# This function moves the arm up and down while the robots move around, to make sure that the aruco camera on the top of the arm can find the clue in the room.
# The arm movement is handled with moveit, the angular velocity is sent on /cmd_vel

def look_for_clues():
    speed = Twist()
    goal = group_cmd.get_current_joint_values()
    goal[1] = 0
    goal[2] = 0
    goal[3] = 0
    group_cmd.go(goal, wait=False)
    time.sleep(10)

    speed.angular.z = 0.8
    cmd_vel.publish(speed)
    
    time.sleep(30)
    speed.angular.z = 0
    cmd_vel.publish(speed)

    goal[1] = math.pi/4
    goal[2] = math.pi/4
    goal[3] = -math.pi/2 + 0.05
    group_cmd.go(goal, wait=False)
    time.sleep(10)
    
    speed.angular.z = -0.8
    cmd_vel.publish(speed)
    
    time.sleep(30)
    speed.angular.z = 0
    cmd_vel.publish(speed)

##
# \brief this function return a random room for the robot to move in
# \param: a, int32
# \return: room, string   
# This function checks the value of a; if a == 1 it chooses a random room to move in, sending the values of the goal to the move_base action service. If a is not 1, it sends
# the home goal. 
def move(a):
    global action, Kitchen,Ballroom,Conservatory,Dining_Room,Biliard_Room,Library
    g = MoveBaseGoal()

    g.target_pose.pose.orientation.w=0.1
    g.target_pose.header.frame_id='map'

    if a == 1:
        print("Going To Room")
        
        
        while True:
            room = random.choice(['Kitchen', 'Ballroom', 'Conservatory', 'Dining_Room', 'Biliard_Room', 'Library'])
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

    g.target_pose.pose.position.x= x
    g.target_pose.pose.position.y= y
    print(g)
    action.send_goal(g)
    res = action.wait_for_result()
    if res:
        rospy.loginfo("Room Reached!")

    return room
    
# def callback(msg):

    global var
    #rospy.loginfo(msg)
    var = msg
    return var

##
# \brief this class defines the state Move, the execute is just a sleep to simulate movement. It returns only clues.
# \param: None
# \return: None  
# This state handles the movement, calling the move function
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['clues'])
        
    def execute(self, userdata):
        place = move(1)
        rospy.loginfo('Moving to %s' %place)
        time.sleep(2)

        return 'clues'

##
# \brief this class defines the state Clues, the execute subscribes to the hint publisher and handles the hypothesis_maker service.
#  It returns move or hypothesis to change states.
# \param: None
# \return: None  
# This state handles the looking for clues behaviour, calling the look_for_clues function. Based on the global variable data it move to different states.
class Clues(smach.State):
##
# \class Clues
# \brief 
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'hypothesis'])
        

    def execute(self, userdata):
        global resp_hyp, var
        rospy.loginfo("Looking for clues...")
        look_for_clues()
        #subscribes to hint_publisher, publishes to hypothesis_maker


        
        try:
            if data == False:
                return 'move'
            else:
                return 'hypothesis'

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

##
# \brief this class defines the state Hyp, the execute handles the oracle service. It returns move if the hypothesis is mistaken or stop if the hypothesis is correct.
# \param: None
# \return: None  
# This state handles the checking the hypothesis behaviour, calling themove function, to go back to home, and the solution service.
# If the solution matches with the one from the service the state machines stops.
class Hyp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'stop'])

    def execute(self, userdata):
        global resp_hyp
        
        rospy.loginfo('Formulating an hypothesis')
        rospy.loginfo('Moving to terminal')
        move(2)

        try:
            oracle_res = solution_srv()
            for x in ID_complete:
                if oracle_res.ID == ID_complete[x]:
                    print("Yay! I got the answer right!")
                    return 'stop'

            print("Mmh... I need more hints...")
            return 'move'
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


def cb(msg):
    global data
    data = msg.data

def clbk(msg):
    global ID_complete
    ID_complete.append(msg.data) 
##
# \brief this is the main function, it calls the smach state machine, all of the states and a sis to see the state machine graph. Also calls the publishers and the 
# services.
# \param: None
# \return: None  

def main():

    global action, cmd_vel, arm, group_cmd, consistent, solution_srv
    rospy.init_node('state_machine')
    action = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    action.wait_for_server()
    solution_srv = rospy.ServiceProxy("/oracle_solution", Oracle)
    rospy.wait_for_service('/oracle_solution')
    rospy.wait_for_service('/hypothesis_maker')
    rospy.wait_for_service('/oracle_hint')
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    consistent = rospy.Subscriber('/consistent', Bool, cb)
    complete = rospy.Subscriber('/complete', String, clbk)
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