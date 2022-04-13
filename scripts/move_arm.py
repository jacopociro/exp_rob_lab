#! /usr/bin/env python

import queue
import sys
import copy
from tokenize import group
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from exp_rob_lab.srv import *

group = None
def arm_callback(req):
    global group

    joint_goal = group.get_current_joint_values()
    joint_goal[0] = req.joint0
    joint_goal[1] = req.joint1
    joint_goal[2] = req.joint2
    joint_goal[3] = req.joint3
    joint_goal[4] = req.joint4

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.set_start_state_to_current_state()
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()
  
    return True 

def main():
    global group
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print ("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print ("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")

    group.set_goal_tolerance(0.05)

    move_arm_server = rospy.Service('arm_service', Arm, arm_callback)

    rospy.spin()

if __name__== '__main__':
    main()


