#! /usr/bin/env python

#! /usr/bin/env python2

## @package expr_rob_lab
#
#  \file hypothesis_maker.py
#  \brief This script handles the hypothesis maker service and the onthology part.
#
#  \author Jacopo Ciro Soncini
#  \version 1.0
#  \date 28/08/2022
#  \details
#  
#  Subscribes to: <BR>
#       /odom
#
#  Publishes to: <BR>
#	    /cmd_vel
#
#  Services: <BR>
#       none
#
#  Client Services: <BR>
#       none
#
#  Action Services: <BR>
#       /reaching_goal
#
#  Description: <BR>
#   This node is the node tasked with making sure that the robot reaches the goal, both in orientation and position.
#   The behaviour is handled with 3 states. The first one is tasked with aiming for the goal, using angular velocities. 
#   The second state makes sure the robot reaches the goal, giving linear velocities. The last state handles the orientation
#   given with the goal, using angular velocities. The angular and linear velocities are published on the /cmd_vel publisher.
#   The goal is handled with the /reaching_goal action. On the /odom subsciber the node reads the robot position


# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import motion_plan.msg

# robot state variables
position_ = Point()
pose_ = Pose()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0  
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publisher
pub = None

# action_server
act_s = None

# callbacks

##
#	\brief This function is the /odom subscriber callback
#	\param msg: Odometry
#	\return : None
# 	
#	The callback saves the current position and orientation of the robot on global variables, reading it from /odom. 
#   I needed to change the orientation from quaternion to euler.

def clbk_odom(msg):
    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##
#   \brief This function handles the state change.
#	\param state: Int
#	\return : None
# 	
#	This function saves the passed parameter state to a global variable. Prints a message for 
#   clear output.
def fix_final_yaw(des_yaw):
    global Vel
    # It orients the robot in the final desired orientation 
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)

##
#   \brief This function handles the state change.
#	\param state: Int
#	\return : None
# 	
#	This function saves the passed parameter state to a global variable. Prints a message for 
#   clear output.

def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

##
#	\brief This function changes the angle to a fixed interval
#	\param angle: Float
#	\return : Float
# 	
#	This function receives and angle, normalizes it in the interval [-pi, pi] and returns the
#   new angle.

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
#	\brief This function turns the robot towards the goal
#	\param des_pos: array
#	\return : None
# 	
#	This function calculates the yaw the robot needs to reach to face the goal and the publishes 
#   the angular velocity to reach it. The algorithm is implemented with a treshold that needs to be 
#   satisfied before changing state.

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

##
#	\brief This function makes the robot reach the positional goal
#	\param des_pos: array
#	\return : None
# 	
#	This function publishes the velocity to reach the goal until the set treshold is satisfied.
#   Then it changes the state.

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


##
#	\brief This function is for stopping the robot
#	\param : None
#	\return : None
# 	
#	This function is called when the robot has reached the goal and publishes all velocities
#   as zero. 

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


##
#	\brief This function is for stopping the robot
#	\param : goal: motion_plan.msg.PlanningAction
#	\return : None
# 	
#	This is the function called with the action and handle the various behaviour of the robot. This function is tasked
#   with calling the other functions.

def planning(goal):

    global state_, desired_position_
    global act_s

    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    desired_yaw = goal.target_pose.pose.orientation.w
    state_ = 0
    rate = rospy.Rate(20)
    success = True

    feedback = motion_plan.msg.PlanningFeedback()
    result = motion_plan.msg.PlanningResult()

    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            break
        elif state_ == 0:
            feedback.stat = "Fixing the yaw"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position_)
        elif state_ == 1:
            feedback.stat = "Angle aligned"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            fix_final_yaw(desired_yaw)
        elif state_ == 3:
            feedback.stat = "Target reached!"
            feedback.actual_pose = pose_
            act_s.publish_feedback(feedback)
            done()
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)


##
#	\brief This function initializes the ros node and the various ros publisher, subscribers and action.
#	\param : None
#	\return : None
# 	
#	This is the main function. It initializes the node, the publisher, the subscribers and actionserver
def main():
    global pub, active_, act_s
    rospy.init_node('go_to_point')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/reaching_goal', motion_plan.msg.PlanningAction, planning, auto_start=False)
    act_s.start()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()




