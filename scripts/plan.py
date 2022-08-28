#! /usr/bin/env python

## @package expr_rob_lab
#
#  \file hypothesis_maker.py
#  \brief This script handles the ROSplan services and the knowledge.
#
#  \author Jacopo Ciro Soncini
#  \version 1.0
#  \date 28/08/2022
#  \details
#  
#  Subscribes to: <BR>
#       None
#
#  Publishes to: <BR>
#	    None
#
#  Services: <BR>
#       /rosplan_knowledge_base/update
#       /rosplan_plan_dispatcher/dispatch_plan
#       /rosplan_parsing_interface/parse_plan
#       /rosplan_planner_interface/planning_server
#       /rosplan_problem_interface/problem_generation_server
#
#  Client Services: <BR>
#       None
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#       This node handles the ROSPlan services, updating the knowledge as needed with every new plan.

import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import time

##
#	\brief This function update the hypothesis_complete fact
#	\param : 
#	\return : None
# 	
#	This function call the knowledge base server to update the predicate
def complete():
    print("5")
    req=KnowledgeUpdateServiceRequest()
    req.update_type= 2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type= 2
    req.knowledge.attribute_name= 'hypothesis_complete'
    result=update(req)
    req=KnowledgeUpdateServiceRequest()
    req.update_type= 0
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type= 2
    req.knowledge.attribute_name= 'hypothesis_complete'
    req.knowledge.function_value = 2
    result=update(req)

##
#	\brief This function update the checked fact
#	\param : 
#	\return : None
# 	
#	This function call the knowledge base server to update the predicate
def checked():
    print("4")
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'checked'
    result=update(req)

##
#	\brief This function update the location_visited fact
#	\param : 
#	\return : None
# 	
#	This function call the knowledge base server to update the predicate
def location_visited(wp):
    print(wp)
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'reached'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', wp))	
    result=update(req)
    
    req=KnowledgeUpdateServiceRequest()
    req.update_type=0
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'visited'
    req.knowledge.values.append(diagnostic_msgs.msg.KeyValue('waypoint', wp))	
    result=update(req)

##
#	\brief This function updates the knowledge base
#	\param :
#	\return : None
# 	
#	This function update the plan knowledge with every new plan.

def new_knowledge():
    print("3")
    
    location_visited('wp0')
    location_visited('wp1')
    location_visited('wp2')
    location_visited('wp3')

    checked()
    complete()
##
#	\brief This function is main function
#	\param :
#	\return : None
# 	
#	This function initializes all of the needed services, then it start a while loop that goes on until the goal is reached.
def main():
    global update
    rospy.init_node('plan')
    
    rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
    problem = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    rospy.wait_for_service('/rosplan_planner_interface/planning_server')
    planning = rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
    parse = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
    dispatch = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan',DispatchService)	
    rospy.wait_for_service('/rosplan_knowledge_base/update')
    update= rospy.ServiceProxy('/rosplan_knowledge_base/update', KnowledgeUpdateService)
    rospy.wait_for_service("/hypothesis_maker")
    print("1")
    while(True):
        print("2")
        problem()
        
        planning()
        
        parse()
        
        res=dispatch()
        
        print(res)

        new_knowledge()
        time.sleep(1)
        if res.success and res.goal_achieved:
            print("DONE")
            break

if __name__ == '__main__':
    main()