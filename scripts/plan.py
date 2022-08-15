#! /usr/bin/env python

import rospy
from rosplan_knowledge_msgs.srv import *
from std_srvs.srv import Empty, EmptyResponse
from rosplan_interface_mapping.srv import CreatePRM
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse, PlanningService, PlanningServiceResponse
from rosplan_knowledge_msgs.srv import KnowledgeUpdateService, KnowledgeUpdateServiceRequest
from diagnostic_msgs.msg import KeyValue
import time
import actionlib
import exp_rob_lab.msg
import random

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

def checked():
    print("4")
    req=KnowledgeUpdateServiceRequest()
    req.update_type=2
    req.knowledge.is_negative=False
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name= 'checked'
    result=update(req)

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


def new_knowledge():
    print("3")
    
    location_visited('wp0')
    location_visited('wp1')
    location_visited('wp2')
    location_visited('wp3')

    checked()
    complete()

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