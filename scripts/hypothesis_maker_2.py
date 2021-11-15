#! /usr/bin/env python2

from os import name
from exp_rob_lab.srv import *
import rospy
from armor_msgs.msg import *
from armor_msgs.srv import *

def hypothesis_maker_server():
    global armor
    rospy.init_node('hypothesis_maker2')
    rospy.wait_for_service('armor_interface_srv')

    armor = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)

    load()
    comp_consist()
    hypo_detail()
    rospy.Service('hypothesis_maker', Hypothesis, make_hypothesis)
    print('ready to formulate hypothesis')
    rospy.spin()

def load():
    try:
        request=ArmorDirectiveReq()
        request.client_name= 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'LOAD'
        request.primary_command_spec= 'FILE'
        request.secondary_command_spec= ''
        request.args= ['/root/ros_ws/src/exp_rob_lab/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
        msg = armor(request)
        print('load ok')
        res=msg.armor_response
        #print(res)
    except rospy.ServiceException as e:
        print(e)

def make_hypothesis(request):
    print('make_hypothesis')
    
    id = request.id
    name = request.name
    class_id = request.class_id
    add_to_hypo(id,name,class_id)
    add_to_ont(name, class_id)
    
    disjoint(class_id)
    apply()
    consistent = reason()
    
    
    if class_id == 'who':
            who = name
            what = ''
            where = ''
    elif class_id == 'what':
            what = name
            who = ''
            where = ''
    elif class_id == 'where':
            where = name
            who = ''
            what = ''

    if consistent == 'True':
        is_consistent = True
    else:
        is_consistent = False
    res = HypothesisResponse(id, who, what, where, is_consistent)
    print(res)

    return res


def comp_consist():
    try:
        
        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'IND'
        request.secondary_command_spec= 'CLASS'
        request.args= ['COMPLETED']
        msg = armor(request)
        msg = armor(request)
        res = msg.armor_response
        print(' COMP_CONSISTENT_2 OK')
        #if res.is_consistent == 'True':
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN COMP_CONSIST')
        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'IND'
        request.secondary_command_spec= 'CLASS'
        request.args= ['INCONSISTENT']
        msg = armor(request)
        res = msg.armor_response
        print(' COMP_CONSISTENT_2 OK')
        #if res.is_consistent == 'True':
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN COMP_CONSIST')

    except rospy.ServiceException as e:
        print(e)

def hypo_detail():

    try:
        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'OBJECTPROP'
        request.secondary_command_spec= 'IND'
        request.args= ['who', 'HP0']
        msg = armor(request)
        res = msg.armor_response
        print(' HYPO_DETAIL_1 OK')
        #if res.is_consistent == 'True':
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN HYPO_DETAIL')

        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'OBJECTPROP'
        request.secondary_command_spec= 'IND'
        request.args= ['where', 'HP0']
        msg = armor(request)
        res = msg.armor_response
        print(' HYPO_DETAIL_2 OK')
        #if res.is_consistent == True:
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN HYPO_DETAIL')

        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'OBJECTPROP'
        request.secondary_command_spec= 'IND'
        request.args= ['what', 'HP0']
        msg = armor(request)
        res = msg.armor_response
        print(' HYPO_DETAIL_3 OK')
        #if res.is_consistent == 'True':
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN HYPO_DETAIL')

    except rospy.ServiceException as e:
        print(e)

def add_to_hypo(id,name,class_id):
    try:
        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'ADD'
        request.primary_command_spec= 'OBJECTPROP'
        request.secondary_command_spec= 'IND'
        request.args= [class_id, id, name]
        msg = armor(request)
        res = msg.armor_response
        print(' ADD_TO_HYPO OK')
        #if res.is_consistent == 'True':
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN ADD_TO_HYPO')

    except rospy.ServiceException as e:
        print(e)

def add_to_ont(name,class_id):
    try:
        identifier = class_ont(class_id)

        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'ADD'
        request.primary_command_spec= 'IND'
        request.secondary_command_spec= 'CLASS'
        request.args= [name, identifier]
        msg = armor(request)
        res = msg.armor_response
        print(' ADD_TO_ONT OK')
        #if res.is_consistent == 'True':
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN ADD_TO_ONT')

    except rospy.ServiceException as e:
        print(e)

def disjoint(class_id):
    try:
        identifier = class_ont(class_id)

        request=ArmorDirectiveReq()
        request.client_name= 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'DISJOINT'
        request.primary_command_spec= 'IND'
        request.secondary_command_spec= 'CLASS'
        request.args= [identifier]
        msg = armor(request)   
        res = msg.armor_response
        print(' DISJOINT OK')
        #if res.is_consistent == 'True':
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN ADD_TO_HYPO')

    except rospy.ServiceException as e:
        print(e)
     
def class_ont(class_id):
        if class_id == 'who':
            return  'PERSON'
        elif class_id == 'what':
            return 'WEAPON'
        elif class_id == 'where':
            return  'PLACE'
            
def reason():
    try:
        request=ArmorDirectiveReq()
        request.client_name= 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'REASON'
        request.primary_command_spec= ''
        request.secondary_command_spec= ''
        request.args= []
        msg = armor(request)
        res=msg.armor_response
        print(' REASON OK')
        #if res.is_consistent == 'True':
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN ADD_TO_HYPO')

        return res.is_consistent
    except rospy.ServiceException as e:
        print(e)

def apply():
    try:
        request=ArmorDirectiveReq()
        request.client_name= 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'APPLY'
        request.primary_command_spec= ''
        request.secondary_command_spec= ''
        request.args= []
        msg = armor(request)
        res=msg.armor_response
        print(' APPLY OK')
        #if res.is_consistent == 'True':
        #    print('OK')
        #else:
        #    exit('SGANZO PAZZO IN ADD_TO_HYPO')

        return res.is_consistent
    except rospy.ServiceException as e:
        print(e)


if __name__ == '__main__':
    try:
        hypothesis_maker_server()
    except rospy.ROSInterruptException: pass