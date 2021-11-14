#! /usr/bin/env python2

from os import name
from exp_rob_lab.srv import *
from genpy import message
import rospy
from armor_msgs.msg import *
from armor_msgs.srv import *

armor = None
hypothesis = []
char = []
weapon = []
place = []


def make_hypothesis(request):
    print ('make_hypothesis')
    global hypothesis
    done = 0
    find = check(request)
    if find == 0:
        add(request.name, request.class_id)
    ont_check(request.id, request.name, request.class_id)
    send = complete_consistent(request.id)

    if send == 1:
        for i in range (len(hypothesis)):
            if request.id == hypothesis[i]:
                done = 1
            if done == 0:
                print('Hypothesis formulated')
                res = HypothesisResponse(request)
                hypothesis.append(request.id)
                t = hypo_find( request.id, 'who')
                res.who = t
                t = hypo_find(request.id, 'what')
                res.what = t
                t = hypo_find(request.id, 'where')
                res.where = t
                res.consistent = True
                print (res)
                return res
    else:
        print("not complete or not consistent")
        ret = HypothesisResponse()
        ret.id = request.id
        ret.who = 'empty'
        ret.what = 'empty'
        ret.where = 'empty'
        ret.consistent = False

        return ret



def hypothesis_maker_server():
    global armor
    rospy.init_node('hypothesis_maker_server')
    rospy.wait_for_service('armor_interface_srv')
    armor = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)
    load()
    
    rospy.Service('hypothesis_maker', Hypothesis, make_hypothesis)
    
    #load()
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
        #print('load ok')
        res=msg.armor_response
        #print(res)
    except rospy.ServiceException as e:
        print(e)

def check(data):
    global char, weapon, place
    find = 0
    #a = 0
    #b = 0
    #c = 0
    print(data)
    if data.class_id == 'who':
        for a in range(len(char)):
            if char[a]==data.name:
                find = 1
        if find == 0:
            char.append(data.name)
    if data.class_id == 'what':
        for b in range(len(weapon)):
            if weapon[b]==data.name:
                find = 1
        if find == 0:
            weapon.append(data.name)
    if data.class_id == 'where':
        for c in range(len(place)):
            if weapon[c]==data.name:
                find = 1
            if find == 0:
                place.append(data.name)
    return find


def add(name, class_id):
    try:
        class_ont = ont_class(class_id)
        request=ArmorDirectiveReq()
        request.client_name= 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'ADD'
        request.primary_command_spec= 'IND'
        request.secondary_command_spec= 'CLASS'
        request.args= [name, class_ont]
        msg = armor(request)
        res=msg.armor_response
        reason()
        disjoint(class_ont)
        reason()
    except rospy.ServiceException as e:
        print(e)



def ont_class(class_id):
    if class_id == 'who':
        return 'PERSON'
    if class_id == 'what':
        return 'WEAPON'
    if class_id == 'where':
        return 'LOCATION'

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
        #print(res)
    except rospy.ServiceException as e:
        print(e)	


def disjoint(class_ont):
    try:
        request=ArmorDirectiveReq()
        request.client_name= 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'DISJOINT'
        request.primary_command_spec= 'IND'
        request.secondary_command_spec= 'CLASS'
        request.args= [class_ont]
        msg = armor(request)		 
    except rospy.ServiceException as e:
        print(e)  

def ont_check(id, name, class_id):
    try:
        name_list = []
        res_final= hypo_find(id, class_id)
        name_list.append(name)

        hypo_form(id, name, class_id)
        reason()
    except rospy.ServiceException as e:
        print(e)

def hypo_form(id, name, class_id):
    try:
        request=ArmorDirectiveReq()
        request.client_name= 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'ADD'
        request.primary_command_spec= 'OBJECTPROP'
        request.secondary_command_spec= 'IND'
        request.args= [class_id,id,name]
        msg = armor(request)
        res=msg.armor_response
    except rospy.ServiceException as e:
        print(e)

def hypo_find(id, class_id):
    try:
        request=ArmorDirectiveReq()
        request.client_name= 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'OBJECTPROP'
        request.secondary_command_spec= 'IND'
        request.args= [class_id, id]
        msg = armor(request)
        res=msg.armor_response.queried_objects
        res_final=query(res)
        return res_final
    except rospy.ServiceException as e:
        print(e)   

def query(query):
    for i in range(len(query)):
        t=query[i]
        t=t.split('#')
        index=len(t)
        t=t[index-1]
        query[i]=t[:-1]
    return query

def complete_consistent(id):
    try:
        complete = 0
        consistent = 0
        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'IND'
        request.secondary_command_spec= 'CLASS'
        request.args= ['COMPLETED']
        msg = armor(request)
        res = msg.armor_response.queried_objects
        res_final = query(res)
        for i in range(len(res_final)):
            if res_final[i]==id:
                complete = 1

        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'IND'
        request.secondary_command_spec= 'CLASS'
        request.args= ['INCONSISTENT']
        msg = armor(request)
        res = msg.armor_response.queried_objects
        res_final = query(res)
        for i in range(len(res_final)):
            if res_final[i]==id:
                consistent = 1    
        if complete == 1 and consistent == 0:
            return 1
        else : 
            return 0
    except rospy.ServiceException as e:
        print(e)

if __name__ == '__main__':
    try:
        hypothesis_maker_server()
    except rospy.ROSInterruptException: pass