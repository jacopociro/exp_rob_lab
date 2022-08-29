#! /usr/bin/env python2

## @package expr_rob_lab
#
#  \file hypothesis_maker.py
#  \brief This script handles the hypothesis maker service and the onthology part.
#
#  \author Jacopo Ciro Soncini
#  \version 1.0
#  \date 15/11/2021
#  \details
#  
#  Subscribes to: <BR>
#       None
#
#  Publishes to: <BR>
#	    None
#
#  Services: <BR>
#       hypothesis_maker
#
#  Client Services: <BR>
#       armor_interface_srv
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
<<<<<<< HEAD
#      This node handles the armor service, updating the knowledge. It return the hypothesis, a complete or incomplete one
=======
#      This node handles the armor service, updating the knowledge. It return the hypothesis, a complete or incomplete one.
>>>>>>> 949412ae0765622f01931a08e8e0310bf027cd76

from os import name
from exp_rob_lab.srv import *
import rospy
from armor_msgs.msg import *
from armor_msgs.srv import *


locations = [[]]
weapons = [[]]
people = [[]]

##
# \brief this function initializes the node, the hypothesis_maker service and the proxy to the armor service   
# \param: None
# \return: None
# This is the main function, it initializes the node and the services. it calls the various function to update the armor knowledge.
def hypothesis_maker_server():
    global armor
    rospy.init_node('hypothesis_maker2')
    rospy.wait_for_service('armor_interface_srv')

    armor = rospy.ServiceProxy('armor_interface_srv', ArmorDirective)

    load()
    comp_consist()
    hypo_detail()
    rospy.Service('hypothesis_maker', Hypothesis, make_hypothesis)
    print('Ready to formulate hypothesis')
    rospy.spin()

##
# \brief this function defines the load command for armor  
# \param: None
# \return: None
# 
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
        
    except rospy.ServiceException as e:
        print(e)
##
# \brief this function is the handle of the hypothesis maker service. it returns the response to the state machine.
# \param: request, HypothesisRequest
# \return: res, HypothesisResponse
# This function handles the service callback. Depending on the received hint and the updated knowledge it returns a complete or incomplete hypothesis.
def make_hypothesis(request):
    global people, locations, weapons
    
    id = request.id
    name = request.name
    class_id = request.class_id

    
    add_to_ont(name, class_id)
    check(id,name,class_id)
    
    if class_id == 'who':
        who = name
    else:
        who = ''

    if class_id == 'what':
        what = name
    else:
        what = ''

    if class_id == 'where':
        where = name
    else:
        where = ''

    control = hypo_control(id)

    if control == True:

        pos_people = position_find(people, id)
        #print(pos_people)
        pos_place = position_find(locations, id)
        #print(pos_place)
        pos_weapon = position_find(weapons, id)
        #print(pos_weapon)

        person = people[pos_people][0] 
        #print(person)
        who = add_to_hypo(id,person,'who')
        place = locations[pos_place][0]
        #print(place)
        where = add_to_hypo(id,place,'where')
        weapon = weapons[pos_weapon][0]
        #print(weapon)
        what = add_to_hypo(id,weapon,'what')
        consistent = True
    else:
        consistent = False
        
        

    disjoint(class_id)
    apply()
    reason()

    res = HypothesisResponse(id, who, where, what, consistent)
    print(res)

    return res

##
# \brief this function loads in the onthology the completed and incostistent for armor
# \param: None
# \return: None

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
        #print(' COMP_CONSISTENT_2 OK')
        
        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'IND'
        request.secondary_command_spec= 'CLASS'
        request.args= ['INCONSISTENT']
        msg = armor(request)
        res = msg.armor_response
        #print(' COMP_CONSISTENT_2 OK')
        

    except rospy.ServiceException as e:
        print(e)


##
# \brief this function defines the hypothesis' classes
# \param: None
# \return: None
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
        #print(' HYPO_DETAIL_1 OK')
        

        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'OBJECTPROP'
        request.secondary_command_spec= 'IND'
        request.args= ['where', 'HP0']
        msg = armor(request)
        res = msg.armor_response
        #print(' HYPO_DETAIL_2 OK')
        

        request=ArmorDirectiveReq()
        request.client_name = 'hypothesis_maker'
        request.reference_name= 'cluedo_ont'
        request.command= 'QUERY'
        request.primary_command_spec= 'OBJECTPROP'
        request.secondary_command_spec= 'IND'
        request.args= ['what', 'HP0']
        msg = armor(request)
        res = msg.armor_response
        #print(' HYPO_DETAIL_3 OK')
        

    except rospy.ServiceException as e:
        print(e)

##
# \brief this function adds an hyopthesis to the reasoner
# \param: None
# \return: None
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
        return name
        #print(' ADD_TO_HYPO OK')
        

    except rospy.ServiceException as e:
        print(e)

##
# \brief this function adds an hypothesis to the onthology
# \param: None
# \return: None
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
        #print(' ADD_TO_ONT OK')
        

    except rospy.ServiceException as e:
        print(e)

##
# \brief this function calls the disjoint command
# \param: None
# \return: None
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
        #print(' DISJOINT OK')
        

    except rospy.ServiceException as e:
        print(e)

##
# \brief this function gets the class_id and returns the class name for the onthology
# \param: None
# \return: None        
def class_ont(class_id):
        if class_id == 'who':
            return  'PERSON'
        elif class_id == 'what':
            return 'WEAPON'
        elif class_id == 'where':
            return  'PLACE'

##
# \brief this function calls the reason command for armor
# \param: None
# \return: None             
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
        #print(' REASON OK')
        

        return res.is_consistent
    except rospy.ServiceException as e:
        print(e)

##
# \brief this function calls the apply command for armor
# \param: None
# \return: None 
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
        #print(' APPLY OK')
        

        return res.is_consistent
    except rospy.ServiceException as e:
        print(e)

##
# \brief this function check if the hypothesis has already beeen added to the matrix where I store them, if not it add the hypothesis' name and id.
# \param: id, name, class_id; all strings
# \return: None 
def check(id,name,class_id):
    global locations, weapons, people
    
    if class_id == 'who':
        control = matrix_find(people, name)
        if control == False:
            temp = [name, id]
            people.append(temp)
    elif class_id == 'what':
        control = matrix_find(weapons, name)
        if control == False:
            temp = [name, id]
            weapons.append(temp)
    elif class_id == 'where':
        control = matrix_find(locations, name)
        if control == False:
            temp = [name, id]
            locations.append(temp)

    print('people:\n')
    print(people)

    print('weapons:\n')
    print(weapons)

    print('locations:\n')
    print(locations)

##
# \brief this function finds the idex of the row I'm looking for and return it
# \param: matrix, value
# \return: i, int32 
def position_find(matrix, value):

    i = 0
    for row in matrix:
        for element in row:
            if element == value:
                    #print('row count is: %d' %i)
                    return i
        i =+ 1

##
# \brief this function return if a value exists in a matrix
# \param: matrix, value
# \return: Boolean Value
def matrix_find(matrix, value):

    for row in matrix:
        for element in row:
            if element == value:
                return True
    return False

##
# \brief this function sees if in the matrix exists a complete hypothesis
# \param: id, string
# \return: Boolean Value
def hypo_control(id):

    global people, locations, weapons

    a = matrix_find(people, id)
    b = matrix_find(locations, id)
    c = matrix_find(weapons, id)
    if a == True and b == True and c == True:
        return True
    else: 
        return False

if __name__ == '__main__':
    try:
        hypothesis_maker_server()
    except rospy.ROSInterruptException: pass
