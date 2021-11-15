##! /usr/bin/env python2

## @package expr_rob_lab
#
#  \file oracle_service.py
#  \brief This script handle the oracle service.
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
#       oracle
#
#  Client Services: <BR>
#       None
#
#  Action Services: <BR>
#       None
#
#  Description: <BR>
#       This node handles the oracle service, here is where we define the correct hypothesis id and where we feed
#       the request, string, and get the response, bool.

from exp_rob_lab.srv import oracle, oracleResponse
import rospy

def oracle_handle(req):
##
# \brief this function hadle the service, giving a different bool variable after judging the correcteness of the 
# hypothesis
    print('Hypothesis number %s is...'%req.id)
    if req.id == 'HP3':
        print("correct!")
        right = 0
    else:
        print("wrong! Try again!")
        right = 1
    return oracleResponse(right)

def oracle_server():
##
#\brief here we initialize the node and the service
    rospy.init_node('oracle')
    s = rospy.Service('oracle', oracle, oracle_handle)
    print("Oracle is ready to tell you the truth!")
    rospy.spin()

if __name__ == '__main__':
    try:
        oracle_server()
    except rospy.ROSInterruptException: pass