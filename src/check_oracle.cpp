/** @ package exp_rob_lab
* 
*  \file check_oracle.cpp
*  \brief implements the check_oracle action
*
*  \author Jacopo Ciro Soncini
*  \version 1.0
*  \date 28/08/2022
*  \details
*   
*  Subscribes to: <BR>
*	/complete
*
*  Publishes to: <BR>
*	None
*
*  Services: <BR>
*    None
* 
*  Client Services: <BR>
*    None
*    
*  Action Services: <BR>
*    None
*
*  Description: <BR>
*  This program implements the real action to be completed when the planner
* dispatches the action check_oracle. It also calls the needed subscriber.
*/

#include "exp_rob_lab/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <exp_rob_lab/Oracle.h>
#include <exp_rob_lab/Hypothesis.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <exp_rob_lab/Hyp.h>

exp_rob_lab::Hyp data;
void clbk(const exp_rob_lab::Hyp msg);
namespace KCL_rosplan{
/**
 * \brief: OracleInterface callback
 * \param msg : rosplan_dispatch_msgs::ActionDispatch, variables received from the plan dispatcher
 * 
 * \return True or False
 * 
 * This function implements the behaviour for the robot when the planner dispatches
 * the action oracle_check
 */ 
    OracleInterface::OracleInterface(ros::NodeHandle &nh){

    }

    bool OracleInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){

        ros::NodeHandle n1;
        ros::ServiceClient client = n1.serviceClient<exp_rob_lab::Oracle>("/oracle_solution");
        
        exp_rob_lab::Oracle resp;

        client.call(resp);
        std::string s = std::to_string(resp.response.ID);
        if (s == data.id){
            ROS_INFO("Action (%s) performed", msg->name.c_str());
            printf("SOLUTION FOUND");
            return true;
        }
        else{
            ROS_INFO("Action (%s) failed!", msg->name.c_str());
            printf("SOLUTION NOT CORRECT");
            return false;
        }
    }
}


/**
 * \brief: /complete subscriber callback
 * \param msg : exp_rob_lab::Hyp msg
 * 
 * \return None
 * 
 * This function implements the behaviour of the callback for /complete subscriber adn saves the message received.
 */ 
void clbk(const exp_rob_lab::Hyp msg){
    data = msg;
}

/**
 * \brief: main function
 * \param msg : None
 * 
 * \return 0
 * 
 * This function implements the main function, with the subscrber initialiazer.
 */ 
int main(int argc, char **argv){
    ros::init(argc, argv, "oracle", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/complete", 1000, clbk);
    KCL_rosplan::OracleInterface act(nh);
    act.runActionInterface();
    ros::AsyncSpinner spinner(1);
    spinner.start();
    return 0;
}