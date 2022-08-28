/** @ package exp_rob_lab
* 
*  \file hypothesis_check.cpp
*  \brief implements the hypothesis_check action
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
* dispatches the action hypothesis_check.
*/

#include "exp_rob_lab/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <exp_rob_lab/Hypothesis.h>
#include <string.h>
#include <exp_rob_lab/Hyp.h>


exp_rob_lab::Hyp data;
/**
 * \brief: /complete subscriber callback
 * \param msg : exp_rob_lab::Hyp msg
 * 
 * \return None
 * 
 * This function implements the callback for when the /complete subscriber is called and saves the message.
 */
void clbk(exp_rob_lab::Hyp msg){
    data = msg;
}

namespace KCL_rosplan{
/**
 * \brief: Hypothesis_checkInterface callback
 * \param msg : rosplan_dispatch_msgs::ActionDispatch, variables received from the plan dispatcher
 * 
 * \return True or False
 * 
 * This function implements the behaviour for the robot when the planner dispatches
 * the action Hypothesis_checkInterface. 
 */
    Hypothesis_checkInterface::Hypothesis_checkInterface(ros::NodeHandle &nh){

    }

    bool Hypothesis_checkInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){

        if (data.consistent == true){
            return true;
        }
        else if(data.consistent == false){
            return false;
        }
    }   
}
/**
 * \brief: Main functions
 * \param msg : None
 * 
 * \return 0
 * 
 * This function is the main function, initializes the subscriber.
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "hypothesis_check", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/complete", 1000, clbk);
    KCL_rosplan::Hypothesis_checkInterface act(nh);
    act.runActionInterface();
    ros::AsyncSpinner spinner(1);
    spinner.start();
    return 0;
}