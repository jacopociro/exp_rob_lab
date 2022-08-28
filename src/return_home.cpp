/** @ package exp_rob_lab
* 
*  \file return_home.cpp
*  \brief implements the return_home action
*
*  \author Jacopo Ciro Soncini
*  \version 1.0
*  \date 28/08/2022
*  \details
*   
*  Subscribes to: <BR>
*	None
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
*    /reaching_goal
*
*  Description: <BR>
*  This program implements the real action to be completed when the planner
* dispatches the action return_home.
*/
#include "exp_rob_lab/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

namespace KCL_rosplan {
    ReturnHomeInterface::ReturnHomeInterface(ros::NodeHandle &nh){

    }
/**
 * \brief: ReturnHomeInterface callback
 * \param msg : rosplan_dispatch_msgs::ActionDispatch, variables received from the plan dispatcher
 * 
 * \return True
 * 
 * This function implements the behaviour for the robot when the planner dispatches
 * the action return_home action. Initializes and publishes to the /reaching_goal action service.
 */
    bool ReturnHomeInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){
        actionlib::SimpleActionClient<motion_plan::PlanningAction>  client("/reaching_goal", true);
        motion_plan::PlanningGoal target;
        client.waitForServer();
        std::cout << "Reaching "<<msg->parameters[1].value << "from" << msg->parameters[0].value <<std::endl;

        target.target_pose.pose.position.x = 0.0;
        target.target_pose.pose.position.y = 0.0;
        target.target_pose.pose.orientation.w= 0.0;
        client.sendGoal(target);
        client.waitForResult();
        ROS_INFO("Action(%s) performed!", msg->name.c_str());
        return true;
    }
}
/**
 * \brief: Main function
 * \param msg : None
 * 
 * \return 0
 * 
 * This function implements the main function.
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "return_home", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::ReturnHomeInterface act(nh);
    act.runActionInterface();
    return 0;
}