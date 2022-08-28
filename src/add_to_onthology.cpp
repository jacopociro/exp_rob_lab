/** @ package exp_rob_lab
* 
*  \file add_to_onthology.cpp
*  \brief implements the add_to_onthology action
*
*  \author Jacopo Ciro Soncini
*  \version 1.0
*  \date 28/08/2022
*  \details
*   
*  Subscribes to: <BR>
*	/oracle_hint
*
*  Publishes to: <BR>
*	/complete
*
*  Services: <BR>
*    None
* 
*  Client Services: <BR>
*    /hypothesis_maker
*    
*  Action Services: <BR>
*    None
*
*  Description: <BR>
*  This program implements the real action to be completed when the planner
* dispatches the action add to onthology. It also calls the needed service, publisher and subscriber.
*/

#include "exp_rob_lab/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <cstdlib>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <exp_rob_lab/ErlOracle.h>
#include <exp_rob_lab/Hypothesis.h>
#include <string.h>
#include <exp_rob_lab/Hyp.h>

int gohigh();
int golow();
void hint_clbk(const exp_rob_lab::ErlOracle msg);
int arm_pos();

bool rec = false;
int pos = 0;
exp_rob_lab::ErlOracle data;


ros::Publisher pub;

namespace KCL_rosplan{
/**
 * \brief: HypothesisInterface callback
 * \param msg : rosplan_dispatch_msgs::ActionDispatch, variables received from the plan dispatcher
 * 
 * \return True
 * 
 * This function implements the behaviour for the robot when the planner dispatches
 * the action add_to_onthology
 */    
    HypothesisInterface::HypothesisInterface(ros::NodeHandle &nh){

    }

    bool HypothesisInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){
        std::cout << "Reaching the hint" << std::endl;
        arm_pos();
        ROS_INFO("Action performed!");
        return true ;
    }
}

/**
 * \brief: main function
 * \param msg : None
 * 
 * \return 0
 * 
 * This function is the main function, calling the needed publisher and subscribers
 */   

int main(int argc, char **argv){
    ros::init(argc, argv, "hypothesis_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");

    ros::NodeHandle n;
    ros::Subscriber hint = n.subscribe("/oracle_hint", 1000, hint_clbk);

    ros::NodeHandle h1;
    pub = h1.advertise<exp_rob_lab::Hyp>("/complete", 1000);
    
    KCL_rosplan::HypothesisInterface act(nh);
    act.runActionInterface();
    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(1);
    return 0;
}
/**
 * \brief: This function handles the arm position on moveit and the call to hypothesis_maker
 * \param msg : None
 * 
 * \return 0
 * 
 * This function implements the behaviour for the arm on moveit. The arm move to find the clue and then the function
 * send the hint to the hypthesis_maker service to receive a full hypothesis. the hypothesis is also published on /complete.
 */   
int arm_pos() {
    ros::NodeHandle h;
    ros::ServiceClient client = h.serviceClient<exp_rob_lab::Hypothesis>("hypothesis_maker");
      
    while(rec == false){
        if (pos == 0){
            golow();
            pos = 1;
            sleep(2);
            break;
        }
        else if (pos == 1){
            gohigh();
            pos = 0;
            sleep(2);
            break;
        }
    }
    if  (rec == true){
        exp_rob_lab::Hypothesis req;
        std::string s = std::to_string(data.ID);
        req.request.id = s;
        req.request.class_id = data.key;
        req.request.name = data.value;

        client.call(req);
        

        sleep(2);
        exp_rob_lab::Hyp risposta;
        risposta.id = req.response.id;
        risposta.who = req.response.who;
        risposta.where = req.response.where;
        risposta.what = req.response.what;
        risposta.consistent = req.response.consistent;

        pub.publish(risposta);
        

    }
    
    rec = false;
    return 0;
}
/**
 * \brief: oracle_hint callback
 * \param msg : exp_rob_lab::ErlOracle msg
 * 
 * \return 0
 * 
 * This function implements the callback to read the oracle_hint msg and save it on a global variable.
 */   
void hint_clbk(const exp_rob_lab::ErlOracle msg){
    data.ID = msg.ID;
    data.key = msg.key;
    data.value = msg.value;
    rec = true;
}
/**
 * \brief: moveit plan for the arm
 * \param msg : None
 * 
 * \return 0
 * 
 * This function implements the arm movement to go to the highest point.
 */
int gohigh(){
    geometry_msgs::Pose pose1;
	moveit::planning_interface::MoveGroupInterface group("arm");
	group.setEndEffectorLink("arm_link_04");
	group.setPoseReferenceFrame("arm_base_link");
	group.setPlannerId("RRTstar");
	group.setNumPlanningAttempts(10);
	group.setPlanningTime(10.0);
	group.allowReplanning(true);
	group.setGoalPositionTolerance(0.01);
	pose1.position.x = 0.0;
	pose1.position.y = 0.5;
	pose1.position.z = 1.25;
	group.setStartStateToCurrentState();
	group.setApproximateJointValueTarget(pose1, "arm_link_04");
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	group.plan(plan);
	group.execute(plan);
	return 0;
}
/**
 * \brief: moveit plan for the arm
 * \param msg : None
 * 
 * \return 0
 * 
 * This function implements the arm movement to go to the lowest point.
 */
int golow(){
    geometry_msgs::Pose pose1;
	moveit::planning_interface::MoveGroupInterface group("arm");
	group.setEndEffectorLink("arm_link_04");
	group.setPoseReferenceFrame("arm_base_link");
	group.setPlannerId("RRTstar");
	group.setNumPlanningAttempts(10);
	group.setPlanningTime(10.0);
	group.allowReplanning(true);
	group.setGoalPositionTolerance(0.01);
	pose1.position.x = 0.0;
	pose1.position.y = 0.5;
	pose1.position.z = 0.75;
	group.setStartStateToCurrentState();
	group.setApproximateJointValueTarget(pose1, "arm_link_04");
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	group.plan(plan);
	group.execute(plan);
	return 0;
}