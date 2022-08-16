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
    HypothesisInterface::HypothesisInterface(ros::NodeHandle &nh){

    }

    bool HypothesisInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){
        std::cout << "Reaching the hint" << std::endl;
        arm_pos();
        ROS_INFO("Action performed!");
        return true ;
    }
}

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
    sleep(3);
    return 0;
}

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

void hint_clbk(const exp_rob_lab::ErlOracle msg){
    data.ID = msg.ID;
    data.key = msg.key;
    data.value = msg.value;
    rec = true;
}

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