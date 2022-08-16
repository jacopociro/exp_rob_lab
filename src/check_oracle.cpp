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
    OracleInterface::OracleInterface(ros::NodeHandle &nh){

    }

    bool OracleInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/complete", 1000, clbk);
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



void clbk(const exp_rob_lab::Hyp msg){
    data = msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "oracle", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::OracleInterface act(nh);
    act.runActionInterface();
    return 0;
}