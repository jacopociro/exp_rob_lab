#include "exp_rob_lab/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <exp_rob_lab/Hypothesis.h>
#include <string.h>
#include <exp_rob_lab/Hyp.h>

// subscribe a complete e publisha a oracle_check
exp_rob_lab::Hyp data;

void clbk(exp_rob_lab::Hyp msg){
    data = msg;
}

namespace KCL_rosplan{
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