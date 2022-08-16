#include "exp_rob_lab/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>

namespace KCL_rosplan {
    ReturnHomeInterface::ReturnHomeInterface(ros::NodeHandle &nh){

    }

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

int main(int argc, char **argv){
    ros::init(argc, argv, "return_home", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::ReturnHomeInterface act(nh);
    act.runActionInterface();
    return 0;
}