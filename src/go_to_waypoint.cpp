#include "exp_rob_lab/InterfaceAction.h"
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include <math.h>

namespace KCL_rosplan{
    Go_To_PointInterface::Go_To_PointInterface(ros::NodeHandle &nh){
    }

    bool Go_To_PointInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg){

        actionlib::SimpleActionClient<motion_plan::PlanningAction>  client("/reaching_goal", true);
        motion_plan::PlanningGoal target;
        client.waitForServer();
        std::cout << "Reaching "<<msg->parameters[1].value << "from" << msg->parameters[0].value <<std::endl;
        
        if(msg->parameters[1].value == "wp0")
            {
                target.target_pose.pose.position.x = 2.7;
                target.target_pose.pose.position.y = 0.0;
                target.target_pose.pose.orientation.w= -M_PI/2;
            }
        else if(msg->parameters[1].value == "wp1")
            {
                target.target_pose.pose.position.x = 0.0;
                target.target_pose.pose.position.y = 2.7;
                target.target_pose.pose.orientation.w= 0.0;
            }
        else if(msg->parameters[1].value == "wp2")
            {
                target.target_pose.pose.position.x = -2.7;
                target.target_pose.pose.position.y = 0.0;
                target.target_pose.pose.orientation.w= M_PI/2;
            }
        else if(msg->parameters[1].value == "wp3")
            {
                target.target_pose.pose.position.x = 0.0;
                target.target_pose.pose.position.y = -2.7;
                target.target_pose.pose.orientation.w= M_PI;
            }
        else
            {
                std::cout << "out of bound" << std::endl ;
            }
        client.sendGoal(target);
        client.waitForResult();
        ("Action(%s) performed!", msg->name.c_str());
        return true;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "go_to_waypoint", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::Go_To_PointInterface act(nh);
    act.runActionInterface();
    return 0;
}