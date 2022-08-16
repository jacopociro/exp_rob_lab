#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
namespace KCL_rosplan {
class HypothesisInterface: public RPActionInterface
{
private:
public:
/* constructor */
HypothesisInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class OracleInterface: public RPActionInterface
{
private:
public:
/* constructor */
OracleInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class Go_To_PointInterface: public RPActionInterface
{
private:
public:
/* constructor */
Go_To_PointInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};


class Hypothesis_checkInterface: public RPActionInterface
{
private:
public:
/* constructor */
Hypothesis_checkInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class LeaveHomeInterface: public RPActionInterface
{
private:
public:
/* constructor */
LeaveHomeInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

class ReturnHomeInterface: public RPActionInterface
{
private:
public:
/* constructor */
ReturnHomeInterface(ros::NodeHandle &nh);
/* listen to and process action_dispatch topic */
bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
};

}
