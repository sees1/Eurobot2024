#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>

#include <geometry_msgs/Twist.h>

#include <chrono>
#include <string>

class UnmanagedControl
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
  std::string action_name_;
  behavior_tree_core::BTFeedback feedback_;
  behavior_tree_core::BTResult result_;

  ros::Publisher cmd_vel_pub_;

  double velocity_x;
  double velocity_y;
  double duration;

public:
  enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status

public:
  explicit UnmanagedControl(std::string name);
  ~UnmanagedControl() {}

  void execCB(const behavior_tree_core::BTGoalConstPtr &goal);

  void setStatus(int status);
};