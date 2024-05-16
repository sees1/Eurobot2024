#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_tree_core/BTAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Point.h>

#include <string>

class PlanPusher
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  std::string action_name_;
  behavior_tree_core::BTFeedback feedback_;
  behavior_tree_core::BTResult result_;

  geometry_msgs::Point target_point_;

public:
  enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status

public:
  explicit PlanPusher(std::string name);
  ~PlanPusher() {}

  void execCB(const behavior_tree_core::BTGoalConstPtr &goal);

  void setStatus(int status);
};