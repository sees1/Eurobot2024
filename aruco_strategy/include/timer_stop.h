#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_tree_core/BTAction.h>
#include <std_msgs/Bool.h>

#include <string>


class TimerStop
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // NodeHandle instance must be created before this line. Otherwise strange errors may occur.
  actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  behavior_tree_core::BTFeedback feedback_;
  behavior_tree_core::BTResult result_;

  int duration;

public:
  enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status

public:
  explicit TimerStop(std::string name);
  ~TimerStop(){ };

  void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal);

  void set_status(int status);
};