#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <behavior_tree_core/BTAction.h>
#include <std_msgs/Bool.h>

#include <energybot1_lift/InterfaceAction.h>
#include <energybot1_lift/InterfaceGoal.h>

#include <string>


class GripperMove
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // NodeHandle instance must be created before this line. Otherwise strange errors may occur.
  actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
  actionlib::SimpleActionClient<energybot1_lift::InterfaceAction> ac_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  behavior_tree_core::BTFeedback feedback_;
  behavior_tree_core::BTResult result_;

  int is_up;
  int height;

public:
  enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status

public:
  explicit GripperMove(std::string name);
  ~GripperMove(){ };

  void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal);

  void liftDoneCB(const actionlib::SimpleClientGoalState& state,
                  const energybot1_lift::InterfaceResult::ConstPtr& result);
  void liftFeedbackCB(const energybot1_lift::InterfaceFeedback::ConstPtr& feedback);
  void liftActiveCB();

  void set_status(int status);
};