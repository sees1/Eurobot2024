#include <gripper_move.h>

GripperMove::GripperMove(std::string name)
  : pnh_("~")
  , as_(nh_, name, boost::bind(&GripperMove::execute_callback, this, _1), false)
  , ac_("energybot1_lift", true)
  , action_name_(name)
{
  // start the action server (action in sense of Actionlib not BT action)
  as_.start();
  ac_.waitForServer();

  pnh_.param<int>("Height_signal", height, 0);
  pnh_.param<int>("isUp_signal", is_up, 0);

  ROS_INFO("Action Server Started");
}

void GripperMove::liftDoneCB(const actionlib::SimpleClientGoalState& state,
                             const energybot1_lift::InterfaceResult::ConstPtr& result)
{
  ROS_INFO("lift finished in state [%s]", state.toString().c_str());
  this->set_status(SUCCESS);
}
void GripperMove::liftFeedbackCB(const energybot1_lift::InterfaceFeedback::ConstPtr& feedback)
{
  ROS_INFO("Got Feedback from lift");
}

void GripperMove::liftActiveCB()
{
  ROS_INFO("Goal from lift just went active");
}

void GripperMove::execute_callback(const behavior_tree_core::BTGoalConstPtr& goal)
{
  ROS_INFO("Starting Action");
  // check that preempt has not been requested by the client
  if (as_.isPreemptRequested())
  {
    ROS_INFO("Action Halted");
    set_status(FAILURE);
    // set the action state to preempted
    as_.setPreempted();
    return;
  }

  energybot1_lift::InterfaceGoal goal_msg;
  goal_msg.Height = height;
  goal_msg.isUp = is_up;

  using namespace std::placeholders;

  ac_.sendGoal(goal_msg, std::bind(&GripperMove::liftDoneCB, this, std::placeholders::_1, std::placeholders::_2),
               std::bind(&GripperMove::liftActiveCB, this),
               std::bind(&GripperMove::liftFeedbackCB, this, std::placeholders::_1));

  ac_.waitForResult();
}

//  returns the status to the client (Behavior Tree)
void GripperMove::set_status(int status)
{
  // Set The feedback and result of BT.action
  feedback_.status = status;
  result_.status = feedback_.status;
  // publish the feedback
  as_.publishFeedback(feedback_);
  // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
  as_.setSucceeded(result_);

  switch (status)  // Print for convenience
  {
    case SUCCESS:
      ROS_INFO("Condition %s Succeeded", ros::this_node::getName().c_str());
      break;
    case FAILURE:
      ROS_INFO("Condition %s Failed", ros::this_node::getName().c_str());
      break;
    default:
      break;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getName());
  ROS_INFO(" Enum: %d", GripperMove::RUNNING);
  ROS_INFO(" condition Ready for Ticks");
  GripperMove bt_action(ros::this_node::getName());
  ros::spin();
  return 0;
}
