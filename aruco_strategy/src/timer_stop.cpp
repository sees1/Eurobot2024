#include <timer_stop.h>

TimerStop::TimerStop(std::string name) : pnh_("~"),
    as_(nh_, name, boost::bind(&TimerStop::execute_callback, this, _1), false),
    action_name_(name)
{
  // start the action server (action in sense of Actionlib not BT action)
  as_.start();

  pnh_.param<int>("duration", duration, 0);

  ROS_INFO("Action Server Started");
}

void TimerStop::execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
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

  sleep(duration);

  set_status(SUCCESS);
}

//  returns the status to the client (Behavior Tree)
void TimerStop::set_status(int status)
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
        ROS_INFO("Condition %s Succeeded", ros::this_node::getName().c_str() );
        break;
    case FAILURE:
        ROS_INFO("Condition %s Failed", ros::this_node::getName().c_str() );
        break;
    default:
        break;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    ROS_INFO(" Enum: %d", TimerStop::RUNNING);
    ROS_INFO(" condition Ready for Ticks");
    TimerStop bt_action(ros::this_node::getName());
    ros::spin();
    return 0;
}
