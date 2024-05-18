#include <button_check.h>

ButtonChecker::ButtonChecker(std::string name) :
    as_(nh_, name, boost::bind(&ButtonChecker::execute_callback, this, _1), false),
    action_name_(name),
    is_pressed(false)
{
    // start the action server (action in sense of Actionlib not BT action)
    as_.start();
    ROS_INFO("Condition Server Started");
    button_subscriber_ = nh_.subscribe("/start_engine", 1, &ButtonChecker::button_callback, this);
}

void ButtonChecker::execute_callback(const behavior_tree_core::BTGoalConstPtr &goal)
{
    ROS_INFO("im in goal loop in button_checker node");
    if (is_pressed)
    {
        set_status(SUCCESS);
    }
    else
    {
        set_status(FAILURE);
    }
}

void ButtonChecker::button_callback(const std_msgs::Bool::ConstPtr &msg)
{
    is_pressed = msg->data;
}

//  returns the status to the client (Behavior Tree)
void ButtonChecker::set_status(int status)
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
    ros::init(argc, argv, "button_checker");
    ROS_INFO(" Enum: %d", ButtonChecker::RUNNING);
    ROS_INFO(" condition Ready for Ticks");
    ButtonChecker bt_action(ros::this_node::getName());
    ros::spin();
    return 0;
}
