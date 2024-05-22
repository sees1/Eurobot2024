#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>
#include <std_msgs/Bool.h>

#include <string>


class ButtonChecker
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange errors may occur.
    actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
    std::string action_name_;
    ros::Subscriber button_subscriber_;
    // create messages that are used to published feedback/result
    behavior_tree_core::BTFeedback feedback_;
    behavior_tree_core::BTResult result_;

    bool is_pressed;
    bool shoot_once;

public:
    enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status

public:
    explicit ButtonChecker(std::string name);
    ~ButtonChecker(){ };

    void execute_callback(const behavior_tree_core::BTGoalConstPtr &goal);
    void button_callback(const std_msgs::Bool::ConstPtr &msg);
    void set_status(int status);
};