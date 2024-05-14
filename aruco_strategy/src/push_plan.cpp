/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>

#include <string>

PlanPusher::PlanPusher(std::string name) :
    as_(nh_, name, boost::bind(&BTAction::execute_callback, this, _1), false),
    ac_("move_base", true),
    action_name_(name)
{
    as_.start();
    ac_.waitForServer();

    target_point_.x = nh_.getParam("target_point_x", 0.0);
    target_point_.y = nh_.getParam("target_poing_y", 0.0);
    target_point_.z = 0.0;
}

void PlanPusher::execCB(const behavior_tree_core::BTGoalConstPtr &goal)
{
    ROS_INFO("Starting Action");
    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested())
    {
      ROS_INFO("Action Halted")
      setStatus(FAILURE);
      // set the action state to preempted
      as_.setPreempted();
      break;
    }
    
    ROS_INFO("Executing Action")
    ROS_INFO("target_point_x and target_point_y are: %f, %f", target_point_.x, target_point_.y);
    ROS_INFO("Move_base subscriber initialized");

    // Build the destination message (geometry_msgs::PoseStamped)
    move_base_msgs::MoveBaseGoal goal;

    //Goal (geometry_msgs/PoseStamped)
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = target_point_.x;
    goal.target_pose.pose.position.y = target_point_.y;
    goal.target_pose.pose.position.z = target_point_.z;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ac_->sendGoal(goal, &PlanPublisher::moveBaseDoneCB, &PlanPublisher::moveBaseActiveCB, &PlanPublisher::moveBaseFeedbackCB);
}

void PlanPublisher::moveBaseDoneCB(const actionlib::SimpleClientGoalState& state,
                    const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
  ROS_INFO("move_base finished in state [%s]", state.toString().c_str());
  setStatus(SUCCESS);
}

void PlanPublisher::moveBaseFeedbackCB(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  ROS_INFO("Got Feedback from move_base");
}

void PlanPublisher::moveBaseActiveCB()
{
  ROS_INFO("Goal from move_base just went active");
}

// returns the status to the client (Behavior Tree)
void PlanPusher::setStatus(int status)
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
      ROS_INFO("Action %s Succeeded", ros::this_node::getName().c_str() );
      break;
    case FAILURE:
      ROS_INFO("Action %s Failed", ros::this_node::getName().c_str() );
      break;
    default:
      break;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "action");
    ROS_INFO(" Enum: %d", RUNNING);
    ROS_INFO(" Action Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());
    ros::spin();

    return 0;
}
