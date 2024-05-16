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

#include <unmanaged_control.h>


UnmanagedControl::UnmanagedControl(std::string name) :
    as_(nh_, name, boost::bind(&UnmanagedControl::execCB, this, _1), false),
    action_name_(name)
{
  as_.start();

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  nh_.param<double>("velocity_x", velocity_x, 0.0);
  nh_.param<double>("velocity_y", velocity_y, 0.0);
  nh_.param<double>("duration",   duration,   0.0);  
}

// returns the status to the client (Behavior Tree)
void UnmanagedControl::setStatus(int status)
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

void UnmanagedControl::execCB(const behavior_tree_core::BTGoalConstPtr &goal)
{
    ROS_INFO("Starting Action");
    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested())
    {
      ROS_INFO("Action Halted");
      setStatus(FAILURE);
      // set the action state to preempted
      as_.setPreempted();
      return;
    }

    geometry_msgs::Twist velocity;
    velocity.linear.x = velocity_x;
    velocity.linear.y = velocity_y;
    velocity.linear.z = 0.0;
    velocity.angular.x = 0.0;
    velocity.angular.y = 0.0;
    velocity.angular.z = 0.0;
    
    cmd_vel_pub_.publish(velocity);
    sleep(duration);

    setStatus(SUCCESS);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "unmanaged_action");
    ROS_INFO(" Enum: %d", UnmanagedControl::RUNNING);
    ROS_INFO(" Action Ready for Ticks");
    UnmanagedControl bt_action(ros::this_node::getName());
    ros::spin();

    return 0;
}
