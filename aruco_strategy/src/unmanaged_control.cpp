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


UnmanagedControl::UnmanagedControl(std::string name) : nh_(""), pnh_("~"),
    as_(nh_, name, boost::bind(&UnmanagedControl::execCB, this, _1), false),
    action_name_(name)
{
  as_.start();

  cmd_vel_pub    = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  near_point_sub = nh_.subscribe("/nearest_point", 1, &UnmanagedControl::nearPointCB, this);

  pnh_.param<double>("velocity_x", velocity_x, 0.0);
  pnh_.param<double>("velocity_y", velocity_y, 0.0);
  pnh_.param<double>("duration",   duration,   0.0);  
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
  safety_stop = false;
  begin = ros::WallTime::now();
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

  ros::Rate r(10);
  ros::WallTime stop_time;

  // loop until time to stop or lidar stop
  while((ros::WallTime::now().toSec() - begin.toSec()) < duration &&
        !isTimeToStop())
  {
    cmd_vel_pub.publish(velocity);
    stop_time = ros::WallTime::now();
    r.sleep();
  }

  // new duration for next action
  duration = stop_time.toSec() - begin.toSec();

  geometry_msgs::Twist zero;
  zero.linear.x = 0.0;
  zero.linear.y = 0.0;
  zero.linear.z = 0.0;
  zero.angular.x = 0.0;
  zero.angular.y = 0.0;
  zero.angular.z = 0.0;

  begin = ros::WallTime::now();

  //
  while((ros::WallTime::now().toSec() - begin.toSec()) < 1 )
  {
    cmd_vel_pub.publish(zero);
    r.sleep();
  }

  while(true)
  {
    if (isTimeToStop() == true)
    {
      r.sleep();
      continue;
    }
    else
      break;
  }

  setStatus(SUCCESS);
}

// already x, a relative to laser frame
void UnmanagedControl::nearPointCB(const geometry_msgs::Point::ConstPtr& msg)
{
  // boost::scoped_lock<boost::mutex> lock(internal_mutex);
  double dist = std::sqrt(std::pow(msg->x, 2) + std::pow(msg->y, 2));

  if (dist < 0.4)
  {
    safety_stop = true;
  }
  else
  {
    safety_stop = false;
  }
}

bool UnmanagedControl::isTimeToStop()
{
  return safety_stop;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getName());
  ROS_INFO(" Enum: %d", UnmanagedControl::RUNNING);
  ROS_INFO(" Action Ready for Ticks");
  UnmanagedControl* bt_action = new UnmanagedControl(ros::this_node::getName());
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
