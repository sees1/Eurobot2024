#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <behavior_tree_core/BTAction.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <chrono>
#include <string>
#include <cmath>

#include <thread>
#include <mutex>


class UnmanagedControl
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
  std::string action_name_;
  behavior_tree_core::BTFeedback feedback_;
  behavior_tree_core::BTResult result_;

  ros::Publisher  cmd_vel_pub;
  ros::Subscriber near_point_sub;

  double velocity_x;
  double velocity_y;
  double duration;

  // is it time for stop?
  bool  safety_stop;

  ros::WallTime begin;

  // mutex internal_mutex;

public:
  enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status

public:
  explicit UnmanagedControl(std::string name);
  ~UnmanagedControl() {}

  void execCB(const behavior_tree_core::BTGoalConstPtr &goal);
  void nearPointCB(const geometry_msgs::Point::ConstPtr &point);

  bool isTimeToStop();

  void setStatus(int status);
};