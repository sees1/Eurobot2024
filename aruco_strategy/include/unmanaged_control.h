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

class SafetyController
{
private:
  ros::NodeHandle nh_s;
  ros::Subscriber cloud_sub = nh_s.subscribe("/cloud", 1, &SafetyController::safetyCB, this);

  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;

  sensor_msgs::PointCloud2 front_cloud;
  nav_msgs::Odometry::ConstPtr odom;

  bool safety_stop;

public:
  struct Cluster
  {
    std::pair<double, double> xy_cord;
    double intensity;
    double i_max;
    double i_min;
    double stdev;
    int saturation;
  };

public:
  SafetyController(std::string name);
  ~SafetyController() {}

  void safetyCB(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void odomCB(const nav_msgs::Odometry::ConstPtr& msg);

  bool isStop();
};

class UnmanagedControl
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
  std::string action_name_;
  behavior_tree_core::BTFeedback feedback_;
  behavior_tree_core::BTResult result_;

  SafetyController safety_;

  ros::Publisher cmd_vel_pub_;

  double velocity_x;
  double velocity_y;
  double duration;

public:
  enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status

public:
  explicit UnmanagedControl(std::string name);
  ~UnmanagedControl() {}

  void execCB(const behavior_tree_core::BTGoalConstPtr &goal);

  void setStatus(int status);
};