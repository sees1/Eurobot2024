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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <visualization_msgs/Marker.h>

#include <chrono>
#include <string>
#include <cmath>
#include <limits>

// for future work with multi-thread spinner
#include <thread>
#include <mutex>


class SafetyControl
{
// should be first!!! don't touch mazafaka
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<behavior_tree_core::BTAction> as_;
  std::string action_name_;
  behavior_tree_core::BTFeedback feedback_;
  behavior_tree_core::BTResult result_;

public:
  using PointT      = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

public:
  struct Cluster
  {
    std::pair<double, double> xy_cord;
    double stdev;
    int saturation;
  };

public:
  enum Status {RUNNING, SUCCESS, FAILURE};  // BT return status
  enum StopEvent    {WAIT, GOTONEXT};
  enum RuntimeEvent {HANDLE, IGNORE};

public:
  // rate parameters
  ros::Rate r;

  // main topics
  ros::Publisher  cmd_vel_pub;
  ros::Subscriber laser_sub;
  
  // debug topics
  ros::Publisher  out_cloud_pub;
  ros::Publisher  marker_pub;

  //topic parameters
  std::string laser_topic;
  std::string marker_topic;
  std::string out_cloud_topic;

  // velocities parameters
  geometry_msgs::Twist velocity;
  double duration;

  // cloud filtering parameters
  double padding;
  double min_point_x;
  double min_point_y;
  double max_point_x;
  double max_point_y;

  // pcl filters
  pcl::RadiusOutlierRemoval<PointT>       rad;
  pcl::VoxelGrid<PointT>                  vg;
  pcl::CropBox<PointT>                    cBox;
  pcl::EuclideanClusterExtraction<PointT> ec;

  // Cluster's geometric center's
  std::vector<Cluster> clusters_centroid_point;

  // Temporarly cloud
  PointCloudT::Ptr cloud;

  // Box filter's points
  Eigen::Vector4f minPoint, maxPoint;

  // debug flag
  bool debug;

  // is it time for stop?
  bool  safety_stop;
  float safety_distance;

  // manage next
  int    stop_event;  // convert to StopEvent
  int    lidar_event; // convert to RuntimeEvent

  // begin of time
  ros::WallTime begin;

  // maybe? why not?
  // mutex internal_mutex;

public:
  explicit SafetyControl(std::string name);
  ~SafetyControl() {}

  void execCB(const behavior_tree_core::BTGoal::ConstPtr &goal);
  void nearPointCB(const sensor_msgs::LaserScan::Ptr& msg);

  // support function
  bool isTimeToStop();
  void resetVelocity();
  void manageAfterStop();
  void manageRuntime();
  void publishZeroVel();

  void setStatus(int status);
};