#include <iostream>
#include <string>
#include <cmath>

#include <ros/ros.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <process_pcl2/processConfig.h>

#include <limits>

#include <visualization_msgs/Marker.h>

class ProcessLaser
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
public:
  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

public:
  struct Cluster
  {
    std::pair<double, double> xy_cord;
    double stdev;
    int saturation;
  };

public:
  ProcessLaser();
  ~ProcessLaser() {};

  void dynamicCallBack(process_pcl2::processConfig &config, uint32_t level);

  void laserCB(const sensor_msgs::LaserScan::Ptr laser_msg);

private:
  // is debug needed?
  bool debug;

  double padding;
  double min_point_x;
  double min_point_y;
  double max_point_x;
  double max_point_y;

  std::string laser_topic;
  std::string out_cloud_topic;

  ros::Publisher  out_cloud_pub;
  ros::Publisher  near_point_pub;
  ros::Publisher  marker_pub;
  ros::Subscriber laser_sub;

  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;

  // pcl filters
  pcl::RadiusOutlierRemoval<PointT>       rad;
  pcl::VoxelGrid<PointT>                  vg;
  pcl::CropBox<PointT>                    cBox;
  pcl::EuclideanClusterExtraction<PointT> ec;

  // Cluster's geometric center
  std::vector<Cluster> clusters_centroid_point;

  // Input cloud
  PointCloudT::Ptr cloud;
  PointT invalid_point_;

  dynamic_reconfigure::Server<process_pcl2::processConfig> server;
  dynamic_reconfigure::Server<process_pcl2::processConfig>::CallbackType f;
};

ProcessLaser::ProcessLaser() : nh_(""), pnh_("~"), cloud(new PointCloudT)
{
  pnh_.param<double>("padding",        padding,       0.2);
  pnh_.param<double>("min_point_x",    min_point_x,     3.0);
  pnh_.param<double>("min_point_y",    min_point_y,     3.0);
  pnh_.param<double>("max_point_x",    max_point_x,     3.0);
  pnh_.param<double>("max_point_y",    max_point_y,     3.0);
  pnh_.param<bool>  ("debug",          debug,          true);

  pnh_.param<std::string>("laser_in",  laser_topic,     "/scan");
  pnh_.param<std::string>("cloud_out", out_cloud_topic, "/filtered_cloud");

  invalid_point_.x = std::numeric_limits<float>::quiet_NaN();
  invalid_point_.y = std::numeric_limits<float>::quiet_NaN();
  invalid_point_.z = 0.0;

  rad.setRadiusSearch(0.19581718236207962);
  rad.setMinNeighborsInRadius(1);
  rad.setKeepOrganized(true);

  vg.setLeafSize(0.07, 0.07, 0.07); //(0.005f, 0.005f, 0.01f)

  laser_sub      = nh_.subscribe(laser_topic, 10, &ProcessLaser::laserCB, this);
  out_cloud_pub  = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 10, false);
  marker_pub     = nh_.advertise<visualization_msgs::Marker>("/nearest_cluster_center", 1);
  near_point_pub = nh_.advertise<geometry_msgs::Point>("/nearest_point", 1);

  tf_buffer   = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);

  f = boost::bind(&ProcessLaser::dynamicCallBack, this, _1, _2);
  server.setCallback(f);
}

void ProcessLaser::dynamicCallBack(process_pcl2::processConfig &config, uint32_t level)
{
  return;
}

void ProcessLaser::laserCB(const sensor_msgs::LaserScan::Ptr laser_msg)
{
  cloud->points.resize(laser_msg->ranges.size());

  for (size_t i = 0; i < laser_msg->ranges.size(); ++i)
  {
    PointT& p   = cloud->points[i];
    float range = laser_msg->ranges[i];

    if (range > laser_msg->range_min && range < laser_msg->range_max)
    {
      float angle = laser_msg->angle_min + i*laser_msg->angle_increment;

      p.x = range * cos(angle);
      p.y = range * sin(angle);
      p.z = 0.0;
    }
    else
      p = invalid_point_;
  }

  cloud->width = laser_msg->ranges.size();
  cloud->height = 1;
  cloud->is_dense = false; //contains nans
  pcl_conversions::toPCL(laser_msg->header, cloud->header);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  PointCloudT::Ptr vg_filtered(new PointCloudT);
  vg.setInputCloud(cloud);
  vg.filter(*vg_filtered);

  // RadiusOutlierRemoval filter
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  rad.setInputCloud(vg_filtered);
  rad.filter(*cloud_filtered);

  // CropBox filter
  PointCloudT::Ptr box_filtered(new PointCloudT);

  geometry_msgs::TransformStamped transform_box;
  try
  {
    transform_box = tf_buffer->lookupTransform("laser", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException ex)
  {
    ROS_WARN("Process PCL: %s", ex.what());
  }

  Eigen::Vector3f translation;
  translation << transform_box.transform.translation.x, transform_box.transform.translation.y, transform_box.transform.translation.z;

  Eigen::Vector4f minPoint, maxPoint;
  minPoint <<  min_point_x - padding,  min_point_y - padding, -1, 1.0;
  maxPoint <<  max_point_x + padding,  max_point_y + padding,  1, 1.0;

  cBox.setMin(minPoint);
  cBox.setMax(maxPoint);
  cBox.setTranslation(translation);
  cBox.setInputCloud(cloud_filtered);
  cBox.filter(*box_filtered);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  std::vector<pcl::PointIndices> cluster_indices;

  tree->setInputCloud(box_filtered);
  ec.setClusterTolerance(0.1); // 15cm
  ec.setMinClusterSize(1);      // 5
  ec.setMaxClusterSize(5);     // 100
  ec.setSearchMethod(tree);
  ec.setInputCloud(box_filtered);
  ec.extract(cluster_indices);

  if (cluster_indices.size() >= 2)
  {
    clusters_centroid_point.clear();

    sensor_msgs::PointCloud2 cloud_cluster_msg;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      PointCloudT::Ptr cloud_cluster(new PointCloudT);

      for (const auto &idx : it->indices)
        cloud_cluster->push_back((*box_filtered)[idx]); //*

      cloud_cluster->width = cloud_cluster->size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      double x_mean = 0.0, y_mean = 0.0;
      for (PointT point : cloud_cluster->points)
      {
        x_mean += point.x;
        y_mean += point.y;
      }

      x_mean = x_mean / cloud_cluster->size();
      y_mean = y_mean / cloud_cluster->size();

      // This should take care of outliers giving optimal point within cluster close to robot and cluster center
      clusters_centroid_point.push_back(Cluster{std::make_pair(x_mean, y_mean), 0, int(cloud_cluster->size())});
    }

    auto sortPred {[](Cluster const &a, Cluster const &b)
                  {
                    return std::sqrt(std::pow(a.xy_cord.first, 2) + std::pow(a.xy_cord.second, 2)) < 
                           std::sqrt(std::pow(b.xy_cord.first, 2) + std::pow(b.xy_cord.second, 2)); 
                  }};

    //  xy_point is sorted by distance to robot
    std::sort(clusters_centroid_point.begin(), clusters_centroid_point.end(), sortPred);
  }

  if (clusters_centroid_point.size() > 0)
  {
    if (debug)
    {
      visualization_msgs::Marker centroid_center;
      centroid_center.header.frame_id = "laser";
      centroid_center.header.stamp = ros::Time::now();
      centroid_center.id = 1;
      centroid_center.type = visualization_msgs::Marker::CUBE;
      centroid_center.scale.x = 0.05;  
      centroid_center.scale.y = 0.05;
      centroid_center.scale.z = 0.05;
      centroid_center.color.g = 1.0;  
      centroid_center.color.a = 1.0; 
      centroid_center.pose.position.x = clusters_centroid_point.begin()->xy_cord.first; 
      centroid_center.pose.position.y = clusters_centroid_point.begin()->xy_cord.second;
      centroid_center.pose.position.z = 1.0;
      centroid_center.pose.orientation.x = 0.0;
      centroid_center.pose.orientation.y = 0.0;
      centroid_center.pose.orientation.z = 0.0;
      centroid_center.pose.orientation.w = 1.0;
      marker_pub.publish(centroid_center);
    }

    geometry_msgs::Point nearest_point;
    nearest_point.x = clusters_centroid_point.begin()->xy_cord.first;
    nearest_point.y = clusters_centroid_point.begin()->xy_cord.second;
    nearest_point.z = 0.0;

    near_point_pub.publish(nearest_point);
  }

  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*box_filtered, *cloud_msg);
  cloud_msg->header.stamp = ros::Time::now();

  out_cloud_pub.publish(cloud_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_pcl2");

  ProcessLaser* c = new ProcessLaser();

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}