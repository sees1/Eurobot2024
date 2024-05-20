#include <iostream>
#include <string>

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
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <process_pcl2/processConfig.h>

class ProcessCloud
{
public:
  ProcessCloud();
  ~ProcessCloud() {};

  void dynamicCallBack(process_pcl2::processConfig &config, uint32_t level);

  void cloudCB(const sensor_msgs::PointCloud2 &front_cloud);

private:
  ros::NodeHandle nh_;

  double padding;
  double field_x_lenght;
  double field_y_lenght;

  std::string cloud_topic;
  std::string out_cloud_topic;

  ros::Publisher  out_pub;
  ros::Subscriber cloud_sub;

  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;

  // pcl filters
  pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad;
  pcl::VoxelGrid<pcl::PointXYZI>            vg;
  pcl::CropBox<pcl::PointXYZI>              cBox;

  dynamic_reconfigure::Server<process_pcl2::processConfig> server;
  dynamic_reconfigure::Server<process_pcl2::processConfig>::CallbackType f;

  sensor_msgs::PointCloud2 pcl2_front;
};

ProcessCloud::ProcessCloud()
{
  nh_.param<double>("padding",        padding,       0.2);
  nh_.param<double>("field_x_lenght", field_x_lenght,  4);
  nh_.param<double>("field_y_lenght", field_y_lenght,  2);

  nh_.param<std::string>("cloud_in",  cloud_topic,     "/cloud");
  nh_.param<std::string>("cloud_out", out_cloud_topic, "/filtered_cloud");

  rad.setRadiusSearch(0.0581718236207962);
  rad.setMinNeighborsInRadius(6);
  rad.setKeepOrganized(true);

  vg.setLeafSize(0.01f, 0.01f, 0.01f); //(0.005f, 0.005f, 0.01f)

  cloud_sub = nh_.subscribe(cloud_topic, 10, &ProcessCloud::cloudCB, this);
  out_pub   = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 10, false);

  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);

  f = boost::bind(&ProcessCloud::dynamicCallBack, this, _1, _2);
  server.setCallback(f);
}

void ProcessCloud::dynamicCallBack(process_pcl2::processConfig &config, uint32_t level)
{
  return;
}

void ProcessCloud::cloudCB(const sensor_msgs::PointCloud2 &cloud_msg)
{
  if (cloud_msg.data.size() != 0)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCLPointCloud2 pcl_pc;

    pcl_conversions::toPCL(cloud_msg, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);

    // RadiusOutlierRemoval
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    rad.setInputCloud(cloud_filtered);
    rad.filter(*cloud);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::PointCloud<pcl::PointXYZI>::Ptr vg_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    vg.setInputCloud(cloud_filtered);
    vg.filter(*vg_filtered);

    //CropBox 
    pcl::PointCloud<pcl::PointXYZI>::Ptr box_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    geometry_msgs::TransformStamped transform_box;
    try
    {
      transform_box = tf_buffer->lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_WARN("Process Cloud node: %s", ex.what());
    }

    Eigen::Vector3f translation;
    translation << transform_box.transform.translation.x, transform_box.transform.translation.y, transform_box.transform.translation.z;

    Eigen::Vector4f minPoint, maxPoint;
    minPoint << -(field_x_lenght + 2 * padding) / 2, -(field_y_lenght + 2 * padding) / 2, -1, 1.0;
    maxPoint <<  (field_x_lenght + 2 * padding) / 2,  (field_x_lenght + 2 * padding) / 2, -1, 1.0;

    cBox.setMin(minPoint);
    cBox.setMax(maxPoint);
    cBox.setTranslation(translation);
    cBox.setInputCloud(vg_filtered);
    cBox.filter(*box_filtered);

    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*box_filtered, *cloud_msg);
    cloud_msg->header.stamp = ros::Time::now();

    out_pub.publish(cloud_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_pcl2");

  ProcessCloud c;

  sensor_msgs::PointCloud2 cloud_out;

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}