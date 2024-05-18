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
    action_name_(name), safety_("safety controller")
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
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    while((begin - std::chrono::steady_clock::now()) < std::chrono::duration<double>(duration) ||
           !safety_.isStop())
    {
      cmd_vel_pub_.publish(velocity);
      sleep(0.1);
    }

    setStatus(SUCCESS);
}

SafetyController::SafetyController(std::string name) : nh_s(name), safety_stop(false)
{
  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);
}

void SafetyController::safetyCB(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  if (front_cloud.data.size() != 0)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    // Convert PointCloud2 to Pcl msg
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(front_cloud, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);

    // PassThrough
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PassThrough<pcl::PointXYZI> pass;
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-10.0, 10.0);
    // pass.setNegative(true);
    // pass.filter(*cloud_filtered);

    // RadiusOutlierRemoval
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad;
    rad.setRadiusSearch(0.0581718236207962);
    rad.setMinNeighborsInRadius(6);
    rad.setKeepOrganized(true);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    rad.setInputCloud(cloud);
    rad.filter(*cloud_filtered);

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(0.01f, 0.01f, 0.01f); //(0.005f, 0.005f, 0.01f)
    pcl::PointCloud<pcl::PointXYZI>::Ptr vg_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    vg.setInputCloud(cloud_filtered);
    vg.filter(*vg_filtered);

    //CropBox 
    pcl::CropBox<pcl::PointXYZI> cBox;
    pcl::PointCloud<pcl::PointXYZI>::Ptr box_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    geometry_msgs::TransformStamped transform_box;

    try
    {
      transform_box = tf_buffer->lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_WARN("Rack detect: %s", ex.what());
    }
    Eigen::Vector3f translation;
    translation << transform_box.transform.translation.x, transform_box.transform.translation.y, transform_box.transform.translation.z;
        
    double padding = 0.2;
    double field_x_lenght = 3;
    double field_y_lenght = 2;

    Eigen::Vector4f minPoint, maxPoint;
    minPoint << -(field_x_lenght + 2 * padding) / 2, -(field_y_lenght + 2 * padding) / 2, -1, 1.0;
    maxPoint << (field_x_lenght + 2 * padding) / 2, (field_x_lenght + 2 * padding) / 2, -1, 1.0;

    cBox.setMin(minPoint);
    cBox.setMax(maxPoint);
    cBox.setTranslation(translation);
    // cBox.setRotation(rotation);
    cBox.setInputCloud(vg_filtered);
    cBox.filter(*box_filtered);

    // Euclidean Clustering
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    // Set Euclidean Clustering paramters
    ec.setClusterTolerance(0.06); // 15cm
    ec.setMinClusterSize(6);      // 5
    ec.setMaxClusterSize(25);     // 100
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> cluster_indices;
    if (box_filtered->size() > 0)
    {
      tree->setInputCloud(box_filtered);
      ec.setSearchMethod(tree);
      ec.setInputCloud(box_filtered);
      ec.extract(cluster_indices);
    }

    std::vector<SafetyController::Cluster> clusters_centroid_point;

    // ROS_INFO("Found %d clusters", cluster_indices.size());
    if (cluster_indices.size() > 0)
    {
      clusters_centroid_point.clear();

      sensor_msgs::PointCloud2 cloud_cluster_msg;
      using CPointIterator = std::vector<pcl::PointIndices>::const_iterator;

      for (CPointIterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);

        for (const auto &idx : it->indices)
          cloud_cluster->push_back((*box_filtered)[idx]); //*

        cloud_cluster->width    = cloud_cluster->size();
        cloud_cluster->height   = 1;
        cloud_cluster->is_dense = true;

        double x_mean = 0., y_mean = 0.;
        double i_mean = 0., i_max  = 0.;
        double i_min = std::numeric_limits<double>::max();
        double i_mean_2 = 0.;
        int sat = 0;

        for (pcl::PointXYZI point : cloud_cluster->points)
        {
          double inter_int = point.intensity / 100.;
          x_mean += point.x;
          y_mean += point.y;
          i_mean += inter_int;
          i_mean_2 += inter_int * inter_int;
          i_max = (inter_int - i_max) > 0. ? inter_int : i_max;
          i_min = (inter_int - i_min) > 0. ? i_min     : inter_int;
          sat++;
        }

        x_mean   = x_mean   / cloud_cluster->size();
        y_mean   = y_mean   / cloud_cluster->size();
        i_mean   = i_mean   / cloud_cluster->size();
        i_mean_2 = i_mean_2 / cloud_cluster->size();

        // This should take care of outliers giving optimal point within cluster close to robot and cluster center
        clusters_centroid_point.push_back(Cluster{std::make_pair(x_mean, y_mean), i_mean, i_max, i_min, (i_mean_2 - pow(i_mean, 2)), int(cloud_cluster->size())});
      }

      auto sortPredicat {[](Cluster const &a, Cluster const &b)
                          { 
                            return std::sqrt(std::pow(a.xy_cord.first, 2) + std::pow(a.xy_cord.second, 2)) <
                                   std::sqrt(std::pow(b.xy_cord.first, 2) + std::pow(b.xy_cord.second, 2)); 
                          }};

      //  xy_point is sorted by distance to robot
      std::sort(clusters_centroid_point.begin(), clusters_centroid_point.end(), sortPredicat);

      // calculateRackPosition(clusters_centroid_point);
      // cloud_filtered->clear();

      double pose_x = odom->pose.pose.position.x;
      double pose_y = odom->pose.pose.position.y;\

      std::vector<double> distances_to_obstacles(clusters_centroid_point.size());

      for(auto cluster : clusters_centroid_point)
      {
        double temp_dist = std::sqrt(std::pow(std::abs(cluster.xy_cord.first) - std::abs(pose_x), 2)
                                     + std::pow(std::abs(cluster.xy_cord.second) - std::abs(pose_y), 2));
        distances_to_obstacles.push_back(temp_dist);
      }

      double half_robot_rad = 0.1;
      double stop_dist = 0.4;

      for(size_t i = 0; i < distances_to_obstacles.size(); ++i)
      {
        if(distances_to_obstacles[i] < half_robot_rad + stop_dist)
        {  
          safety_stop = true;
          break;
        }
      }
    }
  }
}

void SafetyController::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  odom = msg;
}

bool SafetyController::isStop()
{
  return safety_stop;
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
