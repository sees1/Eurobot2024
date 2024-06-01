#include <safety_control.h>

SafetyControl::SafetyControl(std::string name)
  : nh_("")
  , pnh_("~")
  , as_(nh_, name, boost::bind(&SafetyControl::execCB, this, _1), false)
  , cloud(new PointCloudT)
  , action_name_(name)
  , debug(true)
  , r(10)
  , safety_stop(false)
{
  as_.start();

  // topic naming param
  pnh_.param<std::string>("laser_topic", laser_topic, "scan");
  pnh_.param<std::string>("marker_topic", marker_topic, "nearest_point_marker");
  pnh_.param<std::string>("out_cloud_topic", out_cloud_topic, "filtered_cloud");

  // set velocity param
  pnh_.param<double>("velocity_x", velocity.linear.x, 0.0);
  pnh_.param<double>("velocity_y", velocity.linear.y, 0.0);
  velocity.linear.z = 0;
  velocity.angular.x = 0;
  velocity.angular.y = 0;
  pnh_.param<double>("velocity_ang", velocity.angular.z, 0.0);
  pnh_.param<double>("duration", duration, 0.0);

  // cloud filtering parameters
  pnh_.param<double>("padding", padding, 0.0);
  pnh_.param<double>("min_point_x", min_point_x, -1.0);
  pnh_.param<double>("min_point_y", min_point_y, -1.0);
  pnh_.param<double>("max_point_x", max_point_x, 1.0);
  pnh_.param<double>("max_point_y", max_point_y, 1.0);

  // safety distance parameter
  pnh_.param<float>("safety_distance", safety_distance, 0.4);

  // safety stop and runtime handle behaviour
  pnh_.param<int>("lidar_event", lidar_event, 0);
  pnh_.param<int>("stop_event", stop_event, 1);

  // filters settings
  rad.setRadiusSearch(0.19581718236207962);
  rad.setMinNeighborsInRadius(1);
  rad.setKeepOrganized(true);

  vg.setLeafSize(0.07, 0.07, 0.07);  //(0.005f, 0.005f, 0.01f)

  // publisher's
  cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  out_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 10);
  marker_pub = nh_.advertise<visualization_msgs::Marker>(marker_topic, 10);

  // single subscriber ()_()
  laser_sub = nh_.subscribe(laser_topic, 1, &SafetyControl::nearPointCB, this);
}

// returns the status to the client (Behavior Tree)
void SafetyControl::setStatus(int status)
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
      ROS_INFO("Action %s Succeeded", ros::this_node::getName().c_str());
      break;
    case FAILURE:
      ROS_INFO("Action %s Failed", ros::this_node::getName().c_str());
      break;
    default:
      break;
  }
}

void SafetyControl::resetVelocity()
{
  velocity.linear.x = 0.0;
  velocity.linear.y = 0.0;
  velocity.linear.z = 0.0;
  velocity.angular.x = 0.0;
  velocity.angular.y = 0.0;
  velocity.angular.z = 0.0;
}

void SafetyControl::manageAfterStop()
{
  switch (stop_event)
  {
    case StopEvent::WAIT: {
      while (true)
      {
        if (isTimeToStop() == true)
        {
          r.sleep();
          continue;
        }
        else
          break;
      }
      break;
    }
    case StopEvent::GOTONEXT: {
      // nothing do, just go to next action;
      break;
    }
  }
}

void SafetyControl::manageRuntime()
{
  switch (lidar_event)
  {
    case RuntimeEvent::HANDLE: {
      while ((ros::WallTime::now().toSec() - begin.toSec()) < duration && !isTimeToStop())
      {
        cmd_vel_pub.publish(velocity);
        r.sleep();
      }
      break;
    }
    case RuntimeEvent::IGNORE: {
      while ((ros::WallTime::now().toSec() - begin.toSec()) < duration)
      {
        cmd_vel_pub.publish(velocity);
        r.sleep();
      }
      break;
    }
  }
}

void SafetyControl::publishZeroVel()
{
  // set to zero
  resetVelocity();

  begin = ros::WallTime::now();
  while ((ros::WallTime::now().toSec() - begin.toSec()) < 1)
  {
    cmd_vel_pub.publish(velocity);
    r.sleep();
  }
}

void SafetyControl::execCB(const behavior_tree_core::BTGoalConstPtr& goal)
{
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

  // loop until time to stop or lidar stop. It should stop in any case
  manageRuntime();

  // stoping the robot
  publishZeroVel();

  // what robot should do after stop
  manageAfterStop();

  setStatus(SUCCESS);
}

void SafetyControl::nearPointCB(const sensor_msgs::LaserScan::Ptr& laser_msg)
{
  cloud->points.resize(laser_msg->ranges.size());

  for (size_t i = 0; i < laser_msg->ranges.size(); ++i)
  {
    PointT& p = cloud->points[i];
    float range = laser_msg->ranges[i];

    if (range > laser_msg->range_min && range < laser_msg->range_max)
    {
      float angle = laser_msg->angle_min + i * laser_msg->angle_increment;

      p.x = range * cos(angle);
      p.y = range * sin(angle);
      p.z = 0.0;
    }
    else
    {
      p.x = std::numeric_limits<float>::quiet_NaN();
      p.y = std::numeric_limits<float>::quiet_NaN();
      p.z = 0.0;
    }
  }

  cloud->width = laser_msg->ranges.size();
  cloud->height = 1;
  cloud->is_dense = false;  // contains nans
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
  minPoint << min_point_x - padding, min_point_y - padding, -1, 1.0;
  maxPoint << max_point_x + padding, max_point_y + padding, 1, 1.0;

  cBox.setMin(minPoint);
  cBox.setMax(maxPoint);
  cBox.setInputCloud(cloud_filtered);
  cBox.filter(*box_filtered);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  std::vector<pcl::PointIndices> cluster_indices;

  tree->setInputCloud(box_filtered);
  ec.setClusterTolerance(0.1);  // 15cm
  ec.setMinClusterSize(1);      // 5
  ec.setMaxClusterSize(5);      // 100
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

      for (const auto& idx : it->indices)
        cloud_cluster->push_back((*box_filtered)[idx]);  //*

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
      clusters_centroid_point.push_back(Cluster{ std::make_pair(x_mean, y_mean), 0, int(cloud_cluster->size()) });
    }

    auto sortPred{ [](Cluster const& a, Cluster const& b) {
      return std::sqrt(std::pow(a.xy_cord.first, 2) + std::pow(a.xy_cord.second, 2)) <
             std::sqrt(std::pow(b.xy_cord.first, 2) + std::pow(b.xy_cord.second, 2));
    } };

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

    float distance_to_nearest = std::sqrt(std::pow(clusters_centroid_point.begin()->xy_cord.first, 2) +
                                          std::pow(clusters_centroid_point.begin()->xy_cord.second, 2));

    if (distance_to_nearest < safety_distance)
    {
      safety_stop = true;
    }
    else
    {
      safety_stop = false;
    }

    if (debug)
    {
      geometry_msgs::Point nearest_point;
      nearest_point.x = clusters_centroid_point.begin()->xy_cord.first;
      nearest_point.y = clusters_centroid_point.begin()->xy_cord.second;
      nearest_point.z = 0.0;

      marker_pub.publish(nearest_point);
    }
  }

  if (debug)
  {
    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*box_filtered, *cloud_msg);
    cloud_msg->header.stamp = ros::Time::now();

    out_cloud_pub.publish(cloud_msg);
  }
}

bool SafetyControl::isTimeToStop()
{
  return safety_stop;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getName());
  ROS_INFO(" Enum: %d", SafetyControl::RUNNING);
  ROS_INFO(" Action Ready for Ticks");
  SafetyControl* bt_action = new SafetyControl(ros::this_node::getName());
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}
