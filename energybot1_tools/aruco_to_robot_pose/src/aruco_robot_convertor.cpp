#include <ros/ros.h>

#include <aruco_opencv_msgs/ArucoDetection.h>
#include <aruco_opencv_msgs/MarkerSize.h>
#include <aruco_opencv_msgs/MarkerPose.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class ArucoRobotConvertor
{
public:
  ros::NodeHandle nh_;

  int main_id;
  std::vector<int> slave_ids;
  std::vector<double> pose_covariance;
  bool handle_main_quat;
  geometry_msgs::Quaternion main_quaternion;

  geometry_msgs::PoseWithCovarianceStamped ground_true_pose;

  ros::Timer pose_calculate_timer;

  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;
  std::vector<geometry_msgs::TransformStamped> transforms;

  ros::Publisher  robot_pose_pub;

public:
  ArucoRobotConvertor();
  ~ArucoRobotConvertor() {};

  void arucoCallback(const ros::TimerEvent& event);

  void CalcMainPose(const geometry_msgs::TransformStamped& msg);
  void CalcSlavePose(const geometry_msgs::TransformStamped& msg);

};

ArucoRobotConvertor::ArucoRobotConvertor() : nh_("~"), slave_ids(4), handle_main_quat(false), pose_covariance(36),
                                             transforms(5)
{
  pose_calculate_timer = nh_.createTimer(ros::Duration(0.1), &ArucoRobotConvertor::arucoCallback, this);
  robot_pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_aruco", 1);

  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);

  nh_.getParam("main_id", main_id);
  nh_.param("slave_ids", slave_ids, slave_ids);
  nh_.param("pose_covariance", pose_covariance, pose_covariance);

  main_quaternion.x = 0.0;
  main_quaternion.y = 0.0;
  main_quaternion.z = 0.0;
  main_quaternion.w = 1.0;
}

void ArucoRobotConvertor::arucoCallback(const ros::TimerEvent& event)
{
  std::array<bool, 5> can_transform = {false, false, false, false, false};

  can_transform[0] = tf_buffer->canTransform("map", "marker_" + std::to_string(main_id), ros::Time(0));

  for(int i = 1; i <= slave_ids.size(); i++)
  {
    can_transform[i] = tf_buffer->canTransform("map", "marker_" + std::to_string(slave_ids[i]), ros::Time(0));
  }

  if(can_transform[0])
    transforms[0] = tf_buffer->lookupTransform("map", "marker_" + std::to_string(main_id), ros::Time(0));

  for(int i = 1; i <= slave_ids.size(); i++)
  {
    if (can_transform[i])
      transforms[0] = tf_buffer->lookupTransform("map", "marker_" + std::to_string(slave_ids[i]), ros::Time(0));
  }

  auto sortPredicate{[](const geometry_msgs::TransformStamped& lhs, const geometry_msgs::TransformStamped& rhs)
                    {
                      return lhs.header.stamp < rhs.header.stamp;
                    }};

  std::sort(transforms.begin(), transforms.end(), sortPredicate);

  if(transforms[0].child_frame_id == "marker_" + std::to_string(main_id))
  {
    CalcMainPose(transforms[0]);
  }
  else if(transforms[0].child_frame_id == "marker_" + std::to_string(slave_ids[0]) ||
          transforms[0].child_frame_id == "marker_" + std::to_string(slave_ids[1]) ||
          transforms[0].child_frame_id == "marker_" + std::to_string(slave_ids[2]) ||
          transforms[0].child_frame_id == "marker_" + std::to_string(slave_ids[3]))
  {
    CalcSlavePose(transforms[0]);
  }

  robot_pose_pub.publish(ground_true_pose);
}

void ArucoRobotConvertor::CalcMainPose(const geometry_msgs::TransformStamped& msg)
{
  if (!handle_main_quat)
  {
    double roll, pitch, yaw;
    tf2::Quaternion tfq(msg.transform.rotation.x,
                      msg.transform.rotation.y,
                      msg.transform.rotation.z,
                      msg.transform.rotation.w);

    tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);

    tfq.setRPY(0, 0, yaw);
    tfq = tfq.normalize();

    main_quaternion.x = tfq.x();
    main_quaternion.y = tfq.y();
    main_quaternion.z = tfq.z();
    main_quaternion.w = tfq.w();

    handle_main_quat = true;
  }
  
  ground_true_pose.header.frame_id = "map";
  ground_true_pose.header.stamp = ros::Time::now();
  ground_true_pose.pose.pose.position.x = msg.transform.translation.x;
  ground_true_pose.pose.pose.position.y = msg.transform.translation.y;
  ground_true_pose.pose.pose.position.z = 0;

  ground_true_pose.pose.pose.orientation.x = main_quaternion.x;
  ground_true_pose.pose.pose.orientation.y = main_quaternion.y;
  ground_true_pose.pose.pose.orientation.z = main_quaternion.z;
  ground_true_pose.pose.pose.orientation.w = main_quaternion.w;

  std::memcpy(ground_true_pose.pose.covariance.data(), pose_covariance.data(), pose_covariance.size()*sizeof(double));
  // ground_true_pose.pose.covariance.data() = pose_covariance.data();
}

void ArucoRobotConvertor::CalcSlavePose(const geometry_msgs::TransformStamped& msg)
{
  ground_true_pose.header.frame_id = "map";
  ground_true_pose.header.stamp = ros::Time::now();
  ground_true_pose.pose.pose.position.x = msg.transform.translation.x - 0.045;
  ground_true_pose.pose.pose.position.y = msg.transform.translation.y;
  ground_true_pose.pose.pose.position.z = 0;

  ground_true_pose.pose.pose.orientation.x = main_quaternion.x;
  ground_true_pose.pose.pose.orientation.y = main_quaternion.y;
  ground_true_pose.pose.pose.orientation.z = main_quaternion.z;
  ground_true_pose.pose.pose.orientation.w = main_quaternion.w;

  std::memcpy(ground_true_pose.pose.covariance.data(), pose_covariance.data(), pose_covariance.size()*sizeof(double));
  // ground_true_pose.pose.covariance.data() = pose_covariance.data();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_robot_convertor");
  ArucoRobotConvertor* aruco_robot_convertor = new ArucoRobotConvertor();
  ros::spin();
  return 0;
}