#include <ros/ros.h>

#include <aruco_opencv_msgs/ArucoDetection.h>
#include <aruco_opencv_msgs/MarkerSize.h>
#include <aruco_opencv_msgs/MarkerPose.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>


class ArucoRobotConvertor
{
public:
  ArucoRobotConvertor();
  ~ArucoRobotConvertor() {};

  void arucoCallback(const aruco_opencv_msgs::ArucoDetection::ConstPtr& msg);

  void CalcMainPose(const aruco_opencv_msgs::MarkerPose& msg);
  void CalcSlavePose(const aruco_opencv_msgs::MarkerPose& msg);

public:
  ros::NodeHandle nh_;

  int main_id;
  std::vector<int> slave_ids;
  std::vector<double> pose_covariance;
  bool handle_main_quat;
  geometry_msgs::Quaternion main_quaternion;

  geometry_msgs::PoseWithCovarianceStamped ground_true_pose;

  ros::Subscriber aruco_detect_sub;
  ros::Publisher  robot_pose_pub;
};

ArucoRobotConvertor::ArucoRobotConvertor() : slave_ids(4), handle_main_quat(false), pose_covariance(36)
{
  aruco_detect_sub = nh_.subscribe("aruco_detections", 1, &ArucoRobotConvertor::arucoCallback, this);
  robot_pose_pub   = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose_aruco", 1);

  nh_.getParam("main_id",         main_id);
  nh_.getParam("slave_ids",       slave_ids);
  nh_.getParam("pose_covariance", pose_covariance);

  main_quaternion.x = 0.0;
  main_quaternion.y = 0.0;
  main_quaternion.z = 0.0;
  main_quaternion.w = 1.0;
}

void ArucoRobotConvertor::arucoCallback(const aruco_opencv_msgs::ArucoDetection::ConstPtr& msg)
{
  for(auto marker : msg->markers)
  {
    if(marker.marker_id == main_id)
    {
      CalcMainPose(marker);
    }
    else if(marker.marker_id == slave_ids[0] ||
            marker.marker_id == slave_ids[1] ||
            marker.marker_id == slave_ids[2] ||
            marker.marker_id == slave_ids[3])
    {
      CalcSlavePose(marker);
    }
  }
}

void ArucoRobotConvertor::CalcMainPose(const aruco_opencv_msgs::MarkerPose& msg)
{
  ground_true_pose.pose.pose.position.x = msg.pose.position.x;
  ground_true_pose.pose.pose.position.y = msg.pose.position.y;
  ground_true_pose.pose.pose.position.z = 0;

  ground_true_pose.pose.pose.orientation.x = msg.pose.orientation.x;
  ground_true_pose.pose.pose.orientation.y = msg.pose.orientation.y;
  ground_true_pose.pose.pose.orientation.z = msg.pose.orientation.z;
  ground_true_pose.pose.pose.orientation.w = msg.pose.orientation.w;

  if (!handle_main_quat)
  {
    main_quaternion.x = msg.pose.orientation.x;
    main_quaternion.y = msg.pose.orientation.y;
    main_quaternion.z = msg.pose.orientation.z;
    main_quaternion.w = msg.pose.orientation.w;

    handle_main_quat = true;
  }

  std::memcpy(ground_true_pose.pose.covariance.data(), pose_covariance.data(), pose_covariance.size()*sizeof(double));
  // ground_true_pose.pose.covariance.data() = pose_covariance.data();
}

void ArucoRobotConvertor::CalcSlavePose(const aruco_opencv_msgs::MarkerPose& msg)
{
  ground_true_pose.pose.pose.position.x = msg.pose.position.z - 0.045;
  ground_true_pose.pose.pose.position.y = msg.pose.position.y;
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
  ArucoRobotConvertor aruco_robot_convertor;
  ros::spin();
  return 0;
}