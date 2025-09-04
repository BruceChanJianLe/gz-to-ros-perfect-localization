#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string_view>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Gazebo Transport includes
#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace utils {
class GazeboPoseToRosBridge : public rclcpp::Node {
public:
  GazeboPoseToRosBridge();
  ~GazeboPoseToRosBridge();

  void setStaticTransformX(const double value) { static_x_ = value; }
  void setStaticTransformY(const double value) { static_y_ = value; }
  void setStaticTransformZ(const double value) { static_z_ = value; }
  void setStaticTransformRoll(const double value) { static_roll_ = value; }
  void setStaticTransformPitch(const double value) { static_pitch_ = value; }
  void setStaticTransformYaw(const double value) { static_yaw_ = value; }
  void setParentFrame(const std::string_view frame_id) {
    parent_frame_ = frame_id;
  }
  void setChildFrame(const std::string_view frame_id) {
    child_frame_ = frame_id;
  }
  void setGZTopic(const std::string_view gz_topic) {
    if (gz_topic_.compare(gz_topic) != 0) {
      gz_node_.Unsubscribe(gz_topic_);
    }
    gz_topic_ = gz_topic;
    gz_node_.Subscribe(gz_topic_, &GazeboPoseToRosBridge::OnPoseMsg, this);
  }
  void setTargetEntity(const std::string_view entity_name) {
    target_entity_name_ = entity_name;
  }

  double getStaticTransformX() { return static_x_; }
  double getStaticTransformY() { return static_y_; }
  double getStaticTransformZ() { return static_z_; }
  double getStaticTransformRoll() { return static_roll_; }
  double getStaticTransformPitch() { return static_pitch_; }
  double getStaticTransformYaw() { return static_yaw_; }
  std::string getParentFrame() { return parent_frame_; }
  std::string getChildFrame() { return child_frame_; }
  std::string getTargetEntityName() { return target_entity_name_; }
  std::string getGZTopic() { return gz_topic_; }

  void updateStaticTransform();

private:
  void OnPoseMsg(const gz::msgs::Pose_V &msg);
  bool CheckEntityById([[maybe_unused]] uint32_t id);
  void ProcessPose(const gz::msgs::Pose &pose);
  void PublishTfTransform(const gz::msgs::Pose &pose,
                          const rclcpp::Time &timestamp);
  void PublishAmclPose();
  bool UpdateGZTopic();

  // Gazebo Transport
  gz::transport::Node gz_node_;

  // ROS 2 publishers and broadcasters
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      amcl_pose_pub_;

  // Parameters
  std::string gz_topic_;
  std::string target_entity_name_;
  std::string parent_frame_;
  std::string amcl_parent_frame_;
  std::string child_frame_;
  std::string amcl_topic_;

  double static_x_;
  double static_y_;
  double static_z_;
  double static_roll_;
  double static_pitch_;
  double static_yaw_;

  bool publish_amcl_;
  bool publish_tf_;

  // AMCL covariance matrix (pre-computed for efficiency)
  std::array<double, 36> amcl_covariance_;

  // Static transform for frame correction (e.g., robot frame to base_link)
  tf2::Transform static_transform_;

  // TF2 components
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
} // namespace utils
