#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Gazebo Transport includes
#include <gz/msgs.hh>
#include <gz/transport.hh>

class GazeboPoseToRosBridge : public rclcpp::Node {
public:
  GazeboPoseToRosBridge() : Node("perfect_localization_node") {
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Declare parameters
    this->declare_parameter("gz_topic", "/world/empty_world/pose/info");
    this->declare_parameter("target_entity_name", "prius");
    this->declare_parameter("parent_frame", "map");
    this->declare_parameter("child_frame", "base_link");
    this->declare_parameter("publish_amcl_pose", true);
    this->declare_parameter("amcl_topic", "/amcl_pose");
    this->declare_parameter("publish_tf", true);

    // Static transform parameters (for frame orientation correction)
    this->declare_parameter("static_transform_x", 0.0);
    this->declare_parameter("static_transform_y", 0.0);
    this->declare_parameter("static_transform_z", 0.0);
    this->declare_parameter("static_transform_roll", 0.0);
    this->declare_parameter("static_transform_pitch", 0.0);
    this->declare_parameter("static_transform_yaw",
                            -1.5708); // -90 degrees in radians

    // Get parameters
    gz_topic_ = this->get_parameter("gz_topic").as_string();
    target_entity_name_ = this->get_parameter("target_entity_name").as_string();
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    publish_amcl_ = this->get_parameter("publish_amcl_pose").as_bool();
    amcl_topic_ = this->get_parameter("amcl_topic").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    // Get static transform parameters
    double static_x = this->get_parameter("static_transform_x").as_double();
    double static_y = this->get_parameter("static_transform_y").as_double();
    double static_z = this->get_parameter("static_transform_z").as_double();
    double static_roll =
        this->get_parameter("static_transform_roll").as_double();
    double static_pitch =
        this->get_parameter("static_transform_pitch").as_double();
    double static_yaw = this->get_parameter("static_transform_yaw").as_double();

    // Create static transform
    tf2::Vector3 static_translation(static_x, static_y, static_z);
    tf2::Quaternion static_rotation;
    static_rotation.setRPY(static_roll, static_pitch, static_yaw);
    static_transform_.setOrigin(static_translation);
    static_transform_.setRotation(static_rotation);

    // Initialize AMCL pose publisher if needed
    if (publish_amcl_) {
      amcl_pose_pub_ =
          this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
              amcl_topic_, 10);
    }

    // Subscribe to Gazebo pose topic
    if (!gz_node_.Subscribe(gz_topic_, &GazeboPoseToRosBridge::OnPoseMsg,
                            this)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to subscribe to Gazebo topic: %s",
                   gz_topic_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Subscribed to Gazebo topic: %s",
                gz_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Looking for entity: %s",
                target_entity_name_.c_str());
    if (publish_tf_) {
      RCLCPP_INFO(this->get_logger(), "Publishing TF: %s -> %s",
                  parent_frame_.c_str(), child_frame_.c_str());
    }
    if (publish_amcl_) {
      RCLCPP_INFO(this->get_logger(), "Publishing fake AMCL pose on: %s",
                  amcl_topic_.c_str());
    }

    // Log static transform info
    tf2::Vector3 t = static_transform_.getOrigin();
    tf2::Quaternion r = static_transform_.getRotation();
    double roll, pitch, yaw;
    tf2::Matrix3x3(r).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(),
                "Static transform: [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f] rad",
                t.x(), t.y(), t.z(), roll, pitch, yaw);

    // Setup covariance matrix for AMCL (low uncertainty since this is ground
    // truth)
    amcl_covariance_.fill(0.0);
    amcl_covariance_[0] = 0.0001;  // x variance
    amcl_covariance_[7] = 0.0001;  // y variance
    amcl_covariance_[14] = 0.0001; // z variance
    amcl_covariance_[21] = 0.0003; // rot_x variance
    amcl_covariance_[28] = 0.0003; // rot_y variance
    amcl_covariance_[35] = 0.0003; // rot_z variance
  }

private:
  void OnPoseMsg(const gz::msgs::Pose_V &msg) {
    try {
      // Search for our target entity in the pose vector
      for (const auto &pose : msg.pose()) {

        // Check if this pose has a name field
        if (pose.name() == target_entity_name_) {
          ProcessPose(pose);
          return;
        }

        // Alternative: check header data for entity name
        if (pose.has_header()) {
          for (const auto &data : pose.header().data()) {
            if (data.key() == "entity_name" && data.value_size() > 0 &&
                data.value(0) == target_entity_name_) {
              ProcessPose(pose);
              return;
            }
          }
        }

        // Alternative: check id field if it matches a known pattern
        if (CheckEntityById(pose.id())) {
          ProcessPose(pose);
          return;
        }
      }

      // Entity not found - only log occasionally to avoid spam
      static int log_counter = 0;
      if (++log_counter % 100 == 0) { // Log every 100 messages
        RCLCPP_DEBUG(
            this->get_logger(),
            "Entity '%s' not found in pose message (logged every 100 msgs)",
            target_entity_name_.c_str());
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Error processing Gazebo pose message: %s", e.what());
    }
  }

  bool CheckEntityById(uint32_t id) {
    // You can maintain a mapping of entity IDs to names if needed
    // For now, just return false - implement this if you know the entity ID
    return false;
  }

  void ProcessPose(const gz::msgs::Pose &pose) {
    // Get current time for stamping
    auto now = this->get_clock()->now();

    // Publish TF transform if enabled
    if (publish_tf_) {
      PublishTfTransform(pose, now);
    }

    // Publish AMCL pose if enabled
    if (publish_amcl_) {
      PublishAmclPose(pose, now);
    }
  }

  void PublishTfTransform(const gz::msgs::Pose &pose,
                          const rclcpp::Time &timestamp) {
    try {
      // Convert Gazebo pose to tf2::Transform
      tf2::Vector3 gazebo_translation(pose.position().x(), pose.position().y(),
                                      pose.position().z());
      tf2::Quaternion gazebo_rotation(
          pose.orientation().x(), pose.orientation().y(),
          pose.orientation().z(), pose.orientation().w());
      tf2::Transform gazebo_transform(gazebo_rotation, gazebo_translation);

      // Apply static transform: corrected_pose = gazebo_pose * static_transform
      tf2::Transform corrected_transform = gazebo_transform * static_transform_;

      // Create and populate transform message
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = timestamp;
      transform_msg.header.frame_id = parent_frame_;
      transform_msg.child_frame_id = child_frame_;

      // Set corrected translation
      tf2::Vector3 corrected_translation = corrected_transform.getOrigin();
      transform_msg.transform.translation.x = corrected_translation.x();
      transform_msg.transform.translation.y = corrected_translation.y();
      transform_msg.transform.translation.z = corrected_translation.z();

      // Set corrected rotation
      tf2::Quaternion corrected_rotation = corrected_transform.getRotation();
      transform_msg.transform.rotation.x = corrected_rotation.x();
      transform_msg.transform.rotation.y = corrected_rotation.y();
      transform_msg.transform.rotation.z = corrected_rotation.z();
      transform_msg.transform.rotation.w = corrected_rotation.w();

      // Broadcast the transform
      tf_broadcaster_->sendTransform(transform_msg);

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error publishing TF transform: %s",
                   e.what());
    }
  }

  void PublishAmclPose(const gz::msgs::Pose &pose,
                       const rclcpp::Time &timestamp) {
    try {
      // Convert Gazebo pose to tf2::Transform
      tf2::Vector3 gazebo_translation(pose.position().x(), pose.position().y(),
                                      pose.position().z());
      tf2::Quaternion gazebo_rotation(
          pose.orientation().x(), pose.orientation().y(),
          pose.orientation().z(), pose.orientation().w());
      tf2::Transform gazebo_transform(gazebo_rotation, gazebo_translation);

      // Apply static transform
      tf2::Transform corrected_transform = gazebo_transform * static_transform_;

      auto amcl_pose =
          std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

      // Set header
      amcl_pose->header.stamp = timestamp;
      amcl_pose->header.frame_id = parent_frame_;

      // Set corrected pose
      tf2::Vector3 corrected_translation = corrected_transform.getOrigin();
      amcl_pose->pose.pose.position.x = corrected_translation.x();
      amcl_pose->pose.pose.position.y = corrected_translation.y();
      amcl_pose->pose.pose.position.z = corrected_translation.z();

      tf2::Quaternion corrected_rotation = corrected_transform.getRotation();
      amcl_pose->pose.pose.orientation.x = corrected_rotation.x();
      amcl_pose->pose.pose.orientation.y = corrected_rotation.y();
      amcl_pose->pose.pose.orientation.z = corrected_rotation.z();
      amcl_pose->pose.pose.orientation.w = corrected_rotation.w();

      // Set covariance (low uncertainty since this is ground truth)
      amcl_pose->pose.covariance = amcl_covariance_;

      // Publish the AMCL pose
      amcl_pose_pub_->publish(std::move(amcl_pose));

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error publishing AMCL pose: %s",
                   e.what());
    }
  }

private:
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
  std::string child_frame_;
  std::string amcl_topic_;
  bool publish_amcl_;
  bool publish_tf_;

  // AMCL covariance matrix (pre-computed for efficiency)
  std::array<double, 36> amcl_covariance_;

  // Static transform for frame correction (e.g., Prius frame to base_link)
  tf2::Transform static_transform_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<GazeboPoseToRosBridge>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
