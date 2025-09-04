#include "gz-to-ros-perfect-localization/perfect_localization.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<utils::GazeboPoseToRosBridge>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    // rclcpp::Node::SharedPtr node_;
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
