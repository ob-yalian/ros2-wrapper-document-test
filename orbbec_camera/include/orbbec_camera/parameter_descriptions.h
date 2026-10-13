#pragma once

#include <rclcpp/rclcpp.hpp>

namespace orbbec_camera {

// Device-specific defaults belong in descriptions, not in the ROS launch default.
rcl_interfaces::msg::ParameterDescriptor describeCameraParameter(const std::string& name);

}  // namespace orbbec_camera
