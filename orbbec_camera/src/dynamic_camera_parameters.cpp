#include "orbbec_camera/ob_camera_node.h"

#include <algorithm>
#include <limits>
#include <set>

namespace orbbec_camera {

bool OBCameraNode::isDynamicProperty(OBPropertyID property_id) const {
  switch (property_id) {
    case OB_PROP_COLOR_EXPOSURE_INT:
    case OB_PROP_COLOR_GAIN_INT:
    case OB_PROP_DEPTH_EXPOSURE_INT:
    case OB_PROP_DEPTH_GAIN_INT:
    case OB_PROP_IR_EXPOSURE_INT:
    case OB_PROP_IR_GAIN_INT:
    case OB_PROP_COLOR_WHITE_BALANCE_INT:
    case OB_PROP_COLOR_AUTO_EXPOSURE_BOOL:
    case OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL:
    case OB_PROP_IR_AUTO_EXPOSURE_BOOL:
    case OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL:
    case OB_PROP_COLOR_AE_MAX_EXPOSURE_INT:
    case OB_PROP_COLOR_AE_MAX_GAIN_INT:
    case OB_PROP_COLOR_BRIGHTNESS_INT:
    case OB_PROP_COLOR_ROI_BRIGHTNESS_INT:
    case OB_PROP_COLOR_SHARPNESS_INT:
    case OB_PROP_COLOR_GAMMA_INT:
    case OB_PROP_COLOR_SATURATION_INT:
    case OB_PROP_COLOR_CONTRAST_INT:
    case OB_PROP_COLOR_HUE_INT:
    case OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT:
    case OB_PROP_COLOR_ANTI_FLICKER_BOOL:
    case OB_PROP_COLOR_DENOISING_LEVEL_INT:
    case OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT:
    case OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT:
    case OB_PROP_IR_AE_MAX_EXPOSURE_INT:
    case OB_PROP_IR_BRIGHTNESS_INT:
    case OB_PROP_LASER_POWER_LEVEL_CONTROL_INT:
    case OB_PROP_MIN_DEPTH_INT:
    case OB_PROP_MAX_DEPTH_INT:
    case OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL:
    case OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL:
    case OB_PROP_HEARTBEAT_BOOL:
      try {
        // Read access is required to snapshot device values for failure recovery.
        return device_->isPropertySupported(property_id, OB_PERMISSION_READ_WRITE);
      } catch (const std::exception &e) {
        RCLCPP_DEBUG_STREAM(logger_, "Runtime property capability query failed: " << e.what());
        return false;
      }
    default:
      return false;
  }
}

rclcpp::ParameterValue OBCameraNode::readDynamicProperty(const DynamicProperty &property) const {
  if (property.type == rclcpp::ParameterType::PARAMETER_BOOL) {
    if (property.id == OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT ||
        property.id == OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT) {
      return rclcpp::ParameterValue(device_->getIntProperty(property.id) != 0);
    }
    return rclcpp::ParameterValue(device_->getBoolProperty(property.id));
  }
  return rclcpp::ParameterValue(device_->getIntProperty(property.id));
}

void OBCameraNode::writeDynamicProperty(const DynamicProperty &property,
                                        const rclcpp::ParameterValue &value) {
  if (property.type == rclcpp::ParameterType::PARAMETER_BOOL) {
    if (property.id == OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT ||
        property.id == OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT) {
      device_->setIntProperty(property.id, value.get<bool>() ? 1 : 0);
    } else {
      device_->setBoolProperty(property.id, value.get<bool>());
    }
  } else {
    device_->setIntProperty(property.id, static_cast<int>(value.get<int64_t>()));
  }
}

void OBCameraNode::setupDynamicParameters() {
  // ROS parameters represent requested configuration, including launch-only sentinels.
  // Do not replace configured values with device readings here.
  setupRuntimeControls();
  setupParameterReaders();
  setupFilterParameterReaders();
  std::map<std::string, rclcpp::ParameterType> types;
  for (const auto &entry : dynamic_properties_) {
    types.emplace(entry.first, entry.second.type);
  }
  for (const auto &entry : runtime_control_groups_) {
    types.emplace(entry.first, node_->get_parameter(entry.first).get_type());
  }
  parameters_->registerDynamicParameters(
      types, [this](const auto &values) { return setDynamicParameters(values); });
}

rcl_interfaces::msg::SetParametersResult OBCameraNode::setDynamicParameters(
    const std::vector<rclcpp::Parameter> &parameters) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  rcl_interfaces::msg::SetParametersResult result;
  if (!is_camera_node_initialized_.load() || !is_running_.load()) {
    result.reason = "Camera is initializing or disconnected.";
    return result;
  }
  std::set<std::string> groups;
  std::set<std::string> requested_names;
  for (const auto &parameter : parameters) {
    if (!requested_names.insert(parameter.get_name()).second) {
      result.reason = "Duplicate parameter: " + parameter.get_name();
      return result;
    }
    const auto group = runtime_control_groups_.find(parameter.get_name());
    groups.insert(group == runtime_control_groups_.end() ? "properties" : group->second);
  }
  if (groups.size() > 1) {
    result.reason = "Update one runtime control group at a time (properties, ROI, or one filter).";
    return result;
  }
  if (!groups.empty() && *groups.begin() != "properties") {
    try {
      return runtime_control_setters_.at(*groups.begin())(parameters);
    } catch (const std::exception &e) {
      result.reason = e.what();
      return result;
    }
  }
  std::vector<rclcpp::Parameter> ordered = parameters;
  // Disable automatic controls first, set manual values next, enable automatic controls last.
  const auto priority = [](const rclcpp::Parameter &value) {
    const auto &name = value.get_name();
    const bool automatic =
        name == "enable_color_auto_exposure" || name == "enable_depth_auto_exposure" ||
        name == "enable_ir_auto_exposure" || name == "enable_color_auto_white_balance";
    return automatic ? (value.as_bool() ? 2 : 0) : 1;
  };
  std::stable_sort(ordered.begin(), ordered.end(),
                   [&](const auto &a, const auto &b) { return priority(a) < priority(b); });
  std::vector<rclcpp::ParameterValue> previous;
  size_t attempted = 0;
  try {
    std::set<std::string> names;
    for (const auto &value : ordered) {
      const auto &name = value.get_name();
      if (!names.insert(name).second) {
        throw std::invalid_argument("Duplicate parameter: " + name);
      }
      const auto &property = dynamic_properties_.at(name);
      if (value.get_type() != property.type) {
        throw std::invalid_argument("Invalid type for " + name);
      }
      if (property.type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        const auto requested = value.as_int();
        const auto range = device_->getIntPropertyRange(property.id);
        if (requested == -1 || requested < range.min || requested > range.max ||
            requested < std::numeric_limits<int>::min() ||
            requested > std::numeric_limits<int>::max() ||
            (range.step > 0 && (requested - range.min) % range.step != 0)) {
          throw std::invalid_argument(name +
                                      " is outside the current SDK range/step; -1 is launch-only.");
        }
      }
      previous.push_back(readDynamicProperty(property));
    }
    const auto white_balance = names.count("color_white_balance") != 0;
    if (white_balance) {
      bool automatic = device_->getBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
      for (const auto &value : ordered) {
        if (value.get_name() == "enable_color_auto_white_balance") {
          automatic = value.as_bool();
        }
      }
      if (automatic) {
        throw std::invalid_argument(
            "Disable auto white balance before setting color_white_balance.");
      }
    }
    for (const auto &value : ordered) {
      // Include the failing call: a transport error can happen after the device accepted a write.
      ++attempted;
      writeDynamicProperty(dynamic_properties_.at(value.get_name()), value.get_parameter_value());
    }
    for (const auto &value : ordered) {
      dynamic_properties_.at(value.get_name()).update_cache(value.get_parameter_value());
    }
    result.successful = true;
    return result;
  } catch (const std::exception &e) {
    result.reason = e.what();
  } catch (...) {
    result.reason = "Unknown SDK parameter update failure.";
  }
  // SDK writes are not transactional. Restore attempted properties best-effort; ROS values
  // and explicit reconnect intent remain unchanged on failure.
  while (attempted > 0) {
    --attempted;
    const auto &name = ordered[attempted].get_name();
    try {
      writeDynamicProperty(dynamic_properties_.at(name), previous[attempted]);
    } catch (const std::exception &e) {
      result.reason += "; rollback failed for " + name + ": " + e.what();
      RCLCPP_ERROR_STREAM(logger_, result.reason);
    } catch (...) {
      result.reason += "; rollback failed for " + name;
      RCLCPP_ERROR_STREAM(logger_, result.reason);
    }
  }
  return result;
}

}  // namespace orbbec_camera
