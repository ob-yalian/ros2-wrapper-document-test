#include "orbbec_camera/ob_camera_node.h"
#include "orbbec_camera/utils.h"

#include <stdexcept>

namespace orbbec_camera {

void OBCameraNode::addParameterReader(const std::string &name, Parameters::ParameterReader reader) {
  if (!node_->has_parameter(name)) return;
  parameters_->registerParameterReader(name, [this, reader = std::move(reader)] {
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    if (!is_camera_node_initialized_.load() || !is_running_.load()) {
      throw std::runtime_error("Camera is initializing or disconnected.");
    }
    return reader();
  });
}

rclcpp::ParameterValue OBCameraNode::readPropertyParameter(const std::string &name, OBPropertyID id,
                                                           rclcpp::ParameterType type) const {
  if (!device_->isPropertySupported(id, OB_PERMISSION_READ)) {
    throw std::runtime_error("Camera property is not readable: " + name);
  }
  if (id == OB_STRUCT_COLOR_AE_ROI || id == OB_STRUCT_DEPTH_AE_ROI) {
    OBRegionOfInterest roi{};
    uint32_t size = sizeof(roi);
    device_->getStructuredData(id, reinterpret_cast<uint8_t *>(&roi), &size);
    if (size != sizeof(roi)) throw std::runtime_error("Invalid camera ROI size.");
    const auto coordinate = name.substr(name.find_last_of('_') + 1);
    if (coordinate == "left") return rclcpp::ParameterValue(static_cast<int>(roi.x0_left));
    if (coordinate == "right") return rclcpp::ParameterValue(static_cast<int>(roi.x1_right));
    if (coordinate == "top") return rclcpp::ParameterValue(static_cast<int>(roi.y0_top));
    if (coordinate == "bottom") return rclcpp::ParameterValue(static_cast<int>(roi.y1_bottom));
    throw std::runtime_error("Unknown ROI coordinate: " + name);
  }
  if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
    if (id == OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT ||
        id == OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT) {
      return rclcpp::ParameterValue(device_->getIntProperty(id) != 0);
    }
    return rclcpp::ParameterValue(device_->getBoolProperty(id));
  }
  if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return rclcpp::ParameterValue(static_cast<double>(device_->getFloatProperty(id)));
  }
  const int value = device_->getIntProperty(id);
  if (id == OB_PROP_DISP_SEARCH_RANGE_MODE_INT) {
    if (value < 0 || value > 2) throw std::runtime_error("Unknown disparity range mode.");
    return rclcpp::ParameterValue(64 << value);
  }
  if (type == rclcpp::ParameterType::PARAMETER_STRING) {
    switch (id) {
      case OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT:
        if (value >= 0 && value <= 3)
          return rclcpp::ParameterValue(colorPowerLineFrequencyToString(value));
        break;
      case OB_PROP_DEVICE_PERFORMANCE_MODE_INT:
        if (value == 0 || value == 1)
          return rclcpp::ParameterValue(exposureRangeModeToString(value));
        break;
      case OB_PROP_INTRA_CAMERA_SYNC_REFERENCE_INT:
        if (value >= 0 && value <= 2)
          return rclcpp::ParameterValue(intraCameraSyncReferenceToString(value));
        break;
      case OB_PROP_DEVICE_AE_REFERENCE_INT:
        if (value == 0 || value == 1) return rclcpp::ParameterValue(value == 0 ? "depth" : "color");
        break;
      case OB_PROP_DEVICE_AE_STRATEGY_INT:
        if (value == 0 || value == 1)
          return rclcpp::ParameterValue(value == 0 ? "default" : "motion");
        break;
      case OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT:
        if (value == 0 || value == 1)
          return rclcpp::ParameterValue(value == 0 ? "FULL" : "OFFSET_80");
        break;
      default:
        break;
    }
    throw std::runtime_error("Unknown camera selector value: " + name);
  }
  return rclcpp::ParameterValue(value);
}

void OBCameraNode::setupParameterReaders() {
  for (const auto &binding : property_parameters_) {
    addParameterReader(binding.first, [this, name = binding.first, property = binding.second] {
      return readPropertyParameter(name, property.first, property.second);
    });
  }
  addParameterReader("enable_laser", [this] {
    if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ)) {
      return rclcpp::ParameterValue(device_->getIntProperty(OB_PROP_LASER_CONTROL_INT) != 0);
    }
    return readPropertyParameter("enable_laser", OB_PROP_LASER_BOOL,
                                 rclcpp::ParameterType::PARAMETER_BOOL);
  });
  addParameterReader("enable_firmware_log",
                     [this] { return rclcpp::ParameterValue(device_->isFirmwareLogEnabled()); });
  addParameterReader("monitor_poll_interval_sec", [this] {
    const auto milliseconds = device_->getMonitorPollInterval();
    if (milliseconds % 1000 != 0) {
      throw std::runtime_error("Monitor interval cannot be represented as whole seconds.");
    }
    return rclcpp::ParameterValue(static_cast<int64_t>(milliseconds / 1000));
  });
  const bool legacy_preset =
      isDepthWorkModeDevices(pid_) && node_->has_parameter("depth_work_mode");
  addParameterReader("device_preset", [this, legacy_preset] {
    if (legacy_preset) {
      return rclcpp::ParameterValue(std::string(device_->getCurrentDepthWorkMode().name));
    }
    const auto preset = device_->getCurrentPresetName();
    if (!preset) throw std::runtime_error("Device preset is unavailable.");
    return rclcpp::ParameterValue(std::string(preset));
  });
  addParameterReader("depth_work_mode", [this] {
    return rclcpp::ParameterValue(std::string(device_->getCurrentDepthWorkMode().name));
  });
  addParameterReader("device_preset_version", [this] {
    const auto version = device_->getCurrentPresetDepthWorkModeVersion();
    if (!version) throw std::runtime_error("Preset version is unavailable.");
    return rclcpp::ParameterValue(std::string(version));
  });
  addParameterReader("color_preset", [this] {
    const auto preset = device_->getCurrentColorPresetName();
    if (!preset) throw std::runtime_error("Color preset is unavailable.");
    return rclcpp::ParameterValue(std::string(preset));
  });
  addParameterReader("depth_precision", [this] {
    if (device_->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_READ)) {
      const auto level = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
      const auto value = depthPrecisionLevelToString(level);
      if (value == "unknown") throw std::runtime_error("Unknown depth precision level.");
      return rclcpp::ParameterValue(value);
    }
    if (!device_->isPropertySupported(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT,
                                      OB_PERMISSION_READ)) {
      throw std::runtime_error("Depth precision is not readable.");
    }
    return rclcpp::ParameterValue(
        std::to_string(device_->getFloatProperty(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT)) +
        "mm");
  });
  addParameterReader("disparity_to_depth_mode", [this] {
    if (readPropertyParameter("disparity_to_depth_mode", OB_PROP_DISPARITY_TO_DEPTH_BOOL,
                              rclcpp::ParameterType::PARAMETER_BOOL)
            .get<bool>()) {
      return rclcpp::ParameterValue("HW");
    }
    const bool software =
        readPropertyParameter("disparity_to_depth_mode", OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL,
                              rclcpp::ParameterType::PARAMETER_BOOL)
            .get<bool>();
    return rclcpp::ParameterValue(software ? "SW" : "disable");
  });
  addParameterReader("preset_resolution_config", [this] {
    OBPresetResolutionConfig config{};
    uint32_t size = sizeof(config);
    device_->getStructuredData(OB_STRUCT_PRESET_RESOLUTION_CONFIG,
                               reinterpret_cast<uint8_t *>(&config), &size);
    if (size != sizeof(config)) throw std::runtime_error("Invalid preset resolution size.");
    return rclcpp::ParameterValue(std::to_string(config.width) + "," +
                                  std::to_string(config.height) + "," +
                                  std::to_string(config.irDecimationFactor) + "," +
                                  std::to_string(config.depthDecimationFactor));
  });
  const std::vector<std::string> sync_names = {"sync_mode",
                                               "depth_delay_us",
                                               "color_delay_us",
                                               "trigger2image_delay_us",
                                               "trigger_out_delay_us",
                                               "trigger_out_enabled",
                                               "frames_per_trigger"};
  for (const auto &name : sync_names) {
    addParameterReader(name, [this, name]() -> rclcpp::ParameterValue {
      const auto config = device_->getMultiDeviceSyncConfig();
      if (name == "depth_delay_us") return rclcpp::ParameterValue(config.depthDelayUs);
      if (name == "color_delay_us") return rclcpp::ParameterValue(config.colorDelayUs);
      if (name == "trigger2image_delay_us")
        return rclcpp::ParameterValue(config.trigger2ImageDelayUs);
      if (name == "trigger_out_delay_us") return rclcpp::ParameterValue(config.triggerOutDelayUs);
      if (name == "trigger_out_enabled") return rclcpp::ParameterValue(config.triggerOutEnable);
      if (name == "frames_per_trigger") return rclcpp::ParameterValue(config.framesPerTrigger);
      for (const auto &mode : {"FREE_RUN", "STANDALONE", "PRIMARY", "SECONDARY", "SECONDARY_SYNCED",
                               "SOFTWARE_TRIGGERING", "HARDWARE_TRIGGERING", "GROUP_ACTIONS"}) {
        if (OBSyncModeFromString(mode) == config.syncMode) return rclcpp::ParameterValue(mode);
      }
      throw std::runtime_error("Unknown synchronization mode.");
    });
  }
  for (const auto &stream : IMAGE_STREAMS) {
    const auto prefix = stream_name_.at(stream);
    addParameterReader("enable_" + prefix, [this, stream] {
      return rclcpp::ParameterValue(enable_stream_.at(stream));
    });
    addParameterReader("enable_" + prefix + "_undistortion", [this, stream] {
      return rclcpp::ParameterValue(enable_undistortion_.at(stream));
    });
    for (const auto &field : {"width", "height", "fps", "format"}) {
      addParameterReader(prefix + "_" + field, [this, stream, field = std::string(field)] {
        const auto profile = stream_profile_.find(stream);
        if (!enable_stream_.at(stream) || profile == stream_profile_.end() || !profile->second) {
          throw std::runtime_error("No active stream profile.");
        }
        const auto video = profile->second->as<ob::VideoStreamProfile>();
        if (field == "width") return rclcpp::ParameterValue(static_cast<int>(video->width()));
        if (field == "height") return rclcpp::ParameterValue(static_cast<int>(video->height()));
        if (field == "fps") return rclcpp::ParameterValue(static_cast<int>(video->fps()));
        return rclcpp::ParameterValue(OBFormatToString(video->format()));
      });
    }
  }
  addParameterReader("point_cloud_decimation_filter_factor", [this] {
    return rclcpp::ParameterValue(point_cloud_decimation_filter_factor_.load());
  });
  addParameterReader("interleave_frame_enable", [this] {
    return readPropertyParameter("interleave_frame_enable", OB_PROP_FRAME_INTERLEAVE_ENABLE_BOOL,
                                 rclcpp::ParameterType::PARAMETER_BOOL);
  });
  for (const auto &name : {"disparity_offset_config", "offset_index0", "offset_index1"}) {
    addParameterReader(name, [this, name = std::string(name)] {
      OBDispOffsetConfig config{};
      uint32_t size = sizeof(config);
      device_->getStructuredData(OB_STRUCT_DISP_OFFSET_CONFIG, reinterpret_cast<uint8_t *>(&config),
                                 &size);
      if (size != sizeof(config)) throw std::runtime_error("Invalid disparity offset size.");
      if (name == "disparity_offset_config") return rclcpp::ParameterValue(config.enable != 0);
      return rclcpp::ParameterValue(
          static_cast<int>(name == "offset_index0" ? config.offset0 : config.offset1));
    });
  }
  for (const auto &name : device_param_names_) {
    if (name.find("hdr_index") == 0 || name.find("laser_index") == 0) {
      addParameterReader(name, []() -> rclcpp::ParameterValue {
        // Selecting another bank writes hardware state; a query must remain read-only.
        throw std::runtime_error("Interleave bank readback requires changing the device selector.");
      });
    }
  }
}
}  // namespace orbbec_camera
