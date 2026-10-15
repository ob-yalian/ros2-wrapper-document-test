#include "orbbec_camera/ob_camera_node.h"
#include "orbbec_camera/utils.h"
#include "orbbec_camera/parameter_descriptions.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <set>

namespace orbbec_camera {
namespace {
using ParameterResult = rcl_interfaces::msg::SetParametersResult;

ParameterResult runtimeResult(bool success, const std::string &message = "") {
  ParameterResult result;
  result.successful = success;
  result.reason = message;
  return result;
}

bool supportsImageDecimation(OBFormat format) {
  switch (format) {
    case OB_FORMAT_Y16:
    case OB_FORMAT_Z16:
    case OB_FORMAT_Y8:
    case OB_FORMAT_YUYV:
    case OB_FORMAT_UYVY:
    case OB_FORMAT_RGB:
    case OB_FORMAT_BGR:
    case OB_FORMAT_RGBA:
    case OB_FORMAT_BGRA:
    case OB_FORMAT_Y12C4:
      return true;
    default:
      return false;
  }
}

struct RuntimeFilter {
  std::string name;
  std::string enable_parameter;
  // ROS parameter name -> SDK/service config name, in legacy positional order.
  std::vector<std::pair<std::string, std::string>> config;
};

const std::vector<RuntimeFilter> &runtimeFilters() {
  static const std::vector<RuntimeFilter> filters = {
      {"DecimationFilter", "enable_decimation_filter", {{"decimation_filter_scale", "scale"}}},
      {"HDRMerge",
       "enable_hdr_merge",
       {{"hdr_merge_exposure_1", "exposure_1"},
        {"hdr_merge_gain_1", "gain_1"},
        {"hdr_merge_exposure_2", "exposure_2"},
        {"hdr_merge_gain_2", "gain_2"}}},
      {"SequenceIdFilter", "enable_sequence_id_filter", {{"sequence_id_filter_id", "sequenceid"}}},
      {"ThresholdFilter",
       "enable_threshold_filter",
       {{"threshold_filter_min", "min"}, {"threshold_filter_max", "max"}}},
      {"SpatialAdvancedFilter",
       "enable_spatial_filter",
       {{"spatial_filter_alpha", "alpha"},
        {"spatial_filter_diff_threshold", "disp_diff"},
        {"spatial_filter_magnitude", "magnitude"},
        {"spatial_filter_radius", "radius"}}},
      {"SpatialFastFilter",
       "enable_spatial_fast_filter",
       {{"spatial_fast_filter_radius", "radius"}}},
      {"SpatialModerateFilter",
       "enable_spatial_moderate_filter",
       {{"spatial_moderate_filter_diff_threshold", "disp_diff"},
        {"spatial_moderate_filter_magnitude", "magnitude"},
        {"spatial_moderate_filter_radius", "radius"}}},
      {"TemporalFilter",
       "enable_temporal_filter",
       {{"temporal_filter_diff_threshold", "diff_scale"}, {"temporal_filter_weight", "weight"}}},
      {"HoleFillingFilter",
       "enable_hole_filling_filter",
       {{"hole_filling_filter_mode", "hole_filling_mode"}}},
      {"NoiseRemovalFilter",
       "enable_noise_removal_filter",
       {{"noise_removal_filter_min_diff", "min_diff"},
        {"noise_removal_filter_max_size", "max_size"}}},
      {"HardwareNoiseRemovalFilter",
       "enable_hardware_noise_removal_filter",
       {{"hardware_noise_removal_filter_threshold", "threshold"}}},
      {"DispOutliersFilter",
       "enable_disp_outliers_filter",
       {{"disp_outliers_filter_search_mode", "search_mode"}}},
      {"FalsePositiveFilter", "enable_false_positive_filter", {}},
      {"EdgeNoiseRemovalFilter", "enable_edge_noise_removal_filter", {}},
      {"MgcNoiseRemovalFilter", "enable_mgc_noise_removal_filter", {}},
      {"LutNoiseRemovalFilter", "enable_lut_noise_removal_filter", {}},
      {"DisparityTransform", "enable_disparity_to_depth", {}},
      {"EnhancedDepthFilter",
       "enable_enhanced_depth",
       {{"enhanced_depth_confidence_threshold", "confidence_threshold"}}},
  };
  return filters;
}

int runtimeInt(const rclcpp::Parameter &value) {
  const auto requested = value.as_int();
  if (requested == -1 || requested < std::numeric_limits<int32_t>::min() ||
      requested > std::numeric_limits<int32_t>::max()) {
    throw std::invalid_argument(value.get_name() + ": -1 is launch-only; value must fit int32.");
  }
  return static_cast<int>(requested);
}

std::string filterConfigValue(const rclcpp::Parameter &value) {
  if ((value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER && value.as_int() == -1) ||
      (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE && value.as_double() == -1.0) ||
      (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING && value.as_string().empty())) {
    throw std::invalid_argument(value.get_name() + ": skip values are launch-only.");
  }
  if (value.get_name() == "hole_filling_filter_mode") {
    if (value.as_string() != "FILL_TOP" && value.as_string() != "FILL_NEAREST" &&
        value.as_string() != "FILL_FAREST") {
      throw std::invalid_argument(
          "hole_filling_filter_mode expects FILL_TOP, FILL_NEAREST or FILL_FAREST.");
    }
    return std::to_string(static_cast<int>(holeFillingModeFromString(value.as_string())));
  }
  return value.get_type() == rclcpp::ParameterType::PARAMETER_STRING ? value.as_string()
                                                                     : value.value_to_string();
}
}  // namespace

bool isRuntimeCameraParameter(const std::string &name) {
  static const std::set<std::string> controls = {
      "ae_reference_stream",
      "ae_strategy",
      "angular_vel_cov",
      "cloud_frame_id",
      "color_powerline_freq",
      "disparity_range_mode",
      "disparity_search_offset",
      "enable_colored_point_cloud",
      "enable_ir_long_exposure",
      "enable_laser",
      "enable_ldp",
      "enable_point_cloud",
      "enable_ptp_config",
      "linear_accel_cov",
      "max_save_images_count",
      "ordered_pc",
      "point_cloud_decimation_filter_factor",
      "point_cloud_qos",
      "publish_tf",
      "sync_io_voltage_level",
      "tf_publish_rate",
  };
  if (controls.count(name) != 0 || name.find("color_ae_roi_") == 0 ||
      name.find("depth_ae_roi_") == 0)
    return true;
  for (const auto &filter : runtimeFilters()) {
    if (filter.enable_parameter == name) return true;
    for (const auto &config : filter.config)
      if (config.first == name) return true;
  }
  for (const std::string stream :
       {"color", "left_color", "right_color", "depth", "ir", "left_ir", "right_ir"}) {
    if (name == stream + "_mirror" || name == stream + "_flip" || name == stream + "_rotation")
      return true;
  }
  for (const std::string stream : {"color", "left_color", "right_color"}) {
    if (name == "enable_" + stream + "_decimation_filter" ||
        name == stream + "_decimation_filter_scale" || name == stream + "_frame_queue_max_frames")
      return true;
  }
  for (const std::string stream : {"left_ir", "right_ir"}) {
    if (name == "enable_" + stream + "_sequence_id_filter" ||
        name == stream + "_sequence_id_filter_id")
      return true;
  }
  return false;
}

void OBCameraNode::addRuntimeControl(const std::vector<std::string> &names,
                                     Parameters::BatchSetter setter) {
  if (names.empty()) {
    return;
  }
  for (const auto &name : names) {
    if (node_->has_parameter(name)) {
      runtime_control_groups_.emplace(name, names.front());
    }
  }
  runtime_control_setters_.emplace(names.front(), std::move(setter));
}

std::vector<rclcpp::Parameter> OBCameraNode::roiServiceParameters(
    const std::shared_ptr<SetArrays::Request> &request,
    const stream_index_pair &stream_index) const {
  if (!request || request->data_param.size() < 4) {
    return {};
  }
  if (std::any_of(request->data_param.begin(), request->data_param.begin() + 4, [](float value) {
        return !std::isfinite(value) || value < std::numeric_limits<int32_t>::min() ||
               value > std::numeric_limits<int32_t>::max();
      })) {
    return {};
  }
  const bool color =
      stream_index == COLOR || stream_index == COLOR_LEFT || stream_index == COLOR_RIGHT;
  const std::string prefix = color ? "color_ae_roi_" : "depth_ae_roi_";
  return {rclcpp::Parameter(prefix + "left", static_cast<int64_t>(request->data_param[0])),
          rclcpp::Parameter(prefix + "right", static_cast<int64_t>(request->data_param[1])),
          rclcpp::Parameter(prefix + "top", static_cast<int64_t>(request->data_param[2])),
          rclcpp::Parameter(prefix + "bottom", static_cast<int64_t>(request->data_param[3]))};
}

std::vector<rclcpp::Parameter> OBCameraNode::filterServiceParameters(
    const SetFilter::Request &request) const {
  const auto name = normalizeDepthFilterName(request.filter_name);
  const auto spec = std::find_if(runtimeFilters().begin(), runtimeFilters().end(),
                                 [&](const auto &filter) { return filter.name == name; });
  if (spec == runtimeFilters().end()) {
    return {};
  }
  std::vector<rclcpp::Parameter> result = {
      rclcpp::Parameter(spec->enable_parameter, request.filter_enable)};
  const auto append = [&](const std::string &parameter_name, const std::string &text) {
    if (!node_->has_parameter(parameter_name)) {
      return;
    }
    try {
      const auto type = node_->get_parameter(parameter_name).get_type();
      if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        const double value = std::stod(text);
        if (std::isfinite(value) && value >= std::numeric_limits<int32_t>::min() &&
            value <= std::numeric_limits<int32_t>::max()) {
          result.emplace_back(parameter_name, static_cast<int64_t>(value));
        }
      } else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.emplace_back(parameter_name, std::stod(text));
      } else {
        if (parameter_name == "hole_filling_filter_mode") {
          const auto value = std::stod(text);
          if (!std::isfinite(value) || value < 0 || value > 2) return;
          const int mode = static_cast<int>(value);
          if (mode == 0) result.emplace_back(parameter_name, "FILL_TOP");
          if (mode == 1) result.emplace_back(parameter_name, "FILL_NEAREST");
          if (mode == 2) result.emplace_back(parameter_name, "FILL_FAREST");
        } else {
          result.emplace_back(parameter_name, text);
        }
      }
    } catch (const std::exception &) {
      // Parsing/validation belongs to the original service implementation.
    }
  };
  for (size_t index = 0; index < request.filter_param.size() && index < spec->config.size();
       ++index) {
    append(spec->config[index].first, std::to_string(request.filter_param[index]));
  }
  for (const auto &config : request.filter_config) {
    for (const auto &binding : spec->config) {
      const bool alias =
          (binding.second == "scale" && config.name == "decimate") ||
          (binding.second == "disp_diff" && config.name == "diff_threshold") ||
          (binding.second == "diff_scale" && config.name == "diff_threshold") ||
          (binding.second == "hole_filling_mode" && config.name == "mode") ||
          (binding.second == "sequenceid" && (config.name == "id" || config.name == "sequence_id" ||
                                              config.name == "sequence_id_filter_id"));
      if (binding.second == config.name || alias) {
        append(binding.first, config.value);
      }
    }
  }
  return result;
}

void OBCameraNode::setupRuntimeControls() {
  if (!static_tf_broadcaster_) {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node_);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);
  }
  const auto add_int = [this](const std::string &name, auto apply) {
    addRuntimeControl({name}, [apply](const auto &values) {
      auto request = std::make_shared<SetInt32::Request>();
      auto response = std::make_shared<SetInt32::Response>();
      request->data = runtimeInt(values.front());
      apply(request, response);
      return runtimeResult(response->success, response->message);
    });
  };
  const auto add_bool = [this](const std::string &name, auto apply) {
    addRuntimeControl({name}, [apply](const auto &values) {
      auto request = std::make_shared<SetBool::Request>();
      auto response = std::make_shared<SetBool::Response>();
      request->data = values.front().as_bool();
      apply(request, response);
      return runtimeResult(response->success, response->message);
    });
  };
  const auto add_string = [this](const std::string &name, auto apply) {
    addRuntimeControl({name}, [apply](const auto &values) {
      auto request = std::make_shared<SetString::Request>();
      auto response = std::make_shared<SetString::Response>();
      request->data = values.front().as_string();
      apply(request, response);
      return runtimeResult(response->success, response->message);
    });
  };
  for (const auto &stream : IMAGE_STREAMS) {
    if (sensors_.count(stream) == 0) {
      continue;
    }
    const auto &name = stream_name_.at(stream);
    const auto check_orientation = [this, stream](bool enabled) {
      // Gemini 330 SDK transforms pass compressed color frames through unchanged.
      return !enabled || !isGemini330SeriesPID(pid_) || stream != COLOR ||
             stream_profile_.at(stream)->format() != OB_FORMAT_MJPG;
    };
    add_bool(
        name + "_mirror", [this, stream, check_orientation](const auto &request, auto &response) {
          if (!check_orientation(request->data)) {
            response->message = "Color orientation requires an uncompressed stream on this device.";
            return;
          }
          applyMirror(request, response, stream);
        });
    add_bool(
        name + "_flip", [this, stream, check_orientation](const auto &request, auto &response) {
          if (!check_orientation(request->data)) {
            response->message = "Color orientation requires an uncompressed stream on this device.";
            return;
          }
          applyFlip(request, response, stream);
        });
    add_int(
        name + "_rotation", [this, stream, check_orientation](const auto &request, auto &response) {
          if (request->data != 0 && request->data != 90 && request->data != 180 &&
              request->data != 270) {
            response->message = "Rotation expects 0, 90, 180 or 270 degrees.";
            return;
          }
          if (!check_orientation(request->data != 0)) {
            response->message = "Color orientation requires an uncompressed stream on this device.";
            return;
          }
          applyRotation(request, response, stream);
        });
  }
  add_bool("enable_laser", [this](const auto &request, auto &response) {
    applyLaserEnable(nullptr, request, response);
  });
  add_bool("enable_ldp", [this](const auto &request, auto &response) {
    applyLdpEnable(nullptr, request, response);
  });
  add_bool("enable_ptp_config", [this](const auto &request, auto &response) {
    applyPtpConfig(nullptr, request, response);
  });
  add_bool("enable_ir_long_exposure",
           [this](const auto &request, auto &response) { applyIRLongExposure(request, response); });
  add_int("point_cloud_decimation_filter_factor", [this](const auto &request, auto &response) {
    applyPointCloudDecimation(request, response);
  });
  add_int("disparity_search_offset", [this](const auto &request, auto &response) {
    applyDisparitySearchOffset(request, response);
  });
  add_int("sync_io_voltage_level", [this](const auto &request, auto &response) {
    applySyncIoVoltageLevel(request, response);
  });
  add_int("disparity_range_mode", [this](const auto &request, auto &response) {
    applyDisparityRangeMode(request, response);
  });
  add_string("ae_reference_stream", [this](const auto &request, auto &response) {
    applyAEReferenceStream(request, response);
  });
  add_string("ae_strategy",
             [this](const auto &request, auto &response) { applyAEStrategy(request, response); });
  add_string("color_powerline_freq", [this](const auto &request, auto &response) {
    const std::vector<std::string> names = {"disable", "50hz", "60hz", "auto"};
    auto frequency = request->data;
    std::transform(frequency.begin(), frequency.end(), frequency.begin(),
                   [](unsigned char value) { return std::tolower(value); });
    const auto selected = std::find(names.begin(), names.end(), frequency);
    if (selected == names.end()) {
      response->message = "color_powerline_freq expects disable, 50hz, 60hz or auto.";
      return;
    }
    device_->setIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, selected - names.begin());
    color_powerline_freq_ = frequency;
    response->success = true;
  });

  const auto add_atomic = [this](const std::string &name, auto &target, double minimum) {
    addRuntimeControl({name}, [&target, minimum](const auto &values) {
      using Value = decltype(target.load());
      const auto value = values.front().get_parameter_value().template get<Value>();
      if (!std::isfinite(static_cast<double>(value)) || static_cast<double>(value) < minimum ||
          (std::is_integral_v<Value> &&
           static_cast<double>(value) > std::numeric_limits<int>::max())) {
        return runtimeResult(false, "Value is outside the supported runtime range.");
      }
      target.store(value);
      return runtimeResult(true);
    });
  };
  add_atomic("ordered_pc", ordered_pc_, 0);
  add_atomic("max_save_images_count", max_save_images_count_, 1);
  add_atomic("linear_accel_cov", linear_accel_cov_, 0);
  add_atomic("angular_vel_cov", angular_vel_cov_, 0);
  add_atomic("color_frame_queue_max_frames", color_frame_queue_max_frames_, 1);
  add_atomic("left_color_frame_queue_max_frames", left_color_frame_queue_max_frames_, 1);
  add_atomic("right_color_frame_queue_max_frames", right_color_frame_queue_max_frames_, 1);

  addRuntimeControl({"cloud_frame_id"}, [this](const auto &values) {
    std::lock_guard<decltype(point_cloud_mutex_)> lock(point_cloud_mutex_);
    cloud_frame_id_ = values.front().as_string();
    return runtimeResult(true);
  });
  addRuntimeControl(
      {"enable_point_cloud", "enable_colored_point_cloud", "point_cloud_qos"},
      [this](const auto &values) {
        bool depth = enable_point_cloud_.load();
        bool color = enable_colored_point_cloud_.load();
        auto qos = point_cloud_qos_;
        for (const auto &value : values) {
          if (value.get_name() == "enable_point_cloud") depth = value.as_bool();
          if (value.get_name() == "enable_colored_point_cloud") color = value.as_bool();
          if (value.get_name() == "point_cloud_qos") qos = value.as_string();
        }
        if ((depth || color) && !enable_stream_[DEPTH]) {
          return runtimeResult(false, "Point clouds require an already enabled depth stream.");
        }
        if (color && (!enable_stream_[COLOR] || !depth_registration_)) {
          return runtimeResult(false,
                               "Colored point clouds require existing color/depth streams and "
                               "alignment; runtime updates cannot restart streams.");
        }
        auto normalized_qos = qos;
        std::transform(normalized_qos.begin(), normalized_qos.end(), normalized_qos.begin(),
                       [](unsigned char value) { return std::toupper(value); });
        const std::set<std::string> profiles = {"SYSTEM_DEFAULT",   "DEFAULT",
                                                "PARAMETER_EVENTS", "SERVICES_DEFAULT",
                                                "PARAMETERS",       "SENSOR_DATA"};
        if (profiles.count(normalized_qos) == 0) {
          return runtimeResult(false, "Unsupported point_cloud_qos profile: " + qos);
        }
        std::lock_guard<decltype(point_cloud_mutex_)> lock(point_cloud_mutex_);
        const bool old_depth = enable_point_cloud_.load();
        const bool old_color = enable_colored_point_cloud_.load();
        const auto old_qos = point_cloud_qos_;
        const auto old_depth_pub = depth_cloud_pub_;
        const auto old_color_pub = depth_registration_cloud_pub_;
        try {
          enable_point_cloud_ = depth;
          enable_colored_point_cloud_ = color;
          point_cloud_qos_ = qos;
          setupPointCloudPublishers();
          return runtimeResult(true);
        } catch (...) {
          enable_point_cloud_ = old_depth;
          enable_colored_point_cloud_ = old_color;
          point_cloud_qos_ = old_qos;
          depth_cloud_pub_ = old_depth_pub;
          depth_registration_cloud_pub_ = old_color_pub;
          throw;
        }
      });
  addRuntimeControl({"publish_tf", "tf_publish_rate"}, [this](const auto &values) {
    bool enabled = publish_tf_.load();
    double rate = tf_publish_rate_.load();
    for (const auto &value : values) {
      if (value.get_name() == "publish_tf") enabled = value.as_bool();
      if (value.get_name() == "tf_publish_rate") rate = value.as_double();
    }
    if (!std::isfinite(rate) || rate < 0 || rate > 1000 ||
        (rate > 0 && 1000.0 / rate > std::numeric_limits<int>::max())) {
      return runtimeResult(false, "tf_publish_rate must be between 0 and 1000 Hz.");
    }
    const bool old_enabled = publish_tf_.load();
    const double old_rate = tf_publish_rate_.load();
    tf_thread_running_.store(false);
    tf_cv_.notify_all();
    if (tf_thread_ && tf_thread_->joinable()) tf_thread_->join();
    try {
      publish_tf_ = enabled;
      tf_publish_rate_ = rate;
      publishStaticTransforms();
    } catch (...) {
      publish_tf_ = old_enabled;
      tf_publish_rate_ = old_rate;
      try {
        publishStaticTransforms();
      } catch (const std::exception &e) {
        RCLCPP_ERROR_STREAM(logger_, "Could not restore TF publishing: " << e.what());
      }
      throw;
    }
    return runtimeResult(true);
  });

  for (const auto &stream : {COLOR, COLOR_LEFT, COLOR_RIGHT}) {
    if (sensors_.count(stream) == 0) continue;
    const auto prefix = stream_name_.at(stream);
    const auto enabled_name = "enable_" + prefix + "_decimation_filter";
    const auto scale_name = prefix + "_decimation_filter_scale";
    addRuntimeControl(
        {enabled_name, scale_name}, [this, stream, enabled_name, scale_name](const auto &values) {
          std::lock_guard<std::mutex> lock(image_filter_mutex_);
          auto &filters = stream == COLOR        ? color_filter_list_
                          : stream == COLOR_LEFT ? left_color_filter_list_
                                                 : right_color_filter_list_;
          std::shared_ptr<ob::DecimationFilter> filter;
          for (const auto &candidate : filters) {
            if (candidate && std::string(candidate->type()) == "DecimationFilter") {
              filter = candidate->template as<ob::DecimationFilter>();
              break;
            }
          }
          const bool created = !filter;
          if (created) filter = std::make_shared<ob::DecimationFilter>();
          bool enabled = filter->isEnabled();
          int scale = filter->getScaleValue();
          bool set_scale = false;
          for (const auto &value : values) {
            if (value.get_name() == enabled_name) enabled = value.as_bool();
            if (value.get_name() == scale_name) {
              scale = runtimeInt(value);
              set_scale = true;
            }
          }
          if (enabled && !supportsImageDecimation(stream_profile_.at(stream)->format())) {
            return runtimeResult(false, "Color decimation requires an uncompressed stream.");
          }
          const auto range = filter->getScaleRange();
          if (set_scale && (scale < range.min || scale > range.max)) {
            return runtimeResult(false, "Color decimation scale is outside the SDK range.");
          }
          if (set_scale) filter->setScaleValue(scale);
          filter->enable(enabled);
          if (created) filters.push_back(filter);
          return runtimeResult(true);
        });
  }

  for (const auto &stream : {INFRA1, INFRA2}) {
    if (sensors_.count(stream) == 0) continue;
    auto &filters = stream == INFRA1 ? left_ir_filter_list_ : right_ir_filter_list_;
    const auto found = std::find_if(filters.begin(), filters.end(), [](const auto &filter) {
      return filter && std::string(filter->type()) == "SequenceIdFilter";
    });
    if (found == filters.end()) continue;
    const auto filter = (*found)->as<ob::SequenceIdFilter>();
    const auto prefix = stream_name_.at(stream);
    const auto enabled_name = "enable_" + prefix + "_sequence_id_filter";
    const auto id_name = prefix + "_sequence_id_filter_id";
    addRuntimeControl({enabled_name, id_name},
                      [this, filter, enabled_name, id_name](const auto &values) {
                        std::lock_guard<std::mutex> lock(image_filter_mutex_);
                        bool enabled = filter->isEnabled();
                        for (const auto &value : values) {
                          if (value.get_name() == id_name)
                            filter->selectSequenceId(runtimeInt(value));
                          if (value.get_name() == enabled_name) enabled = value.as_bool();
                        }
                        filter->enable(enabled);
                        return runtimeResult(true);
                      });
  }

  for (bool color : {false, true}) {
    const std::string prefix = color ? "color_ae_roi_" : "depth_ae_roi_";
    addRuntimeControl(
        {prefix + "left", prefix + "right", prefix + "top", prefix + "bottom"},
        [this, color, prefix](const auto &values) {
          auto request = std::make_shared<SetArrays::Request>();
          auto response = std::make_shared<SetArrays::Response>();
          const std::vector<std::string> names = {prefix + "left", prefix + "right", prefix + "top",
                                                  prefix + "bottom"};
          for (const auto &name : names) {
            const auto coordinate = node_->get_parameter(name).as_int();
            if (coordinate < -1 || coordinate > std::numeric_limits<int16_t>::max()) {
              return runtimeResult(false,
                                   "ROI coordinates must fit the legacy int16 representation.");
            }
            request->data_param.push_back(static_cast<float>(coordinate));
          }
          for (const auto &value : values) {
            const auto index =
                std::find(names.begin(), names.end(), value.get_name()) - names.begin();
            const int coordinate = runtimeInt(value);
            if (coordinate < 0 || coordinate > std::numeric_limits<int16_t>::max()) {
              return runtimeResult(
                  false, "ROI coordinates must fit the legacy nonnegative int16 representation.");
            }
            request->data_param[index] = static_cast<float>(coordinate);
          }
          const auto stream = color ? COLOR : DEPTH;
          const int width = width_[stream];
          const int height = height_[stream];
          if (width <= 0 || height <= 0) {
            return runtimeResult(false, "ROI requires valid stream dimensions.");
          }
          // Preserve unspecified coordinates in ROS/reconnect intent; resolve only the SDK request.
          const auto configuration = roiServiceParameters(request, stream);
          const std::vector<int> defaults = {0, width - 1, 0, height - 1};
          for (size_t index = 0; index < defaults.size(); ++index) {
            if (request->data_param[index] == -1) {
              request->data_param[index] = static_cast<float>(defaults[index]);
            }
          }
          if (request->data_param[0] > request->data_param[1] ||
              request->data_param[2] > request->data_param[3]) {
            return runtimeResult(false, "ROI left/top must not exceed right/bottom.");
          }
          applyAeRoi(request, response, stream);
          if (response->success) {
            parameters_->rememberRuntimeConfiguration(configuration);
          }
          return runtimeResult(response->success, response->message);
        });
  }

  for (const auto &spec : runtimeFilters()) {
    bool supported = false;
    {
      std::lock_guard<std::mutex> lock(depth_filter_mutex_);
      supported = std::any_of(
          depth_filter_list_.begin(), depth_filter_list_.end(), [&](const auto &filter) {
            return filter && normalizeDepthFilterName(filter->type()) == spec.name;
          });
    }
    std::vector<std::pair<std::string, OBPropertyID>> hardware;
    if (spec.name == "NoiseRemovalFilter") {
      hardware = {{spec.enable_parameter, OB_PROP_DEPTH_SOFT_FILTER_BOOL},
                  {"noise_removal_filter_min_diff", OB_PROP_DEPTH_MAX_DIFF_INT},
                  {"noise_removal_filter_max_size", OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT}};
    } else if (spec.name == "HardwareNoiseRemovalFilter") {
      hardware = {{spec.enable_parameter, OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL},
                  {"hardware_noise_removal_filter_threshold",
                   OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT}};
    } else if (spec.name == "DispOutliersFilter") {
      hardware = {
          {spec.enable_parameter, OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL},
          {"disp_outliers_filter_search_mode", OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT}};
    }
    std::vector<std::string> names;
    if (!hardware.empty()) {
      for (const auto &binding : hardware) {
        const auto permission =
            spec.name == "NoiseRemovalFilter" && binding.first != spec.enable_parameter
                ? OB_PERMISSION_WRITE
                : OB_PERMISSION_READ_WRITE;
        if (device_->isPropertySupported(binding.second, permission))
          names.push_back(binding.first);
      }
    } else {
      if (spec.name == "EnhancedDepthFilter") supported = isLingBotSupportedPID(pid_);
      if (supported) {
        names.push_back(spec.enable_parameter);
        for (const auto &config : spec.config) names.push_back(config.first);
      }
    }
    if (names.empty()) continue;
    addRuntimeControl(names, [this, spec](const auto &values) {
      auto request = std::make_shared<SetFilter::Request>();
      auto response = std::make_shared<SetFilter::Response>();
      request->filter_name = spec.name;
      // Preserve the effective enable state when changing only filter configuration.
      request->filter_enable = node_->get_parameter(spec.enable_parameter).as_bool();
      {
        std::lock_guard<std::mutex> lock(depth_filter_mutex_);
        for (const auto &filter : depth_filter_list_) {
          if (filter && normalizeDepthFilterName(filter->type()) == spec.name) {
            request->filter_enable = filter->isEnabled();
            break;
          }
        }
      }
      if (spec.name == "EnhancedDepthFilter")
        request->filter_enable = enable_enhanced_depth_.load();
      if (spec.name == "NoiseRemovalFilter") request->filter_enable = enable_noise_removal_filter_;
      if (spec.name == "HardwareNoiseRemovalFilter")
        request->filter_enable = enable_hardware_noise_removal_filter_;
      if (spec.name == "DispOutliersFilter") request->filter_enable = enable_disp_outliers_filter_;
      for (const auto &value : values) {
        if (value.get_name() == spec.enable_parameter) {
          request->filter_enable = value.as_bool();
        } else {
          const auto binding =
              std::find_if(spec.config.begin(), spec.config.end(),
                           [&](const auto &config) { return config.first == value.get_name(); });
          orbbec_camera_msgs::msg::DepthFilterParam config;
          config.name = binding->second;
          config.value = filterConfigValue(value);
          request->filter_config.push_back(config);
        }
      }
      if (spec.name == "HDRMerge" && !request->filter_config.empty()) {
        for (const auto &config : spec.config) {
          auto value = node_->get_parameter(config.first);
          for (const auto &requested : values) {
            if (requested.get_name() == config.first) {
              value = requested;
            }
          }
          request->filter_param.push_back(static_cast<float>(runtimeInt(value)));
        }
        request->filter_config.clear();
      }
      applyFilter(request, response);
      if (response->success && spec.name == "HDRMerge" && !request->filter_param.empty()) {
        auto configuration = filterServiceParameters(*request);
        if (!configuration.empty()) {
          configuration.erase(configuration.begin());
          parameters_->rememberRuntimeConfiguration(configuration);
        }
      }
      return runtimeResult(response->success, response->message);
    });
  }
}
}  // namespace orbbec_camera
