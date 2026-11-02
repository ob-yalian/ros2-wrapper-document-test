/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include "orbbec_camera/ob_camera_node.h"
#include "orbbec_camera/utils.h"
#include "orbbec_camera/parameter_descriptions.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>

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

struct FalsePositiveFilterParameter {
  std::string name;
  std::string config;
  rclcpp::ParameterValue initial_value;
};

const std::vector<FalsePositiveFilterParameter> &falsePositiveFilterParameters() {
  static const std::vector<FalsePositiveFilterParameter> parameters = {
      {"enable_false_positive_edge_bleed_filter", "fpEdgeBleedFilterEnable",
       rclcpp::ParameterValue(false)},
      {"false_positive_filter_edge_bleed_roi_min_x_ratio", "fpebfROIMinXRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_edge_bleed_roi_max_x_ratio", "fpebfROIMaxXRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_edge_bleed_roi_min_y_ratio", "fpebfROIMinYRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_edge_bleed_roi_max_y_ratio", "fpebfROIMaxYRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_edge_bleed_min_bleed_length", "fpebfMinBleedLength",
       rclcpp::ParameterValue(-1)},
      {"enable_false_positive_texture_sparsity_filter", "fpTextureSparsityFilterEnable",
       rclcpp::ParameterValue(false)},
      {"false_positive_filter_texture_sparsity_roi_min_x_ratio", "fptsfROIMinXRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_texture_sparsity_roi_max_x_ratio", "fptsfROIMaxXRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_texture_sparsity_roi_min_y_ratio", "fptsfROIMinYRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_texture_sparsity_roi_max_y_ratio", "fptsfROIMaxYRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_texture_sparsity_max_noise_level", "fptsfMaxNoiseLevel",
       rclcpp::ParameterValue(-1)},
      {"false_positive_filter_texture_sparsity_max_speckle_size", "fptsfMaxSpeckleSize",
       rclcpp::ParameterValue(-1)},
      {"enable_false_positive_pattern_ambiguity_filter", "fpPatternAmbiguityFilterEnable",
       rclcpp::ParameterValue(false)},
      {"false_positive_filter_pattern_ambiguity_roi_min_x_ratio", "fppafROIMinXRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_pattern_ambiguity_roi_max_x_ratio", "fppafROIMaxXRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_pattern_ambiguity_roi_min_y_ratio", "fppafROIMinYRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_pattern_ambiguity_roi_max_y_ratio", "fppafROIMaxYRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_pattern_ambiguity_max_noise_level", "fppafMaxNoiseLevel",
       rclcpp::ParameterValue(-1)},
      {"false_positive_filter_pattern_ambiguity_max_speckle_size", "fppafMaxSpeckleSize",
       rclcpp::ParameterValue(-1)},
      {"false_positive_filter_pattern_ambiguity_max_width_ratio", "fppafMaxWidthRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_pattern_ambiguity_max_height_ratio", "fppafMaxHeightRatio",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_pattern_ambiguity_tolerance", "fppafTolerance",
       rclcpp::ParameterValue(-1.0)},
      {"false_positive_filter_pattern_ambiguity_score", "fppafScore", rclcpp::ParameterValue(-1)},
  };
  return parameters;
}

void validateFalsePositiveFilterConfig(const std::shared_ptr<ob::Filter> &filter,
                                       const std::vector<rclcpp::Parameter> &values) {
  if (!filter) throw std::runtime_error("FalsePositiveFilter is unavailable.");
  const auto schemas = filter->getConfigSchemaVec();
  for (const auto &value : values) {
    const auto &parameters = falsePositiveFilterParameters();
    const auto binding = std::find_if(parameters.begin(), parameters.end(), [&](const auto &item) {
      return item.name == value.get_name();
    });
    if (binding == parameters.end()) continue;  // The master switch has no config schema.
    if (value.get_type() != binding->initial_value.get_type()) {
      throw std::invalid_argument("Invalid type for " + value.get_name());
    }
    const auto schema = std::find_if(schemas.begin(), schemas.end(), [&](const auto &item) {
      return item.name && binding->config == item.name;
    });
    if (schema == schemas.end()) {
      throw std::invalid_argument(value.get_name() + ": configuration is not supported.");
    }
    const double requested = value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL
                                 ? static_cast<double>(value.as_bool())
                             : value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                                 ? static_cast<double>(value.as_int())
                                 : value.as_double();
    if (!std::isfinite(requested) || requested < schema->min || requested > schema->max) {
      throw std::invalid_argument(value.get_name() + ": value is outside the SDK range.");
    }
    if (std::isfinite(schema->step) && schema->step > 0) {
      const double steps = (requested - schema->min) / schema->step;
      if (std::abs(steps - std::round(steps)) > 1e-6 * std::max(1.0, std::abs(steps))) {
        throw std::invalid_argument(value.get_name() + ": value does not match the SDK step.");
      }
    }
  }
}

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
      {"FalsePositiveFilter", "enable_false_positive_filter",
       [] {
         std::vector<std::pair<std::string, std::string>> config;
         for (const auto &parameter : falsePositiveFilterParameters()) {
           config.emplace_back(parameter.name, parameter.config);
         }
         return config;
       }()},
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

std::vector<std::pair<std::string, OBPropertyID>> runtimeFilterProperties(
    const RuntimeFilter &spec) {
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
  return hardware;
}

std::string filterRangeDescription(const std::string &name,
                                   const OBFilterConfigSchemaItem &schema) {
  if (!std::isfinite(schema.min) || !std::isfinite(schema.max) || !std::isfinite(schema.step) ||
      !std::isfinite(schema.def) || schema.min > schema.max) {
    return " SDK default/range unavailable.";
  }
  std::ostringstream text;
  // ROS string selectors expose names, not the SDK's numeric enum codes.
  if (name.rfind("enable_", 0) == 0 && (schema.def == 0.0 || schema.def == 1.0)) {
    text << " SDK default: " << (schema.def != 0.0 ? "true" : "false") << ".";
  } else if (name == "hole_filling_filter_mode" || name == "disp_outliers_filter_search_mode") {
    const std::vector<std::string> choices =
        name == "hole_filling_filter_mode"
            ? std::vector<std::string>{"FILL_TOP", "FILL_NEAREST", "FILL_FAREST"}
            : std::vector<std::string>{"FULL", "OFFSET_80"};
    if (schema.def >= 0 && schema.def < choices.size() && std::floor(schema.def) == schema.def) {
      text << " SDK default: " << choices[static_cast<size_t>(schema.def)] << ".";
    }
    text << " SDK choices:";
    for (size_t i = 0; i < choices.size(); ++i) {
      if (i >= schema.min && i <= schema.max &&
          (schema.step <= 0 || std::fmod(i - schema.min, schema.step) == 0)) {
        text << " " << choices[i];
      }
    }
    text << ".";
  } else {
    text << " SDK default: " << schema.def << ". SDK range: [" << schema.min << ", " << schema.max
         << "]";
    if (schema.step > 0) text << ", step: " << schema.step;
    text << ".";
  }
  return text.str();
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
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return value.as_bool() ? "1" : "0";
  }
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
      value.get_name().rfind("false_positive_filter_", 0) == 0) {
    std::ostringstream text;
    text << std::setprecision(std::numeric_limits<double>::max_digits10) << value.as_double();
    return text.str();
  }
  return value.get_type() == rclcpp::ParameterType::PARAMETER_STRING ? value.as_string()
                                                                     : value.value_to_string();
}

rclcpp::ParameterValue readFilterConfig(const std::shared_ptr<ob::Filter> &filter,
                                        const std::string &key, rclcpp::ParameterType type) {
  if (!filter)
    throw std::runtime_error("Filter has not been created; configuration is unavailable.");
  const double value = filter->getConfigValue(key == "scale" ? "decimate" : key);
  if (!std::isfinite(value)) throw std::runtime_error("Filter returned a nonfinite value.");
  if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
    if (value != 0.0 && value != 1.0)
      throw std::runtime_error("Filter returned an invalid boolean value.");
    return rclcpp::ParameterValue(value != 0.0);
  }
  if (type == rclcpp::ParameterType::PARAMETER_STRING) {
    const std::vector<std::string> modes = {"FILL_TOP", "FILL_NEAREST", "FILL_FAREST"};
    if (key != "hole_filling_mode" || value < 0 || value >= modes.size() ||
        std::floor(value) != value) {
      throw std::runtime_error("Filter returned an unknown mode.");
    }
    return rclcpp::ParameterValue(modes[static_cast<size_t>(value)]);
  }
  if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) return rclcpp::ParameterValue(value);
  if (std::floor(value) != value || value < std::numeric_limits<int32_t>::min() ||
      value > std::numeric_limits<int32_t>::max()) {
    throw std::runtime_error("Filter value cannot be represented as an integer.");
  }
  return rclcpp::ParameterValue(static_cast<int>(value));
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
    if (node_->has_parameter(name) && !node_->describe_parameter(name).read_only) {
      runtime_control_groups_.emplace(name, names.front());
    }
  }
  runtime_control_setters_.emplace(names.front(), std::move(setter));
}

bool OBCameraNode::isFilterParameterSupported(const std::string &name) const {
  const auto found = filter_parameter_support_.find(name);
  return found != filter_parameter_support_.end() && found->second;
}

void OBCameraNode::setupFilterParameterSupport() {
  filter_parameter_support_.clear();
  filter_property_ids_.clear();
  const bool has_depth = sensors_.count(DEPTH) != 0;
  const bool supports_hdr_config =
      has_depth && isPropertyParameterSupported("HDR configuration", OB_STRUCT_DEPTH_HDR_CONFIG,
                                                OB_PERMISSION_READ_WRITE);
  const bool supports_hdr_merge =
      has_depth && queryDeviceCapability("HDR merge", [this]() {
        if (device_->isFrameInterleaveSupported()) return true;
        if (!is_playback_device_) {
          return device_->isPropertySupported(OB_STRUCT_DEPTH_HDR_CONFIG, OB_PERMISSION_READ_WRITE);
        }
        if (!device_->isPropertySupported(OB_STRUCT_DEPTH_HDR_CONFIG, OB_PERMISSION_READ))
          return false;
        OBHdrConfig config{};
        uint32_t size = sizeof(config);
        device_->getStructuredData(OB_STRUCT_DEPTH_HDR_CONFIG, reinterpret_cast<uint8_t *>(&config),
                                   &size);
        return config.enable != 0;
      });

  for (const auto &spec : runtimeFilters()) {
    const auto hardware = runtimeFilterProperties(spec);
    if (!hardware.empty()) {
      for (const auto &binding : hardware) {
        filter_property_ids_[binding.first] = binding.second;
        filter_parameter_support_[binding.first] =
            has_depth &&
            (isPropertyParameterSupported(binding.first, binding.second) ||
             isPropertyParameterSupported(binding.first, binding.second, OB_PERMISSION_READ));
      }
      continue;
    }

    bool supported = supportsRecommendedFilter(DEPTH, spec.name);
    if (spec.name == "DecimationFilter" && (pid_ == GEMINI2_PID || pid_ == GEMINI2L_PID)) {
      // These devices have an existing software fallback outside the recommendation list.
      supported = has_depth;
    } else if (spec.name == "HDRMerge") {
      supported = supported && supports_hdr_merge;
    } else if (spec.name == "EnhancedDepthFilter") {
      supported = has_depth && !is_playback_device_ && isLingBotSupportedPID(pid_) &&
                  queryDeviceCapability("enhanced depth filter", [this]() {
                    return device_->isLicenseAuthorizationSupported();
                  });
    }
    std::set<std::string> config_names;
    bool has_config_schema = false;
    if (spec.name == "FalsePositiveFilter" && supported) {
      for (const auto &filter : depth_filter_list_) {
        if (!filter || normalizeDepthFilterName(filter->type()) != spec.name) continue;
        queryDeviceCapability("FalsePositiveFilter configuration", [&]() {
          for (const auto &schema : filter->getConfigSchemaVec()) {
            if (schema.name) config_names.insert(schema.name);
          }
          has_config_schema = true;
          return true;
        });
        break;
      }
    }
    filter_parameter_support_[spec.enable_parameter] = supported;
    for (const auto &config : spec.config) {
      filter_parameter_support_[config.first] =
          supported && (spec.name != "HDRMerge" || supports_hdr_config) &&
          (!has_config_schema || config_names.count(config.second) != 0);
    }
  }
  filter_parameter_support_["enhanced_depth_model_path"] =
      isFilterParameterSupported("enable_enhanced_depth");

  for (const auto &stream : {COLOR, COLOR_LEFT, COLOR_RIGHT}) {
    // Runtime color decimation can already create a software filter when none was recommended.
    const auto &name = stream_name_.at(stream);
    filter_parameter_support_["enable_" + name + "_decimation_filter"] =
        sensors_.count(stream) != 0;
    filter_parameter_support_[name + "_decimation_filter_scale"] = sensors_.count(stream) != 0;
  }
  for (const auto &stream : {INFRA1, INFRA2}) {
    const auto &name = stream_name_.at(stream);
    const bool supported = supportsRecommendedFilter(stream, "SequenceIdFilter");
    filter_parameter_support_["enable_" + name + "_sequence_id_filter"] = supported;
    filter_parameter_support_[name + "_sequence_id_filter_id"] = supported;
  }
  for (const auto &stream : IMAGE_STREAMS) {
    // Like OBV, the driver supplies undistortion independently of recommendations.
    filter_parameter_support_["enable_" + stream_name_.at(stream) + "_undistortion"] =
        sensors_.count(stream) != 0;
  }
}

void OBCameraNode::setupFalsePositiveFilterParameters() {
  std::vector<rclcpp::Parameter> values;
  const auto overrides = node_->get_node_parameters_interface()->get_parameter_overrides();
  bool invalid_configuration = false;
  for (const auto &binding : falsePositiveFilterParameters()) {
    if (!isFilterParameterSupported(binding.name)) continue;
    device_param_names_.insert(binding.name);
    auto descriptor = describeCameraParameter(binding.name);
    descriptor.dynamic_typing = true;
    const auto override_it = overrides.find(binding.name);
    if (!parameters_->hasRuntimeOverride(binding.name) && override_it != overrides.end() &&
        override_it->second.get_type() != binding.initial_value.get_type()) {
      invalid_configuration = true;
      RCLCPP_ERROR_STREAM(
          logger_,
          "Skip FalsePositiveFilter startup configuration: invalid type for " << binding.name);
      if (!node_->has_parameter(binding.name)) {
        // Keep a correctly typed control available even when a YAML override is invalid.
        node_->declare_parameter(binding.name, binding.initial_value, descriptor, true);
      }
    }
    const auto value = parameters_->setParam(binding.name, binding.initial_value, {}, descriptor);
    if (value.get_type() != binding.initial_value.get_type()) {
      throw std::invalid_argument("Invalid type for " + binding.name);
    }
    // An omitted boolean and numeric -1 both preserve preset/JSON configuration.
    if (!isLaunchParamProvided(binding.name) ||
        (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
         value.get<int64_t>() == -1) ||
        (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
         value.get<double>() == -1.0))
      continue;
    values.emplace_back(binding.name, value);
  }
  if (invalid_configuration || values.empty()) return;
  try {
    auto request = std::make_shared<SetFilter::Request>();
    auto response = std::make_shared<SetFilter::Response>();
    request->filter_name = "FalsePositiveFilter";
    {
      std::lock_guard<std::mutex> lock(depth_filter_mutex_);
      const auto found = std::find_if(
          depth_filter_list_.begin(), depth_filter_list_.end(), [](const auto &filter) {
            return filter && normalizeDepthFilterName(filter->type()) == "FalsePositiveFilter";
          });
      const auto filter = found == depth_filter_list_.end() ? nullptr : *found;
      // Validate the whole requested configuration before the first SDK write.
      validateFalsePositiveFilterConfig(filter, values);
      request->filter_enable = filter->isEnabled();
    }
    for (const auto &binding : falsePositiveFilterParameters()) {
      for (const auto &value : values) {
        if (value.get_name() == binding.name) {
          orbbec_camera_msgs::msg::DepthFilterParam config;
          config.name = binding.config;
          config.value = filterConfigValue(value);
          request->filter_config.push_back(config);
        }
      }
    }
    setFilterCallback(request, response);
    if (!response->success) throw std::runtime_error(response->message);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_,
                        "Could not apply FalsePositiveFilter startup configuration: " << e.what());
  }
}

void OBCameraNode::setupFilterParameterDescriptions() {
  // Read existing instances after preset/JSON initialization. Metadata never enables a
  // stream, creates a filter, or replaces a requested ROS value with SDK readback.
  const auto find_filter = [](const auto &filters,
                              const std::string &type) -> std::shared_ptr<ob::Filter> {
    for (const auto &filter : filters) {
      if (filter && normalizeDepthFilterName(filter->type()) == type) return filter;
    }
    return nullptr;
  };
  const auto update = [this](const std::string &name, const std::string &metadata) {
    if (!node_->has_parameter(name)) return;
    try {
      parameters_->updateDescription(name, describeCameraParameter(name).description + metadata);
    } catch (const std::exception &e) {
      RCLCPP_WARN_STREAM(logger_, "Could not update description for " << name << ": " << e.what());
    }
  };
  const auto describe_filter = [&update](const std::string &name, const std::string &config,
                                         const std::shared_ptr<ob::Filter> &filter) {
    std::string metadata = " SDK default/range unavailable.";
    const auto sdk_config = config == "scale" ? "decimate" : config;
    if (filter) {
      try {
        for (const auto &schema : filter->getConfigSchemaVec()) {
          if (schema.name && sdk_config == schema.name) {
            metadata = filterRangeDescription(name, schema);
            break;
          }
        }
      } catch (const std::exception &) {
        // Keep the basic description when a filter does not expose its schema.
      }
    }
    update(name, metadata);
  };
  for (const auto &spec : runtimeFilters()) {
    const auto hardware = runtimeFilterProperties(spec);
    if (!hardware.empty()) {
      for (const auto &binding : hardware) {
        if (!node_->has_parameter(binding.first)) continue;
        const auto type = node_->get_parameter(binding.first).get_type();
        rcl_interfaces::msg::ParameterDescriptor metadata;
        if (type == rclcpp::ParameterType::PARAMETER_STRING) {
          try {
            const auto range = device_->getIntPropertyRange(binding.second);
            const OBFilterConfigSchemaItem schema{nullptr,
                                                  OB_FILTER_CONFIG_VALUE_TYPE_INT,
                                                  static_cast<double>(range.min),
                                                  static_cast<double>(range.max),
                                                  static_cast<double>(range.step),
                                                  static_cast<double>(range.def),
                                                  nullptr};
            metadata.description = filterRangeDescription(binding.first, schema);
          } catch (const std::exception &) {
            metadata.description = " SDK default/range unavailable.";
          }
        } else {
          appendPropertyDescription(metadata, binding.second, type);
        }
        update(binding.first, metadata.description);
      }
      continue;
    }
    const std::shared_ptr<ob::Filter> filter = spec.name == "EnhancedDepthFilter"
                                                   ? enhanced_depth_filter_
                                                   : find_filter(depth_filter_list_, spec.name);
    for (const auto &config : spec.config) describe_filter(config.first, config.second, filter);
  }
  describe_filter("color_decimation_filter_scale", "scale",
                  find_filter(color_filter_list_, "DecimationFilter"));
  describe_filter("left_color_decimation_filter_scale", "scale",
                  find_filter(left_color_filter_list_, "DecimationFilter"));
  describe_filter("right_color_decimation_filter_scale", "scale",
                  find_filter(right_color_filter_list_, "DecimationFilter"));
  describe_filter("left_ir_sequence_id_filter_id", "sequenceid",
                  find_filter(left_ir_filter_list_, "SequenceIdFilter"));
  describe_filter("right_ir_sequence_id_filter_id", "sequenceid",
                  find_filter(right_ir_filter_list_, "SequenceIdFilter"));
}

void OBCameraNode::setupFilterParameterReaders() {
  for (const auto &spec : runtimeFilters()) {
    std::vector<std::pair<std::string, std::string>> bindings = spec.config;
    bindings.emplace_back(spec.enable_parameter, "");
    for (const auto &binding : bindings) {
      if (!node_->has_parameter(binding.first)) continue;
      const auto type = node_->get_parameter(binding.first).get_type();
      addParameterReader(binding.first, [this, spec, binding, type] {
        const bool enable = binding.first == spec.enable_parameter;
        for (const auto &property : runtimeFilterProperties(spec)) {
          if (property.first == binding.first) {
            return readPropertyParameter(binding.first, property.second, type);
          }
        }
        if (spec.name == "HDRMerge" && !enable) {
          OBHdrConfig config{};
          uint32_t size = sizeof(config);
          device_->getStructuredData(OB_STRUCT_DEPTH_HDR_CONFIG,
                                     reinterpret_cast<uint8_t *>(&config), &size);
          if (size != sizeof(config)) throw std::runtime_error("Invalid HDR configuration size.");
          if (binding.second == "exposure_1")
            return rclcpp::ParameterValue(static_cast<int64_t>(config.exposure_1));
          if (binding.second == "gain_1")
            return rclcpp::ParameterValue(static_cast<int64_t>(config.gain_1));
          if (binding.second == "exposure_2")
            return rclcpp::ParameterValue(static_cast<int64_t>(config.exposure_2));
          return rclcpp::ParameterValue(static_cast<int64_t>(config.gain_2));
        }
        if (spec.name == "EnhancedDepthFilter") {
          std::lock_guard<std::mutex> lock(enhanced_depth_filter_mutex_);
          if (enable) return rclcpp::ParameterValue(enable_enhanced_depth_.load());
          return readFilterConfig(enhanced_depth_filter_, binding.second, type);
        }
        std::lock_guard<std::mutex> lock(depth_filter_mutex_);
        const auto found = std::find_if(
            depth_filter_list_.begin(), depth_filter_list_.end(), [&](const auto &filter) {
              return filter && normalizeDepthFilterName(filter->type()) == spec.name;
            });
        const auto filter = found == depth_filter_list_.end() ? nullptr : *found;
        if (enable) {
          // Property-backed controls without read access cannot be inferred from absence.
          if (!filter && !runtimeFilterProperties(spec).empty()) {
            throw std::runtime_error("Filter property is not readable.");
          }
          return rclcpp::ParameterValue(filter && filter->isEnabled());
        }
        return readFilterConfig(filter, binding.second, type);
      });
    }
  }
  for (const auto &stream : {COLOR, COLOR_LEFT, COLOR_RIGHT, INFRA1, INFRA2}) {
    const bool color = stream == COLOR || stream == COLOR_LEFT || stream == COLOR_RIGHT;
    const auto prefix = stream_name_.at(stream);
    const auto enable = "enable_" + prefix + (color ? "_decimation_filter" : "_sequence_id_filter");
    const auto config = prefix + (color ? "_decimation_filter_scale" : "_sequence_id_filter_id");
    for (const auto &name : {enable, config}) {
      addParameterReader(name, [this, stream, color, name, enable] {
        std::lock_guard<std::mutex> lock(image_filter_mutex_);
        const auto &filters = stream == COLOR         ? color_filter_list_
                              : stream == COLOR_LEFT  ? left_color_filter_list_
                              : stream == COLOR_RIGHT ? right_color_filter_list_
                              : stream == INFRA1      ? left_ir_filter_list_
                                                      : right_ir_filter_list_;
        const auto found = std::find_if(filters.begin(), filters.end(), [&](const auto &filter) {
          return filter && normalizeDepthFilterName(filter->type()) ==
                               (color ? "DecimationFilter" : "SequenceIdFilter");
        });
        const auto filter = found == filters.end() ? nullptr : *found;
        if (name == enable) return rclcpp::ParameterValue(filter && filter->isEnabled());
        return readFilterConfig(filter, color ? "scale" : "sequenceid",
                                rclcpp::ParameterType::PARAMETER_INTEGER);
      });
    }
  }
}

void OBCameraNode::setupRuntimeControls() {
  if (!static_tf_broadcaster_) {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node_);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);
  }
  // Reuse local device operations; mapped services enter through the parameter API.
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
          setMirrorCallback(request, response, stream);
        });
    add_bool(
        name + "_flip", [this, stream, check_orientation](const auto &request, auto &response) {
          if (!check_orientation(request->data)) {
            response->message = "Color orientation requires an uncompressed stream on this device.";
            return;
          }
          setFlipCallback(request, response, stream);
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
          setRotationCallback(request, response, stream);
        });
  }
  add_bool("enable_laser", [this](const auto &request, auto &response) {
    setLaserEnableCallback(nullptr, request, response);
  });
  add_bool("enable_ldp", [this](const auto &request, auto &response) {
    setLdpEnableCallback(nullptr, request, response);
  });
  add_bool("enable_ptp_config", [this](const auto &request, auto &response) {
    setPtpConfigCallback(nullptr, request, response);
  });
  add_bool("enable_ir_long_exposure", [this](const auto &request, auto &response) {
    setIRLongExposureCallback(request, response);
  });
  add_int("point_cloud_decimation_filter_factor", [this](const auto &request, auto &response) {
    setPointCloudDecimationCallback(request, response);
  });
  add_int("disparity_search_offset", [this](const auto &request, auto &response) {
    setDisparitySearchOffsetCallback(request, response);
  });
  add_int("sync_io_voltage_level", [this](const auto &request, auto &response) {
    setSyncIoVoltageLevelCallback(request, response);
  });
  add_int("disparity_range_mode", [this](const auto &request, auto &response) {
    setDisparityRangeModeCallback(request, response);
  });
  add_string("ae_reference_stream", [this](const auto &request, auto &response) {
    setAEReferenceStreamCallback(request, response);
  });
  add_string("ae_strategy", [this](const auto &request, auto &response) {
    setAEStrategyCallback(request, response);
  });
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
    if (!isFilterParameterSupported(enabled_name)) continue;
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
          const bool was_enabled = !created && filter->isEnabled();
          const int previous_scale = filter->getScaleValue();
          bool enabled = was_enabled;
          int scale = previous_scale;
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
          // Align keeps stream profiles configured at startup. Changing the incoming color
          // dimensions without rebuilding it can crash the SDK.
          if (stream == COLOR && align_filter_ &&
              (enabled ? scale : 1) != (was_enabled ? previous_scale : 1)) {
            return runtimeResult(
                false,
                "Changing color decimation dimensions while software alignment is active "
                "requires restarting the camera with the desired launch configuration.");
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
    if (!isFilterParameterSupported(enabled_name)) continue;
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
          std::vector<rclcpp::Parameter> configuration;
          for (const auto &name : names) {
            const auto coordinate = node_->get_parameter(name).as_int();
            if (coordinate < -1 || coordinate > std::numeric_limits<int16_t>::max()) {
              return runtimeResult(false,
                                   "ROI coordinates must fit the legacy int16 representation.");
            }
            request->data_param.push_back(static_cast<float>(coordinate));
            configuration.emplace_back(name, coordinate);
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
            configuration[index] = value;
          }
          const auto stream = color ? COLOR : DEPTH;
          const int width = width_[stream];
          const int height = height_[stream];
          if (width <= 0 || height <= 0) {
            return runtimeResult(false, "ROI requires valid stream dimensions.");
          }
          // Preserve unspecified coordinates in ROS/reconnect intent; resolve only the SDK request.
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
          setAeRoiCallback(request, response, stream);
          if (response->success) {
            parameters_->markRuntimeParameters(configuration);
          }
          return runtimeResult(response->success, response->message);
        });
  }

  for (const auto &spec : runtimeFilters()) {
    const auto hardware = runtimeFilterProperties(spec);
    if (hardware.empty() && (!isFilterParameterSupported(spec.enable_parameter) ||
                             !node_->has_parameter(spec.enable_parameter)))
      continue;
    bool supported = false;
    {
      std::lock_guard<std::mutex> lock(depth_filter_mutex_);
      supported = std::any_of(
          depth_filter_list_.begin(), depth_filter_list_.end(), [&](const auto &filter) {
            return filter && normalizeDepthFilterName(filter->type()) == spec.name;
          });
    }
    std::vector<std::string> names;
    if (!hardware.empty()) {
      for (const auto &binding : hardware) {
        if (!isFilterParameterSupported(binding.first) || !node_->has_parameter(binding.first))
          continue;
        const auto permission =
            spec.name == "NoiseRemovalFilter" && binding.first != spec.enable_parameter
                ? OB_PERMISSION_WRITE
                : OB_PERMISSION_READ_WRITE;
        if (queryDeviceCapability(binding.first, [&]() {
              return device_->isPropertySupported(binding.second, permission);
            }))
          names.push_back(binding.first);
      }
    } else {
      if (spec.name == "EnhancedDepthFilter")
        supported = isFilterParameterSupported(spec.enable_parameter);
      if (supported) {
        names.push_back(spec.enable_parameter);
        for (const auto &config : spec.config) {
          if (isFilterParameterSupported(config.first) && node_->has_parameter(config.first))
            names.push_back(config.first);
        }
      }
    }
    if (names.empty()) continue;
    addRuntimeControl(names, [this, spec](const auto &values) {
      auto request = std::make_shared<SetFilter::Request>();
      auto response = std::make_shared<SetFilter::Response>();
      std::vector<rclcpp::Parameter> configuration;
      request->filter_name = spec.name;
      if (spec.name == "FalsePositiveFilter" &&
          std::any_of(values.begin(), values.end(), [&](const auto &value) {
            return value.get_name() != spec.enable_parameter;
          })) {
        std::lock_guard<std::mutex> lock(depth_filter_mutex_);
        const auto found = std::find_if(
            depth_filter_list_.begin(), depth_filter_list_.end(), [&](const auto &filter) {
              return filter && normalizeDepthFilterName(filter->type()) == spec.name;
            });
        validateFalsePositiveFilterConfig(found == depth_filter_list_.end() ? nullptr : *found,
                                          values);
      }
      // Preserve the effective enable state when changing only filter configuration.
      request->filter_enable = node_->has_parameter(spec.enable_parameter) &&
                               node_->get_parameter(spec.enable_parameter).as_bool();
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
          if (!node_->has_parameter(config.first)) {
            return runtimeResult(false,
                                 "HDR exposure configuration is not supported by this device.");
          }
          auto value = node_->get_parameter(config.first);
          for (const auto &requested : values) {
            if (requested.get_name() == config.first) {
              value = requested;
            }
          }
          request->filter_param.push_back(static_cast<float>(runtimeInt(value)));
          configuration.push_back(value);
        }
        request->filter_config.clear();
      }
      setFilterCallback(request, response);
      if (response->success && !configuration.empty()) {
        parameters_->markRuntimeParameters(configuration);
      }
      return runtimeResult(response->success, response->message);
    });
  }
}
}  // namespace orbbec_camera
