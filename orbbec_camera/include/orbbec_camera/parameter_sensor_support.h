#pragma once

#include <libobsensor/h/ObTypes.h>

#include <map>
#include <set>
#include <string>

namespace orbbec_camera {
// Parameter ownership follows the SDK sensors exposed by the current mode, as in OBV.
// Unknown ownership is device-wide; SDK property/filter capability checks still apply.
inline bool supportsParameterSensors(const std::string &name, const std::set<OBSensorType> &sensors,
                                     bool known) {
  // Unknown capabilities retain declarations; a disabled stream still has a sensor.
  if (!known) return true;
  const auto has = [&sensors](OBSensorType type) { return sensors.count(type) != 0; };
  const bool depth = has(OB_SENSOR_DEPTH);
  const bool ir = has(OB_SENSOR_IR) || has(OB_SENSOR_IR_LEFT) || has(OB_SENSOR_IR_RIGHT);
  const bool color =
      has(OB_SENSOR_COLOR) || has(OB_SENSOR_COLOR_LEFT) || has(OB_SENSOR_COLOR_RIGHT);
  const std::map<std::string, OBSensorType> streams = {{"depth", OB_SENSOR_DEPTH},
                                                       {"ir", OB_SENSOR_IR},
                                                       {"left_ir", OB_SENSOR_IR_LEFT},
                                                       {"right_ir", OB_SENSOR_IR_RIGHT},
                                                       {"color", OB_SENSOR_COLOR},
                                                       {"left_color", OB_SENSOR_COLOR_LEFT},
                                                       {"right_color", OB_SENSOR_COLOR_RIGHT},
                                                       {"accel", OB_SENSOR_ACCEL},
                                                       {"gyro", OB_SENSOR_GYRO}};
  static const std::set<std::string> stream_fields = {"width",
                                                      "height",
                                                      "fps",
                                                      "format",
                                                      "mirror",
                                                      "flip",
                                                      "rotation",
                                                      "qos",
                                                      "qos_history",
                                                      "qos_depth",
                                                      "camera_info_qos",
                                                      "optical_frame_id",
                                                      "frame_queue_max_frames",
                                                      "decimation_filter_scale",
                                                      "sequence_id_filter_id",
                                                      "decimation_factor",
                                                      "rate",
                                                      "range"};
  for (const auto &stream : streams) {
    if (name == "enable_" + stream.first || name == "enable_" + stream.first + "_undistortion" ||
        name == "enable_" + stream.first + "_decimation_filter" ||
        name == "enable_" + stream.first + "_sequence_id_filter")
      return has(stream.second);
    const auto prefix = stream.first + "_";
    if (name.rfind(prefix, 0) == 0 && stream_fields.count(name.substr(prefix.size())))
      return has(stream.second);
  }
  static const std::set<std::string> depth_parameters = {"preset_resolution_config",
                                                         "depth_exposure",
                                                         "depth_gain",
                                                         "depth_brightness",
                                                         "mean_intensity_set_point",
                                                         "enable_depth_auto_exposure",
                                                         "enable_depth_auto_exposure_priority",
                                                         "min_depth_limit",
                                                         "max_depth_limit",
                                                         "depth_precision",
                                                         "depth_filter_config",
                                                         "disparity_range_mode",
                                                         "disparity_search_offset",
                                                         "disparity_to_depth_mode",
                                                         "disparity_offset_config",
                                                         "offset_index0",
                                                         "offset_index1",
                                                         "enable_depth_scale",
                                                         "depth_colorizer_mode",
                                                         "enable_point_cloud",
                                                         "point_cloud_qos",
                                                         "ordered_pc",
                                                         "cloud_frame_id",
                                                         "point_cloud_decimation_filter_factor"};
  static const std::set<std::string> ir_parameters = {"ir_exposure",
                                                      "ir_gain",
                                                      "ir_brightness",
                                                      "ir_ae_max_exposure",
                                                      "enable_ir_auto_exposure",
                                                      "enable_ir_long_exposure",
                                                      "ir_info_url"};
  static const std::set<std::string> color_parameters = {"color_exposure",
                                                         "color_gain",
                                                         "color_ae_max_exposure",
                                                         "color_ae_max_gain",
                                                         "color_white_balance",
                                                         "color_brightness",
                                                         "color_roi_brightness",
                                                         "color_sharpness",
                                                         "color_gamma",
                                                         "color_saturation",
                                                         "color_contrast",
                                                         "color_hue",
                                                         "color_backlight_compensation",
                                                         "color_anti_flicker",
                                                         "color_denoising_level",
                                                         "color_mjpeg_quality",
                                                         "color_powerline_freq",
                                                         "enable_color_auto_exposure",
                                                         "enable_color_auto_exposure_priority",
                                                         "enable_color_auto_white_balance",
                                                         "color_preset",
                                                         "color_info_url"};
  if (depth_parameters.count(name) || name.rfind("depth_ae_roi_", 0) == 0 ||
      name.rfind("interleave_", 0) == 0 || name.rfind("hdr_index", 0) == 0 ||
      name.rfind("laser_index", 0) == 0)
    return depth;
  if (name == "ir_ae_max_exposure" || name == "ir_info_url") return depth || ir;
  if (ir_parameters.count(name)) return ir;
  if (color_parameters.count(name) || name.rfind("color_ae_roi_", 0) == 0) return color;
  if (name == "depth_registration" || name == "enable_colored_point_cloud" ||
      name == "enable_d2c_viewer" || name == "align_mode" || name == "align_target_stream")
    return depth && has(OB_SENSOR_COLOR);
  if (name == "enable_laser" || name == "enable_ldp" || name == "ldp_power_level" ||
      name == "laser_energy_level")
    return depth || has(OB_SENSOR_LIDAR);
  if (name == "enable_accel_data_correction") return has(OB_SENSOR_ACCEL);
  if (name == "enable_gyro_data_correction") return has(OB_SENSOR_GYRO);
  // Device-wide mode selectors, synchronization and monitoring remain independent.
  return true;
}

}  // namespace orbbec_camera
