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

#include "orbbec_camera/parameter_descriptions.h"

#include <algorithm>
#include <map>

namespace orbbec_camera {
namespace {
bool endsWith(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

rcl_interfaces::msg::ParameterDescriptor describeCameraParameter(const std::string& name) {
  static const std::map<std::string, std::string> descriptions = {
      {"camera_name", "Camera name used in frame IDs and logs."},
      {"serial_number", "Camera serial number; empty selects automatically."},
      {"usb_port", "Camera USB port; empty selects automatically."},
      {"device_type", "Device type: camera or lidar."},
      {"device_num", "Number of cameras expected by the driver."},
      {"device_access_mode",
       "Camera access mode: Default, EA (exclusive), CA (control), or MR (monitor)."},
      {"net_device_ip", "Network camera IP address; empty uses discovery."},
      {"net_device_port", "Network camera port."},
      {"enumerate_net_device", "Discover network cameras."},
      {"uvc_backend", "UVC transport backend, such as libuvc."},
      {"force_ip_enable", "Apply the configured IP address to the camera selected by MAC."},
      {"force_ip_mac", "MAC address of the camera targeted by force-IP configuration."},
      {"force_ip_address", "IPv4 address to assign when force_ip_enable is true."},
      {"force_ip_subnet_mask", "IPv4 subnet mask to assign with force-IP configuration."},
      {"force_ip_gateway", "IPv4 gateway to assign with force-IP configuration."},
      {"log_level", "SDK and ROS log level."},
      {"log_file_name", "SDK log filename; empty generates a name."},
      {"bag_filename", "Orbbec .bag playback file; empty connects to a camera."},
      {"bag_loop", "Repeat playback when the end of the recording is reached."},
      {"bag_record_filename", "Orbbec .bag recording path; empty disables recording."},
      {"bag_record_compression", "Compress Orbbec .bag recordings."},
      {"connection_delay", "Delay before connecting to a detected camera, in milliseconds."},
      {"enable_sync_host_time", "Periodically synchronize the camera clock with the host."},
      {"time_sync_period", "Host-to-camera clock synchronization interval, in seconds."},
      {"time_domain", "Frame timestamp source: global, device, or system."},
      {"timestamp_clock_type", "Timestamp clock: realtime or monotonic; empty keeps the default."},
      {"diagnostic_period", "Diagnostic publication interval, in seconds; 0 disables diagnostics."},
      {"device_preset", "Device preset name; empty keeps the current preset."},
      {"device_preset_version", "Device preset version; empty uses the default version."},
      {"depth_work_mode", "Depth work mode name; empty keeps the current mode."},
      {"depth_registration", "Align depth and color using align_mode and align_target_stream."},
      {"align_mode", "Depth/color alignment implementation: HW or SW."},
      {"align_target_stream", "Alignment target: COLOR or DEPTH. HW alignment requires COLOR."},
      {"depth_colorizer_mode",
       "Published depth visualization: none, jet, jet_inv, or gray. none preserves raw depth."},
      {"disparity_to_depth_mode",
       "Disparity conversion: HW, SW, disable, or empty to keep the camera setting."},
      {"depth_precision",
       "Requested depth precision; supported values depend on the module. Empty skips setting."},
      {"preset_resolution_config",
       "Preset resolution configuration: width,height,IR decimation,depth decimation."},
      {"sync_mode",
       "Multi-camera synchronization mode; supported modes depend on the module. Empty skips "
       "setting."},
      {"software_trigger_period", "Software trigger interval, in milliseconds."},
      {"frames_per_trigger", "Number of frames captured for each trigger."},
      {"tf_publish_rate", "TF publication rate, in Hz; 0 publishes static TF."},
      {"cloud_frame_id", "Point-cloud frame ID; empty uses the selected camera frame."},
      {"ordered_pc", "Keep the image layout in the point cloud."},
      {"linear_accel_cov", "Diagonal covariance assigned to linear acceleration measurements."},
      {"angular_vel_cov", "Diagonal covariance assigned to angular velocity measurements."},
      {"max_save_images_count", "Maximum saved images per stream per save-images request."},
      {"frame_aggregate_mode", "Frame grouping mode: full_frame, color_frame, ANY, or disable."},
      {"load_config_json_file_path",
       "Camera JSON configuration file; empty skips loading. Explicit ROS settings take priority."},
      {"export_config_json_file_path", "Camera JSON export path; empty disables export."},
      {"frame_timestamp_csv_file", "Frame timestamp CSV path; empty disables recording."},
      {"monitor_poll_interval_sec",
       "Heartbeat and firmware-log polling interval, in seconds (1-10); -1 keeps the current "
       "interval."},
      {"lrm_obstacle_distance_publish_rate",
       "Obstacle-distance publication rate, in Hz; must be positive."},
      {"color_powerline_freq",
       "Color power-line frequency: disable, 50hz, 60hz or auto; empty keeps the current setting."},
      {"intra_camera_sync_reference", "Exposure synchronization reference: Start, Middle or End."},
      {"ae_reference_stream",
       "Auto-exposure reference stream: depth or color; empty keeps the current setting."},
      {"ae_strategy",
       "Auto-exposure strategy: default or motion; empty keeps the current setting."},
      {"exposure_range_mode",
       "Exposure range: regular or ultimate; default or empty keeps the current setting."},
      {"disparity_range_mode", "Disparity search range: 64, 128 or 256."},
      {"enable_color_auto_exposure", "Automatically adjust color exposure."},
      {"enable_depth_auto_exposure", "Automatically adjust depth exposure."},
      {"enable_ir_auto_exposure", "Automatically adjust infrared exposure."},
      {"color_exposure",
       "Color exposure; disable auto exposure for manual control. Units depend on the camera."},
      {"depth_exposure",
       "Depth exposure; disable auto exposure for manual control. Units depend on the camera."},
      {"ir_exposure",
       "Infrared exposure; disable auto exposure for manual control. Units depend on the camera."},
      {"color_gain", "Color sensor gain."},
      {"depth_gain", "Depth sensor gain."},
      {"ir_gain", "Infrared sensor gain."},
      {"enable_color_auto_white_balance", "Automatically adjust color white balance."},
      {"color_white_balance",
       "Color white balance; disable auto white balance for manual control."},
      {"depth_brightness", "Target brightness for depth/infrared automatic exposure."},
      {"ir_brightness", "Target brightness for depth/infrared automatic exposure."},
      {"mean_intensity_set_point", "Target brightness for depth/infrared automatic exposure."},
      {"color_preset", "Color preset name; empty keeps the current preset."},
      {"publish_tf", "Publish camera coordinate transforms."},
      {"color_ae_max_exposure", "Maximum color auto-exposure time; units depend on the camera."},
      {"color_ae_max_gain", "Maximum color auto-exposure gain."},
      {"ir_ae_max_exposure", "Maximum infrared auto-exposure time; units depend on the camera."},
      {"color_mjpeg_quality", "Color MJPEG compression quality."},
      {"color_brightness", "Color image brightness compensation."},
      {"color_roi_brightness", "Color region-of-interest brightness adjustment."},
      {"color_sharpness", "Color image sharpness."},
      {"color_gamma", "Color image gamma correction."},
      {"color_saturation", "Color image saturation."},
      {"color_contrast", "Color image contrast."},
      {"color_hue", "Color image hue."},
      {"color_backlight_compensation", "Color backlight compensation level."},
      {"color_anti_flicker", "Enable color anti-flicker control."},
      {"color_denoising_level", "Color image denoising level."},
      {"enable_color_auto_exposure_priority", "Enable color auto-exposure priority."},
      {"enable_depth_auto_exposure_priority", "Enable depth auto-exposure priority."},
      {"enable_ir_long_exposure", "Allow infrared long-exposure mode."},
      {"enable_ptp_config", "Enable PTP clock synchronization."},
      {"enable_ldp", "Enable laser protection."},
      {"ldp_power_level", "Laser power level."},
      {"enable_laser", "Enable the laser emitter."},
      {"retry_on_usb3_detection_failure",
       "Retry USB link detection when the camera connects at USB 2 speed."},
      {"laser_energy_level", "Laser emission energy level."},
      {"min_depth_limit", "Minimum camera depth limit; 0 skips setting at startup."},
      {"max_depth_limit", "Maximum camera depth limit; 0 skips setting at startup."},
      {"enable_heartbeat", "Enable camera heartbeat monitoring."},
      {"enable_fps_boost", "Enable the camera's frame-rate boost mode."},
      {"enable_accel_data_correction", "Apply SDK correction to accelerometer data."},
      {"enable_gyro_data_correction", "Apply SDK correction to gyroscope data."},
      {"disparity_search_offset", "Disparity search offset."},
      {"sync_io_voltage_level", "Synchronization I/O voltage level."},
      {"enable_sync_output_accel_gyro", "Publish synchronized accelerometer and gyroscope data."},
      {"enable_accel", "Enable accelerometer output."},
      {"enable_gyro", "Enable gyroscope output."},
      {"accel_rate", "Accelerometer sampling rate; empty selects the default profile."},
      {"gyro_rate", "Gyroscope sampling rate; empty selects the default profile."},
      {"accel_range", "Accelerometer measurement range; empty selects the default profile."},
      {"gyro_range", "Gyroscope measurement range; empty selects the default profile."},
      {"enable_point_cloud", "Publish the depth point cloud."},
      {"enable_colored_point_cloud",
       "Publish the colored point cloud; requires color, depth and alignment."},
      {"point_cloud_decimation_filter_factor",
       "Point-cloud decimation factor (1-8); 1 keeps all samples."},
      {"enable_d2c_viewer",
       "Publish a depth/color overlay on depth_to_color/image_raw; requires raw 16UC1 depth."},
      {"enable_depth_scale", "Scale depth after depth-to-color alignment."},
      {"enable_frame_sync", "Synchronize frames from different streams."},
      {"depth_delay_us", "Depth capture delay for multi-camera synchronization, in microseconds."},
      {"color_delay_us", "Color capture delay for multi-camera synchronization, in microseconds."},
      {"trigger2image_delay_us", "Trigger-to-capture delay, in microseconds."},
      {"trigger_out_delay_us", "Trigger-output delay, in microseconds."},
      {"trigger_out_enabled", "Enable synchronization trigger output."},
      {"software_trigger_enabled", "Enable periodic software triggering in software-trigger mode."},
      {"enable_firmware_log", "Collect camera firmware logs."},
      {"enable_frame_drop_log", "Log detected frame drops."},
      {"show_fps_enable", "Log stream frame rates."},
      {"enable_publish_extrinsic", "Publish camera extrinsic calibration."},
      {"enable_lrm_obstacle_distance_publish", "Publish laser-ranging obstacle distance."},
      {"enable_gmsl_trigger", "Enable host-generated GMSL triggers."},
      {"gmsl_trigger_fps", "GMSL trigger rate."},
      {"interleave_ae_mode", "Frame-interleave mode: hdr or laser; empty skips mode selection."},
      {"interleave_frame_enable", "Enable alternating frame configurations."},
      {"interleave_skip_enable", "Skip infrared frames with the selected interleave index."},
      {"interleave_skip_index", "Infrared interleave index to skip; -1 skips no frames."},
      {"disparity_offset_config", "Apply the two indexed disparity-offset settings."},
      {"offset_index0",
       "Disparity offset for configuration 0 (0-127); startup requires both offsets; -1 skips the "
       "pair."},
      {"offset_index1",
       "Disparity offset for configuration 1 (0-127); startup requires both offsets; -1 skips the "
       "pair."},
      {"depth_decimation_factor", "Depth hardware decimation factor; 1 keeps the original size."},
      {"left_ir_decimation_factor",
       "Left infrared hardware decimation factor; 1 keeps the original size."},
      {"right_ir_decimation_factor",
       "Right infrared hardware decimation factor; 1 keeps the original size."},
      {"enable_enhanced_depth", "Enable enhanced depth processing."},
      {"enhanced_depth_model_path",
       "Enhanced-depth model file; a valid path is required when enhanced depth is enabled."},
      {"enhanced_depth_confidence_threshold",
       "Enhanced-depth confidence threshold (0-255); -1 skips setting at startup."},
      {"enable_decimation_filter", "Enable depth-image downsampling."},
      {"enable_hdr_merge", "Merge depth frames captured with different exposures."},
      {"enable_sequence_id_filter", "Select depth frames by sequence ID."},
      {"enable_disparity_to_depth", "Convert disparity frames to depth."},
      {"enable_threshold_filter", "Remove depth values outside the configured interval."},
      {"enable_hardware_noise_removal_filter", "Enable hardware depth-noise removal."},
      {"enable_noise_removal_filter", "Enable software depth-noise removal."},
      {"enable_spatial_filter", "Enable advanced spatial depth smoothing."},
      {"enable_spatial_fast_filter", "Enable fast spatial depth smoothing."},
      {"enable_spatial_moderate_filter", "Enable moderate spatial depth smoothing."},
      {"enable_temporal_filter", "Smooth depth across consecutive frames."},
      {"enable_hole_filling_filter", "Fill missing depth pixels."},
      {"enable_edge_noise_removal_filter", "Remove depth-edge noise."},
      {"enable_mgc_noise_removal_filter", "Enable MGC depth-noise removal."},
      {"enable_lut_noise_removal_filter", "Enable LUT depth-noise removal."},
      {"enable_disp_outliers_filter", "Remove disparity outliers."},
      {"enable_false_positive_filter", "Remove false-positive depth measurements."},
      {"enable_false_positive_edge_bleed_filter",
       "Enable edge-bleed filtering; omitted keeps the current setting."},
      {"false_positive_filter_edge_bleed_roi_min_x_ratio",
       "Edge-bleed ROI minimum X ratio; -1 skips setting at startup."},
      {"false_positive_filter_edge_bleed_roi_max_x_ratio",
       "Edge-bleed ROI maximum X ratio; -1 skips setting at startup."},
      {"false_positive_filter_edge_bleed_roi_min_y_ratio",
       "Edge-bleed ROI minimum Y ratio; -1 skips setting at startup."},
      {"false_positive_filter_edge_bleed_roi_max_y_ratio",
       "Edge-bleed ROI maximum Y ratio; -1 skips setting at startup."},
      {"false_positive_filter_edge_bleed_min_bleed_length",
       "Minimum consecutive edge-bleed length, in pixels; -1 skips setting at startup."},
      {"enable_false_positive_texture_sparsity_filter",
       "Enable texture-sparsity filtering; omitted keeps the current setting."},
      {"false_positive_filter_texture_sparsity_roi_min_x_ratio",
       "Texture-sparsity ROI minimum X ratio; -1 skips setting at startup."},
      {"false_positive_filter_texture_sparsity_roi_max_x_ratio",
       "Texture-sparsity ROI maximum X ratio; -1 skips setting at startup."},
      {"false_positive_filter_texture_sparsity_roi_min_y_ratio",
       "Texture-sparsity ROI minimum Y ratio; -1 skips setting at startup."},
      {"false_positive_filter_texture_sparsity_roi_max_y_ratio",
       "Texture-sparsity ROI maximum Y ratio; -1 skips setting at startup."},
      {"false_positive_filter_texture_sparsity_max_noise_level",
       "Maximum depth of texture-sparsity noise; -1 skips setting at startup."},
      {"false_positive_filter_texture_sparsity_max_speckle_size",
       "Maximum texture-sparsity speckle size; -1 skips setting at startup."},
      {"enable_false_positive_pattern_ambiguity_filter",
       "Enable pattern-ambiguity filtering; omitted keeps the current setting."},
      {"false_positive_filter_pattern_ambiguity_roi_min_x_ratio",
       "Pattern-ambiguity ROI minimum X ratio; -1 skips setting at startup."},
      {"false_positive_filter_pattern_ambiguity_roi_max_x_ratio",
       "Pattern-ambiguity ROI maximum X ratio; -1 skips setting at startup."},
      {"false_positive_filter_pattern_ambiguity_roi_min_y_ratio",
       "Pattern-ambiguity ROI minimum Y ratio; -1 skips setting at startup."},
      {"false_positive_filter_pattern_ambiguity_roi_max_y_ratio",
       "Pattern-ambiguity ROI maximum Y ratio; -1 skips setting at startup."},
      {"false_positive_filter_pattern_ambiguity_max_noise_level",
       "Maximum depth of pattern-ambiguity noise; -1 skips setting at startup."},
      {"false_positive_filter_pattern_ambiguity_max_speckle_size",
       "Maximum pattern-ambiguity speckle size; -1 skips setting at startup."},
      {"false_positive_filter_pattern_ambiguity_max_width_ratio",
       "Maximum pattern-ambiguity noise width as a frame-width ratio; -1 skips setting at "
       "startup."},
      {"false_positive_filter_pattern_ambiguity_max_height_ratio",
       "Maximum pattern-ambiguity noise height as a frame-height ratio; -1 skips setting at "
       "startup."},
      {"false_positive_filter_pattern_ambiguity_tolerance",
       "Pattern-ambiguity tolerance; lower values filter weaker ambiguities. -1 skips setting at "
       "startup."},
      {"false_positive_filter_pattern_ambiguity_score",
       "Pattern-ambiguity score; lower values filter weaker ambiguities. -1 skips setting at "
       "startup."},
      {"disp_outliers_filter_search_mode",
       "Disparity-outlier search mode: FULL or OFFSET_80; empty skips setting at startup."},
      {"decimation_filter_scale", "Depth-image downsampling factor; -1 skips setting at startup."},
      {"sequence_id_filter_id", "Depth-frame sequence ID to select; -1 skips setting at startup."},
      {"threshold_filter_min", "Lower depth threshold; -1 skips setting this boundary at startup."},
      {"threshold_filter_max", "Upper depth threshold; -1 skips setting this boundary at startup."},
      {"hardware_noise_removal_filter_threshold",
       "Hardware depth-noise threshold; -1 skips setting at startup."},
      {"noise_removal_filter_min_diff",
       "Depth-difference threshold for noise removal; -1 skips setting at startup."},
      {"noise_removal_filter_max_size",
       "Maximum speckle size for noise removal; -1 skips setting at startup."},
      {"spatial_filter_alpha",
       "Current-pixel weight for advanced spatial smoothing. Startup: set all 4 spatial_filter "
       "values; -1 skips the group."},
      {"spatial_filter_diff_threshold",
       "Edge-preserving difference threshold for advanced spatial smoothing. Startup: set all 4 "
       "spatial_filter values; -1 skips the group."},
      {"spatial_filter_magnitude",
       "Advanced spatial-filter smoothing iterations. Startup: set all 4 spatial_filter values; -1 "
       "skips the group."},
      {"spatial_filter_radius",
       "Advanced spatial-filter hole-filling radius. Startup: set all 4 spatial_filter values; -1 "
       "skips the group."},
      {"spatial_fast_filter_radius",
       "Fast spatial-filter window size; -1 skips setting at startup."},
      {"spatial_moderate_filter_diff_threshold",
       "Edge-preserving difference threshold for moderate spatial smoothing. Startup: set all 3 "
       "spatial_moderate_filter values; -1 skips the group."},
      {"spatial_moderate_filter_magnitude",
       "Moderate spatial-filter smoothing iterations. Startup: set all 3 spatial_moderate_filter "
       "values; -1 skips the group."},
      {"spatial_moderate_filter_radius",
       "Moderate spatial-filter window size. Startup: set all 3 spatial_moderate_filter values; -1 "
       "skips the group."},
      {"temporal_filter_diff_threshold",
       "Difference scale between consecutive frames. Startup: set both temporal_filter values; -1 "
       "skips the group."},
      {"temporal_filter_weight",
       "Current-frame weight for temporal smoothing. Startup: set both temporal_filter values; -1 "
       "skips the group."},
      {"hole_filling_filter_mode",
       "Hole filling: FILL_TOP, FILL_NEAREST or FILL_FAREST; empty skips setting at startup."},
      {"hdr_merge_exposure_1",
       "HDR frame 1 exposure. Startup: set all 4 hdr_merge values; -1 skips the group."},
      {"hdr_merge_gain_1",
       "HDR frame 1 gain. Startup: set all 4 hdr_merge values; -1 skips the group."},
      {"hdr_merge_exposure_2",
       "HDR frame 2 exposure. Startup: set all 4 hdr_merge values; -1 skips the group."},
      {"hdr_merge_gain_2",
       "HDR frame 2 gain. Startup: set all 4 hdr_merge values; -1 skips the group."},
      {"color_frame_queue_max_frames",
       "Color processing queue capacity, in frames; must be at least 1."},
      {"enable_color_decimation_filter", "Enable color-image downsampling."},
      {"color_decimation_filter_scale",
       "Color-image downsampling factor; -1 skips setting at startup."},
      {"left_color_frame_queue_max_frames",
       "Left color processing queue capacity, in frames; must be at least 1."},
      {"enable_left_color_decimation_filter", "Enable left color-image downsampling."},
      {"left_color_decimation_filter_scale",
       "Left color-image downsampling factor; -1 skips setting at startup."},
      {"right_color_frame_queue_max_frames",
       "Right color processing queue capacity, in frames; must be at least 1."},
      {"enable_right_color_decimation_filter", "Enable right color-image downsampling."},
      {"right_color_decimation_filter_scale",
       "Right color-image downsampling factor; -1 skips setting at startup."},
      {"enable_left_ir_sequence_id_filter", "Select left infrared frames by sequence ID."},
      {"left_ir_sequence_id_filter_id",
       "Left infrared-frame sequence ID to select; -1 skips setting at startup."},
      {"enable_right_ir_sequence_id_filter", "Select right infrared frames by sequence ID."},
      {"right_ir_sequence_id_filter_id",
       "Right infrared-frame sequence ID to select; -1 skips setting at startup."},
      {"hdr_index0_laser_control",
       "HDR configuration 0: laser control value; -1 skips setting at startup."},
      {"hdr_index0_depth_exposure",
       "HDR configuration 0: depth exposure; -1 skips setting at startup."},
      {"hdr_index0_depth_gain", "HDR configuration 0: depth gain; -1 skips setting at startup."},
      {"hdr_index0_ir_brightness",
       "HDR configuration 0: infrared auto-exposure target brightness; -1 skips setting at "
       "startup."},
      {"hdr_index0_ir_ae_max_exposure",
       "HDR configuration 0: maximum infrared auto-exposure time; -1 skips setting at startup."},
      {"hdr_index1_laser_control",
       "HDR configuration 1: laser control value; -1 skips setting at startup."},
      {"hdr_index1_depth_exposure",
       "HDR configuration 1: depth exposure; -1 skips setting at startup."},
      {"hdr_index1_depth_gain", "HDR configuration 1: depth gain; -1 skips setting at startup."},
      {"hdr_index1_ir_brightness",
       "HDR configuration 1: infrared auto-exposure target brightness; -1 skips setting at "
       "startup."},
      {"hdr_index1_ir_ae_max_exposure",
       "HDR configuration 1: maximum infrared auto-exposure time; -1 skips setting at startup."},
      {"laser_index0_laser_control",
       "Laser on/off configuration 0: laser control value; -1 skips setting at startup."},
      {"laser_index0_depth_exposure",
       "Laser on/off configuration 0: depth exposure; -1 skips setting at startup."},
      {"laser_index0_depth_gain",
       "Laser on/off configuration 0: depth gain; -1 skips setting at startup."},
      {"laser_index0_ir_brightness",
       "Laser on/off configuration 0: infrared auto-exposure target brightness; -1 skips setting "
       "at startup."},
      {"laser_index0_ir_ae_max_exposure",
       "Laser on/off configuration 0: maximum infrared auto-exposure time; -1 skips setting at "
       "startup."},
      {"laser_index1_laser_control",
       "Laser on/off configuration 1: laser control value; -1 skips setting at startup."},
      {"laser_index1_depth_exposure",
       "Laser on/off configuration 1: depth exposure; -1 skips setting at startup."},
      {"laser_index1_depth_gain",
       "Laser on/off configuration 1: depth gain; -1 skips setting at startup."},
      {"laser_index1_ir_brightness",
       "Laser on/off configuration 1: infrared auto-exposure target brightness; -1 skips setting "
       "at startup."},
      {"laser_index1_ir_ae_max_exposure",
       "Laser on/off configuration 1: maximum infrared auto-exposure time; -1 skips setting at "
       "startup."},
      {"depth_filter_config", "Reserved parameter; currently has no effect."},
      {"enable_color", "Enable the color stream."},
      {"enable_left_color", "Enable the left color stream."},
      {"enable_right_color", "Enable the right color stream."},
      {"enable_depth", "Enable the depth stream."},
      {"enable_ir", "Enable the infrared stream."},
      {"enable_left_ir", "Enable the left infrared stream."},
      {"enable_right_ir", "Enable the right infrared stream."},
  };
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.additional_constraints =
      isRuntimeCameraParameter(name)
          ? "Change while streaming when supported; update one related group at a time."
          : "Startup only; restart the node to change.";
  const auto it = descriptions.find(name);
  if (it != descriptions.end()) {
    descriptor.description = it->second;
    return descriptor;
  }
  std::string label = name;
  std::replace(label.begin(), label.end(), '_', ' ');
  descriptor.description =
      name.rfind("enable_", 0) == 0 ? "Enable " + label.substr(7) + "." : "Set " + label + ".";
  if (endsWith(name, "_width") || endsWith(name, "_height")) {
    descriptor.description = label + ", in pixels; 0 selects the default profile.";
  } else if (endsWith(name, "_fps")) {
    descriptor.description = label + ", in frames per second; 0 selects the default profile.";
  } else if (endsWith(name, "_qos_depth")) {
    descriptor.description =
        "QoS history depth; -1 uses the selected profile, otherwise must be positive.";
  } else if (endsWith(name, "_qos_history")) {
    descriptor.description = "QoS history policy: DEFAULT, KEEP_LAST or KEEP_ALL.";
  } else if (endsWith(name, "_qos")) {
    descriptor.description = label + " profile; default uses the driver default.";
  } else if (endsWith(name, "_optical_frame_id")) {
    descriptor.description = "Optical coordinate frame ID for this stream.";
  } else if (endsWith(name, "_frame_id")) {
    descriptor.description = "Coordinate frame ID for this stream.";
  } else if (endsWith(name, "_format")) {
    descriptor.description =
        "Pixel format for this stream; must match its resolution and frame rate.";
  } else if (endsWith(name, "_mirror")) {
    descriptor.description = "Mirror this stream horizontally.";
  } else if (endsWith(name, "_flip")) {
    descriptor.description = "Flip this stream vertically.";
  } else if (endsWith(name, "_rotation")) {
    descriptor.description = "Image rotation, in degrees: 0, 90, 180 or 270.";
  } else if (name.find("_ae_roi_") != std::string::npos) {
    const auto boundary = name.substr(name.rfind('_') + 1);
    descriptor.description = "Auto-exposure ROI " + boundary +
                             " boundary, in pixels; -1 uses the image boundary. Each boundary can "
                             "be changed separately.";
  } else if (endsWith(name, "_info_url")) {
    descriptor.description = "Calibration URL; empty uses camera calibration.";
  } else if (endsWith(name, "_undistortion")) {
    descriptor.description = "Correct lens distortion for this stream.";
  }
  return descriptor;
}
}  // namespace orbbec_camera
