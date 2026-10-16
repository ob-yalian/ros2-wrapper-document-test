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
      {"camera_name", "Camera name used to construct frame IDs and identify the camera."},
      {"serial_number", "Select a camera by serial number; empty allows automatic selection."},
      {"usb_port", "Select a camera by USB port; empty does not constrain the port."},
      {"device_type", "Device type: camera or lidar."},
      {"device_num", "Number of cameras expected by the driver."},
      {"device_access_mode",
       "Camera access mode: Default, EA (exclusive), CA (control), or MR (monitor)."},
      {"net_device_ip", "IP address of the network camera; empty uses device discovery."},
      {"net_device_port", "Port used to connect to the network camera."},
      {"enumerate_net_device", "Discover network cameras."},
      {"uvc_backend", "UVC transport backend, such as libuvc."},
      {"force_ip_enable", "Apply the configured IP address to the camera selected by MAC."},
      {"force_ip_mac", "MAC address of the camera targeted by force-IP configuration."},
      {"force_ip_address", "IPv4 address to assign when force_ip_enable is true."},
      {"force_ip_subnet_mask", "IPv4 subnet mask to assign with force-IP configuration."},
      {"force_ip_gateway", "IPv4 gateway to assign with force-IP configuration."},
      {"log_level", "Log level for camera and ROS messages."},
      {"log_file_name", "Camera log filename; empty generates a filename automatically."},
      {"bag_filename", "Orbbec .bag file to play; empty connects to a physical camera."},
      {"bag_loop", "Repeat playback when the end of the recording is reached."},
      {"bag_record_filename", "Orbbec .bag recording output path; empty disables recording."},
      {"bag_record_compression", "Compress recorded Orbbec .bag files."},
      {"connection_delay", "Delay before connecting to a detected camera, in milliseconds."},
      {"enable_sync_host_time", "Periodically synchronize the device clock with the host."},
      {"time_sync_period", "Host-to-device clock synchronization period, in seconds."},
      {"time_domain", "Frame timestamp source: global, device, or system."},
      {"timestamp_clock_type", "Timestamp clock: realtime or monotonic; empty keeps the default."},
      {"diagnostic_period", "Diagnostic publication period, in seconds."},
      {"device_preset",
       "Device preset name; empty keeps the device preset. Available presets depend on the "
       "module."},
      {"device_preset_version", "Version of the selected device preset; empty uses the default."},
      {"depth_work_mode",
       "Depth work mode. Available modes depend on the camera; empty keeps the current mode."},
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
      {"software_trigger_period", "Software trigger timer period, in milliseconds."},
      {"frames_per_trigger", "Number of frames captured for each trigger."},
      {"tf_publish_rate", "TF publication rate in Hz; zero publishes static transforms."},
      {"cloud_frame_id", "Point-cloud frame ID override; empty uses the selected camera frame."},
      {"ordered_pc", "Preserve image organization in published point clouds."},
      {"linear_accel_cov", "Diagonal covariance assigned to linear acceleration measurements."},
      {"angular_vel_cov", "Diagonal covariance assigned to angular velocity measurements."},
      {"max_save_images_count", "Maximum number of images saved per save-images request."},
      {"frame_aggregate_mode", "Frame grouping mode: full_frame, color_frame, ANY, or disable."},
      {"load_config_json_file_path",
       "Camera configuration JSON to load at startup. Explicit ROS settings take priority; "
       "empty disables loading."},
      {"export_config_json_file_path",
       "Path to export the camera configuration after startup; empty disables export."},
      {"frame_timestamp_csv_file",
       "Frame timestamp CSV output path; empty disables timestamp recording."},
      {"monitor_poll_interval_sec",
       "Device monitor polling interval in seconds (1 to 10); -1 keeps the camera setting."},
      {"lrm_obstacle_distance_publish_rate",
       "Laser-ranging obstacle distance publication rate in Hz; must be positive."},
      {"color_powerline_freq",
       "Color power-line frequency: disable, 50hz, 60hz or auto, when supported. Empty keeps the "
       "camera setting."},
      {"intra_camera_sync_reference",
       "Intra-camera synchronization reference: Start, Middle, or End."},
      {"ae_reference_stream", "Auto-exposure reference: depth or color; empty keeps the setting."},
      {"ae_strategy", "Auto-exposure strategy: default or motion; empty keeps the setting."},
      {"exposure_range_mode",
       "Exposure range mode: regular or ultimate; default or empty keeps the camera setting."},
      {"disparity_range_mode", "Disparity search range: 64, 128 or 256."},
      {"enable_color_auto_exposure", "Automatically adjust color exposure."},
      {"enable_depth_auto_exposure", "Automatically adjust depth exposure."},
      {"enable_ir_auto_exposure", "Automatically adjust infrared exposure."},
      {"color_exposure",
       "Color exposure. Disable automatic exposure for manual control. Units depend on the "
       "camera."},
      {"depth_exposure",
       "Depth exposure. Disable automatic exposure for manual control. Units depend on the "
       "camera."},
      {"ir_exposure",
       "Infrared exposure. Disable automatic exposure for manual control. Units depend on the "
       "camera."},
      {"color_gain", "Color image gain. Higher values brighten the image and may increase noise."},
      {"depth_gain", "Sensor gain for depth capture. Higher values may increase noise."},
      {"ir_gain", "Infrared image gain. Higher values brighten the image and may increase noise."},
      {"enable_color_auto_white_balance", "Automatically adjust color white balance."},
      {"color_white_balance", "Color white balance. Disable automatic white balance to set it."},
      {"depth_brightness", "Target brightness for depth/infrared automatic exposure."},
      {"ir_brightness", "Target brightness for depth/infrared automatic exposure."},
      {"mean_intensity_set_point", "Target brightness for depth/infrared automatic exposure."},
  };
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.additional_constraints =
      "Set at startup. Restart the camera node to change this setting.";
  if (isRuntimeCameraParameter(name)) {
    descriptor.additional_constraints =
        "Can be changed while streaming when supported by the camera. Change one setting or "
        "related group at a time. Keep-current values (-1 or empty) are only accepted at startup.";
  }
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
    descriptor.description += " Image dimension in pixels; zero selects the default.";
  } else if (endsWith(name, "_fps")) {
    if (name != "gmsl_trigger_fps") {
      descriptor.description += " Frames per second; zero selects the default.";
    }
  } else if (endsWith(name, "_qos_depth")) {
    descriptor.description +=
        " QoS history depth; -1 uses the selected QoS profile, otherwise must be positive.";
  } else if (endsWith(name, "_qos_history")) {
    descriptor.description += " DEFAULT, KEEP_LAST, or KEEP_ALL.";
  } else if (endsWith(name, "_qos")) {
    descriptor.description += " ROS QoS profile name; default uses the driver default profile.";
  } else if (endsWith(name, "_frame_id")) {
    descriptor.description += " ROS TF frame ID for this stream.";
  } else if (endsWith(name, "_format")) {
    descriptor.description +=
        " Pixel format; must be supported at the selected resolution and FPS.";
  } else if (endsWith(name, "_delay_us")) {
    descriptor.description += " Delay in microseconds.";
  } else if (endsWith(name, "_frame_queue_max_frames")) {
    descriptor.description += " Maximum queued frames; must be at least one.";
  } else if (endsWith(name, "_exposure") || endsWith(name, "_gain")) {
    descriptor.description += " Units and supported values depend on the camera.";
  } else if (name.find("_ae_roi_") != std::string::npos) {
    descriptor.description +=
        " Auto-exposure region boundary, in pixels. At startup, -1 uses the image boundary. "
        "Individual boundaries can be changed while streaming.";
  } else if (endsWith(name, "_info_url")) {
    descriptor.description += " Camera calibration URL; empty uses device calibration.";
  } else if (name.find("filter") != std::string::npos && name.find("enable_") != 0) {
    descriptor.description +=
        " Supported values depend on the filter. At startup, -1 or empty keeps the current "
        "setting where supported.";
  }
  return descriptor;
}
}  // namespace orbbec_camera
