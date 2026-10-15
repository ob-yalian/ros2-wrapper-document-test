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
      {"device_type", "Device implementation: camera or lidar."},
      {"device_num", "Number of cameras expected by the driver."},
      {"device_access_mode",
       "SDK device access mode: Default, EA (exclusive), CA (control), or MR (monitor)."},
      {"net_device_ip", "IP address of the network camera; empty uses device discovery."},
      {"net_device_port", "Port used to connect to the network camera."},
      {"enumerate_net_device", "Enable SDK network device discovery."},
      {"uvc_backend", "UVC transport backend, such as libuvc."},
      {"force_ip_enable", "Apply the configured IP address to the camera selected by MAC."},
      {"force_ip_mac", "MAC address of the camera targeted by force-IP configuration."},
      {"force_ip_address", "IPv4 address to assign when force_ip_enable is true."},
      {"force_ip_subnet_mask", "IPv4 subnet mask to assign with force-IP configuration."},
      {"force_ip_gateway", "IPv4 gateway to assign with force-IP configuration."},
      {"log_level", "Logging severity for the SDK and ROS driver."},
      {"log_file_name", "SDK log filename; empty generates a filename automatically."},
      {"bag_filename", "SDK recording to play back; empty connects to a physical device."},
      {"bag_loop", "Repeat SDK bag playback when the end of the recording is reached."},
      {"bag_record_filename", "SDK recording output path; empty disables recording."},
      {"bag_record_compression", "Enable compression when recording an SDK bag."},
      {"connection_delay", "Delay before connecting to a detected camera, in milliseconds."},
      {"enable_sync_host_time", "Periodically synchronize the device clock with the host."},
      {"time_sync_period", "Host-to-device clock synchronization period, in seconds."},
      {"time_domain", "Frame timestamp source: global, device, or system."},
      {"timestamp_clock_type", "SDK timestamp clock selection; empty keeps SDK behavior."},
      {"diagnostic_period", "Diagnostic publication period, in seconds."},
      {"device_preset",
       "Device preset name; empty keeps the device preset. Available presets depend on the "
       "module."},
      {"device_preset_version", "Version of the selected device preset; empty keeps SDK behavior."},
      {"depth_work_mode",
       "Legacy depth work-mode selector for modules using that SDK interface; empty skips "
       "setting."},
      {"depth_registration", "Align depth and color using align_mode and align_target_stream."},
      {"align_mode", "Depth/color alignment implementation: HW or SW."},
      {"align_target_stream", "Alignment target: COLOR or DEPTH. HW alignment requires COLOR."},
      {"depth_colorizer_mode",
       "Published depth visualization: none, jet, jet_inv, or gray. none preserves raw depth."},
      {"disparity_to_depth_mode",
       "Disparity conversion: HW, SW, disable, or empty to keep the SDK setting."},
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
      {"frame_aggregate_mode",
       "SDK frameset aggregation: full_frame, color_frame, ANY, or disable."},
      {"load_config_json_file_path",
       "SDK configuration JSON to load before explicit ROS overrides; empty disables loading."},
      {"export_config_json_file_path",
       "Path to export the initialized SDK configuration; empty disables export."},
      {"frame_timestamp_csv_file",
       "Frame timestamp CSV output path; empty disables timestamp recording."},
      {"monitor_poll_interval_sec",
       "Device monitor polling interval in seconds (1 to 10); -1 keeps the SDK setting."},
      {"lrm_obstacle_distance_publish_rate",
       "Laser-ranging obstacle distance publication rate in Hz; must be positive."},
      {"color_powerline_freq",
       "Color power-line frequency selector; empty skips setting. Values depend on the module."},
      {"intra_camera_sync_reference",
       "Intra-camera synchronization reference: Start, Middle, or End."},
      {"ae_reference_stream", "Auto-exposure reference stream; empty skips setting."},
      {"ae_strategy", "Device auto-exposure strategy; empty skips setting."},
      {"exposure_range_mode", "Device exposure/performance mode; empty skips setting."},
      {"disparity_range_mode",
       "Disparity search range: 64, 128 or 256; -1 skips setting at launch. "
       "The SDK property represents these values as indices 0, 1 and 2."},
      {"mean_intensity_set_point",
       "Depth/IR auto-exposure target brightness, sharing the IR brightness property."},
  };
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.additional_constraints =
      "Initialization only; restart the camera to change this parameter.";
  if (isRuntimeCameraParameter(name)) {
    descriptor.additional_constraints =
        "Runtime writable without stopping streams when the connected device/filter supports it. "
        "Runtime types are fixed. Update one control group at a time; ROI/filter settings are "
        "applied as a group. Optional skip values (-1/empty) are launch-only. The ROS value is "
        "requested configuration, not a live device reading.";
  }
  const auto it = descriptions.find(name);
  if (it != descriptions.end()) {
    descriptor.description = it->second;
    return descriptor;
  }
  std::string label = name;
  std::replace(label.begin(), label.end(), '_', ' ');
  descriptor.description = "Configure " + label + ".";
  if (endsWith(name, "_width") || endsWith(name, "_height")) {
    descriptor.description += " Image dimension in pixels; zero selects an SDK stream profile.";
  } else if (endsWith(name, "_fps")) {
    if (name != "gmsl_trigger_fps") {
      descriptor.description += " Frames per second; zero selects an SDK stream profile.";
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
    descriptor.description += " SDK pixel format name; must match a supported stream profile.";
  } else if (endsWith(name, "_delay_us")) {
    descriptor.description += " Delay in microseconds.";
  } else if (endsWith(name, "_frame_queue_max_frames")) {
    descriptor.description += " Maximum queued frames; must be at least one.";
  } else if (endsWith(name, "_exposure") || endsWith(name, "_gain")) {
    descriptor.description += " Values and units follow the connected module's SDK property.";
  } else if (name.find("_ae_roi_") != std::string::npos) {
    descriptor.description +=
        " Coordinate of the auto-exposure region of interest; launch value -1 leaves this "
        "coordinate unspecified. Runtime updates may change a single coordinate; unspecified "
        "coordinates use the corresponding full-frame boundaries.";
  } else if (endsWith(name, "_info_url")) {
    descriptor.description += " Camera calibration URL; empty uses device calibration.";
  } else if (name.find("filter") != std::string::npos && name.find("enable_") != 0) {
    descriptor.description +=
        " Filter settings and valid values depend on the SDK filter; -1 or empty skips an optional "
        "setting.";
  }
  return descriptor;
}
}  // namespace orbbec_camera
