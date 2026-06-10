# Tool Index

This page is an index of common OrbbecSDK_ROS2 tools. It only lists each tool's purpose and the page that contains the full usage notes. Before running `ros2 run` commands, source ROS 2 and the workspace:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

## Tool Overview

| Tool | Scenario / Full Guide | Summary |
| --- | --- | --- |
| `list_devices_node` (Recommended) | <a href="device_query_tools.html">Device query and basic maintenance</a> | Enumerates Orbbec devices and prints device, firmware, preset, and IP status. |
| `list_depth_work_mode_node` | <a href="device_query_tools.html">Device query and basic maintenance</a> | Lists the depth work modes supported by the current device. |
| `list_camera_profile_mode_node` (Recommended) | <a href="device_query_tools.html">Device query and basic maintenance</a> | Lists supported stream profiles, depth work modes, and presets. |
| `list_ob_devices.sh` | <a href="device_query_tools.html">Device query and basic maintenance</a> | Scans Orbbec USB devices from Linux USB sysfs. |
| `install_udev_rules.sh` | <a href="device_query_tools.html">Device query and basic maintenance</a> | Installs USB device udev rules. |
| `firmware_update_tool` (Recommended) | <a href="firmware_update_tool.html">Device maintenance</a> | Updates device firmware or burns preset files. |
| `ip_config_tool` (Recommended) | <a href="network_config_tools.html">Network configuration</a> | Configures DHCP, persistent IP, and Force IP for network cameras. |
| `common_benchmark_node.py` (Recommended) | <a href="benchmark_tools.html">Performance benchmark</a> | Collects FPS, latency, CPU, memory, and frame drop statistics. |
| `service_benchmark_node.py` | <a href="benchmark_tools.html">Performance benchmark</a> | Measures service call latency and success rate. |
| `ob_benchmark_node` | <a href="benchmark_tools.html">Performance benchmark</a> | Runs benchmark configurations periodically and records CPU/memory data. |
| `start_benchmark_node` | <a href="benchmark_tools.html">Performance benchmark</a> | Multi-topic subscriber used by the benchmark workflow. |
| `enable_frame_drop_log` / `frame_timestamp_csv_file` (Recommended) | <a href="diagnostic_tools.html">Performance diagnostics</a> | Built-in camera node frame drop logging and timestamp CSV recording. |
| `topic_statistics_node` | <a href="diagnostic_tools.html">Performance diagnostics</a> | Collects image topic age and period statistics. |
| `frame_latency_node` | <a href="diagnostic_tools.html">Performance diagnostics</a> | Measures latency and FPS for a specified topic. |
| `monitor_fd.sh` | <a href="diagnostic_tools.html">Performance diagnostics</a> | Monitors the file descriptor count of `component_container`. |
| `plot_stat.py` | <a href="diagnostic_tools.html">Performance diagnostics</a> | Plots topic statistics curves from `statistics.csv`. |
| `receive_pc.py` | <a href="diagnostic_tools.html">Debug helper</a> | Verifies whether a Python node can receive the point cloud topic. |
| `multi_save_rgbir_node` (Recommended) | <a href="multi_camera_tools.html">Multi-camera</a> | Saves RGB/IR images from multiple cameras. |
| `image_sync_example_node` (Recommended) | <a href="multi_camera_tools.html">Multi-camera</a> | Displays synchronized images and reports multi-topic timestamp statistics. |
| `SyncFramesMain.py` | <a href="multi_camera_tools.html">Multi-camera</a> | Offline analysis script for multi-camera synchronization verification. |
| `group_image.py` | <a href="multi_camera_tools.html">Multi-camera</a> | Groups saved multi-camera images by timestamp. |
| `static_transforms_publisher.py` | <a href="multi_camera_tools.html">Multi-camera debugging</a> | Publishes hard-coded static TFs for multi-camera debugging. |
