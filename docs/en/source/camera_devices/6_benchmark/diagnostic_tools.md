# Diagnostic and Latency Analysis Tools

This section describes frame continuity, timestamp, topic statistics, end-to-end latency, and debugging helper tools.

## Frame Drop Log and Timestamp CSV Recording

After `enable_frame_drop_log` is enabled, the camera node prints color and depth frame drop statistics in the log. This helps locate frame drops at the SDK receive stage and ROS publish stage. When `frame_timestamp_csv_file` is set, the camera node also records color and depth frame timestamp data to a CSV file for analyzing frame continuity, publish latency, and timestamp anomalies.

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
enable_frame_drop_log:=true \
frame_timestamp_csv_file:=/tmp/frame_timestamp.csv
```

The CSV contains SDK frame index, hardware frame number, sensor timestamp, device/global/system timestamp, steady arrival/publish delta, ROS publish duration, and SDK delay fields.

### Field Description

The current CSV contains two groups of fields with the same structure, prefixed by `color_` and `depth_`, for example `color_sdk_frame_index` and `depth_sdk_frame_index`. The two groups have identical definitions and differ only in data source.

| Field suffix | Description | Unit / Notes |
| --- | --- | --- |
| `_sdk_frame_index` | SDK frame index | `frame->index()` |
| `_hardware_frame_number` | Hardware frame number | `frame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER)` |
| `_sensor_ts_sec` | Sensor timestamp | Seconds, usually exposure midpoint |
| `_sensor_ts_delta_us` | Delta between adjacent sensor timestamps | us |
| `_device_ts_sec` | Device clock timestamp | Seconds |
| `_device_ts_delta_us` | Delta between adjacent device timestamps | us |
| `_global_ts_sec` | Global timestamp | Seconds |
| `_global_ts_delta_us` | Delta between adjacent global timestamps | us |
| `_system_ts_sec` | SDK system timestamp | Seconds |
| `_system_ts_delta_us` | Delta between adjacent SDK system timestamps | us |
| `_arrival_steady_delta_us` | Delta between adjacent host steady arrival times | us |
| `_publish_steady_delta_us` | Delta between adjacent host steady times before publishing | us |
| `_arrival_to_publish_steady_us` | Time from ROS receiving the frame to publishing it (steady) | `publish_steady - arrival_steady` |
| `_sdk_delay_from_global_us` | SDK publish delay (global reference) | `arrival_system - global_ts` |
| `_sdk_delay_from_system_us` | SDK publish delay (system reference) | `arrival_system - sdk_system_ts` |

### Analysis Methods

#### Hardware Frame Drop Detection

- Check whether `_hardware_frame_number` is continuous.
- Plot `_sensor_ts_delta_us` as a line chart or scatter plot and check for obvious jumps.
- For example, at 30 fps, the adjacent frame interval should usually be close to 33333 us.

#### SDK / ROS Frame Drop Detection

- Check whether `_sdk_frame_index` is continuous.
- Plot `_device_ts_delta_us`, `_global_ts_delta_us`, and `_system_ts_delta_us` as line charts or scatter plots and check for jumps.
- After `enable_frame_drop_log` is enabled, `stage=SDK_RECEIVE` in the log indicates frame drops detected at the SDK receive stage, and `stage=ROS_PUBLISH` indicates frame drops detected at the ROS publish stage.

#### Latency Analysis

- SDK latency: check `_sdk_delay_from_global_us` and `_sdk_delay_from_system_us` as line charts or scatter plots to observe delay changes from the low-level timestamp to arrival at the ROS node.
- ROS latency: check `_arrival_to_publish_steady_us` as a line chart or scatter plot to measure the time from the SDK callback receiving a frame to publishing the image on the ROS side.
- If you need a value closer to real processing time, prefer fields related to the steady clock.

#### Synchronization Note

This CSV is mainly used to analyze continuity and latency of a single color or depth stream. It cannot directly measure synchronization between color and depth.

## topic_statistics_node

`topic_statistics_node` uses ROS 2 topic statistics to collect image topic age and period statistics and writes `statistics.csv` in the current directory. The camera node must be running before using it.

```bash
ros2 run orbbec_camera topic_statistics_node \
--ros-args \
-p image_topic:=/camera/color/image_raw \
-p statistics_topic:=/statistics
```

## frame_latency_node

`frame_latency_node` subscribes to a specified topic and calculates end-to-end latency from the message header stamp, while also printing FPS. It supports `image`, `points`, `imu`, `metadata`, `camera_info`, `rgbd`, `imu_info`, and `tf` topic types. The camera node must be running before using it.

```bash
ros2 run orbbec_camera frame_latency_node \
--ros-args \
-p topic_name:=/camera/color/image_raw \
-p topic_type:=image
```

Point cloud topic example:

```bash
ros2 run orbbec_camera frame_latency_node \
--ros-args \
-p topic_name:=/camera/depth/points \
-p topic_type:=points
```

## monitor_fd.sh

`monitor_fd.sh` prints the file descriptor count of the `component_container` process once per second. It is useful for checking file descriptor leaks. The target process must be running before using it.

```bash
cd orbbec_camera/scripts
./monitor_fd.sh
```

## plot_stat.py

`plot_stat.py` reads `statistics.csv` from the current directory and plots age and period curves from topic statistics. It is usually used together with `topic_statistics_node`.

```bash
cd <directory-containing-statistics.csv>
python3 /path/to/orbbec_camera/scripts/plot_stat.py
```

## receive_pc.py

`receive_pc.py` subscribes to `/camera/depth/points` and quickly verifies whether a Python node can receive the point cloud topic. Start the camera node and enable point cloud before using it.

```bash
python3 orbbec_camera/scripts/receive_pc.py
```
