# Other Tools

## Frame Drop Logging and Frame Timestamp CSV Logging

When `enable_frame_drop_log` is enabled, the camera node prints Color and Depth frame drop statistics to the log. The log helps distinguish drops detected at the SDK receive stage from drops detected at the ROS publish stage. When `frame_timestamp_csv_file` is set, the camera node also records Color and Depth frame timestamp data to a CSV file for frame continuity, publish latency, and timestamp debugging.

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
enable_frame_drop_log:=true \
frame_timestamp_csv_file:=/tmp/frame_timestamp.csv
```

The CSV includes SDK frame index, hardware frame number, sensor timestamp, device/global/system timestamp, steady arrival/publish delta values, ROS publish latency, and SDK delay fields.

### Field Description

The current CSV contains two sets of homogeneous fields with the prefixes `color_` and `depth_`, for example `color_sdk_frame_index` and `depth_sdk_frame_index`. The definitions are identical for both sets; only the data source differs.

| Field suffix | Description | Unit / Notes |
| --- | --- | --- |
| `_sdk_frame_index` | SDK frame index | `frame->index()` |
| `_hardware_frame_number` | Hardware frame number | `frame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER)` |
| `_sensor_ts_sec` | Sensor timestamp | Seconds, usually the midpoint of the exposure time |
| `_sensor_ts_delta_us` | Delta between adjacent sensor timestamps | us |
| `_device_ts_sec` | Device clock timestamp | Seconds |
| `_device_ts_delta_us` | Delta between adjacent device timestamps | us |
| `_global_ts_sec` | Global timestamp | Seconds |
| `_global_ts_delta_us` | Delta between adjacent global timestamps | us |
| `_system_ts_sec` | SDK system timestamp | Seconds |
| `_system_ts_delta_us` | Delta between adjacent SDK system timestamps | us |
| `_arrival_steady_delta_us` | Delta between adjacent arrival steady timestamps | us |
| `_publish_steady_delta_us` | Delta between adjacent publish steady timestamps | us |
| `_arrival_to_publish_steady_us` | Time from frame arrival to publish on the ROS side (steady) | `publish_steady - arrival_steady` |
| `_sdk_delay_from_global_us` | SDK publish delay referenced to global time | `arrival_system - global_ts` |
| `_sdk_delay_from_system_us` | SDK publish delay referenced to system time | `arrival_system - sdk_system_ts` |

### Analysis Method

#### Hardware Frame Drop Detection

- Check whether `_hardware_frame_number` is continuous.
- Plot `_sensor_ts_delta_us` as a line chart or scatter plot and look for obvious jumps.
- For example, at 30 fps, the interval between adjacent frames should usually be close to 33333 us.

#### SDK / ROS Frame Drop Detection

- Check whether `_sdk_frame_index` is continuous.
- Plot `_device_ts_delta_us`, `_global_ts_delta_us`, and `_system_ts_delta_us` to see whether any of them show abnormal jumps.
- When `enable_frame_drop_log` is enabled, `stage=SDK_RECEIVE` means drops were detected at the SDK receive stage, and `stage=ROS_PUBLISH` means drops were detected at the ROS publish stage.

#### Latency Analysis

- SDK latency: inspect `_sdk_delay_from_global_us` and `_sdk_delay_from_system_us` with line charts or scatter plots to observe the delay from the underlying timestamp to frame arrival at the ROS node.
- ROS latency: inspect `_arrival_to_publish_steady_us` to measure the time from receiving a frame in the SDK callback to publishing the image on the ROS side.
- If you want a metric closer to actual processing time, prefer fields based on the steady clock.

#### Synchronization Note

This CSV is mainly intended for analyzing continuity and latency of a single Color or Depth stream. It cannot be used directly to evaluate synchronization between Color and Depth.

## Ob_benchmark tool

> The goal of this tool is to benchmark the performance of various OrbbecSDK_ROS2 camera configurations. The benchmark results depend on the camera and settings used.(Currently only works with ROS2 Humble)

You can find example usage code in the [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples).

### Tool Configuration ([start_benchmark_params.json](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/tools/startbenchmark/start_benchmark_params.json))

```json
{
    "start_benchmark_params": {
        "camera_name": [
            "camera_01",
            "camera_02",
            "camera_03",
            "camera_04"
        ],
        "process_name": "component_conta",
        "switch_cycle": 300,
        "test_cycle": 1,
        "skip_number": 30
    }
}
```

- `camera_name`: Names of the cameras to be configured. Example: `"camera_01"`, `"camera_02"`, etc.
- `process_name`: The name of the process to be monitored. For example, `"component_conta"` will monitor the data of the container process.
- `switch_cycle`: The cycle time for switching configurations, in seconds. For example, setting it to `300` means the configuration will switch every 300 seconds.
- `test_cycle`: The testing cycle, in seconds. For example, setting it to `1` means the tool will collect data for the monitored process every 1 second.
- `skip_number`: The number of data points to skip. For example, setting it to `30` means that the first 30 data points will be ignored.

### Camera configuration (launch files)

In the launch folder, there are multiple.launch.py files (`ob_benchmark_0.launch.py`, `ob_benchmark_1.launch.py`, ..., `ob_benchmark_19.launch.py`). Each file corresponds to a different camera configuration.

### Running the ob_benchmark tool

To run the tool, use the following commands:

```bash
source install/setup.bash
ros2 run orbbec_camera ob_benchmark_node
```

### Output Data Files

The output data files will be stored in the ob_benchmark folder with filenames like `0.csv`, `1.csv`, ..., 19.csv. For example:

- `0.csv` contains data from the `ob_benchmark_0.launch.py` configuration.
- `1.csv` contains data from the `ob_benchmark_1.launch.py` configuration.
