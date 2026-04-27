# Other Tools

## firmware_update_tool

`firmware_update_tool` upgrades device firmware or writes preset files from the ROS 2 command line. Before upgrading, make sure the device connection is stable. When multiple devices are connected, specify the serial number to avoid updating the wrong device.

Show help:

```bash
ros2 run orbbec_camera firmware_update_tool -- --help
```

Upgrade firmware for one device:

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number <SN> \
--firmware_path /path/to/firmware.bin
```

Write a preset file:

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number <SN> \
--preset_path /path/to/preset.bin
```

For batch updates, `--serial_number` accepts comma-separated values. Add `--continue_on_error` if later devices should still be processed after one device fails.

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number SN1,SN2 \
--firmware_path /path/to/firmware.bin \
--continue_on_error
```

## Frame Timestamp CSV Logging

When `enable_frame_timestamp_csv` is enabled, the camera node records Color and Depth frame timestamp data to a CSV file. This is useful for frame synchronization, publish latency, and timestamp debugging.

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
enable_frame_timestamp_csv:=true \
frame_timestamp_csv_file:=/tmp/frame_timestamp.csv
```

The CSV includes SDK frame index, hardware frame number, sensor timestamp, device/global/system timestamp, arrival timestamp, publish timestamp, inter-frame delta values, and SDK delay fields.

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
