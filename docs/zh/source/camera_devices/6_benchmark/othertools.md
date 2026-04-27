# 其他工具

## firmware_update_tool 工具

`firmware_update_tool` 用于从 ROS 2 命令行升级设备固件或烧录 preset 文件。升级前请确认设备连接稳定；多设备连接时建议指定序列号，避免升级到错误设备。

查看帮助：

```bash
ros2 run orbbec_camera firmware_update_tool -- --help
```

升级单个设备固件：

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number <SN> \
--firmware_path /path/to/firmware.bin
```

烧录 preset 文件：

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number <SN> \
--preset_path /path/to/preset.bin
```

批量升级多个设备时，`--serial_number` 支持逗号分隔；如希望某个设备失败后继续处理后续设备，可增加 `--continue_on_error`。

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number SN1,SN2 \
--firmware_path /path/to/firmware.bin \
--continue_on_error
```

## 帧时间戳 CSV 记录

开启 `enable_frame_timestamp_csv` 后，相机节点会记录彩色和深度帧的时间戳数据到 CSV 文件，用于分析帧同步、发布延迟和时间戳异常。

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
enable_frame_timestamp_csv:=true \
frame_timestamp_csv_file:=/tmp/frame_timestamp.csv
```

CSV 中包含 SDK frame index、hardware frame number、sensor timestamp、device/global/system timestamp、arrival timestamp、publish timestamp、相邻帧 delta 以及 SDK delay 等字段。

## Ob_benchmark 工具

> 此工具的目标是对各种 OrbbecSDK_ROS2 相机配置的性能进行基准测试。基准测试结果取决于使用的相机和设置。（目前仅适用于 ROS2 Humble）

您可以在 [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples) 中找到示例使用代码。

### 工具配置 ([start_benchmark_params.json](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/tools/startbenchmark/start_benchmark_params.json))

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

- `camera_name`：要配置的相机名称。例如：`"camera_01"`、`"camera_02"` 等。
- `process_name`：要监控的进程名称。例如，`"component_conta"` 将监控容器进程的数据。
- `switch_cycle`：切换配置的周期时间，以秒为单位。例如，设置为 `300` 意味着配置将每 300 秒切换一次。
- `test_cycle`：测试周期，以秒为单位。例如，设置为 `1` 意味着工具将每 1 秒收集一次监控进程的数据。
- `skip_number`：要跳过的数据点数量。例如，设置为 `30` 意味着将忽略前 30 个数据点。

### 相机配置（启动文件）

在 launch 文件夹中，有多个 .launch.py 文件（`ob_benchmark_0.launch.py`、`ob_benchmark_1.launch.py`、...、`ob_benchmark_19.launch.py`）。每个文件对应不同的相机配置。

### 运行 ob_benchmark 工具

要运行该工具，请使用以下命令：

```bash
source install/setup.bash
ros2 run orbbec_camera ob_benchmark_node
```

### 输出数据文件

输出数据文件将存储在 ob_benchmark 文件夹中，文件名如 `0.csv`、`1.csv`、...、19.csv。例如：

- `0.csv` 包含来自 `ob_benchmark_0.launch.py` 配置的数据。
- `1.csv` 包含来自 `ob_benchmark_1.launch.py` 配置的数据。
