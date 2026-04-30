# 其他工具

## 帧时间戳 CSV 记录

开启 `enable_frame_timestamp_csv` 后，相机节点会记录彩色和深度帧的时间戳数据到 CSV 文件，用于分析帧同步、发布延迟和时间戳异常。

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
enable_frame_timestamp_csv:=true \
frame_timestamp_csv_file:=/tmp/frame_timestamp.csv
```

CSV 中包含 SDK frame index、hardware frame number、sensor timestamp、device/global/system timestamp、arrival timestamp、publish timestamp、相邻帧 delta 以及 SDK delay 等字段。

### 字段说明

当前 CSV 中包含两组同构字段，分别以 `color_` 和 `depth_` 为前缀，例如 `color_sdk_frame_index` 和 `depth_sdk_frame_index`。两组字段定义完全一致，仅数据来源不同。

| 字段后缀 | 描述 | 单位/说明 |
| --- | --- | --- |
| `_sdk_frame_index` | SDK 帧序号 | `frame->index()` |
| `_hardware_frame_number` | 硬件帧序号 | `frame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER)` |
| `_sensor_ts_sec` | 传感器时间戳 | 秒，通常为曝光时间中点 |
| `_sensor_ts_delta_us` | 传感器时间戳相邻帧差值 | us |
| `_device_ts_sec` | 设备时钟时间戳 | 秒 |
| `_device_ts_delta_us` | 设备时钟相邻帧差值 | us |
| `_global_ts_sec` | global 时间戳 | 秒 |
| `_global_ts_delta_us` | global 时间戳相邻帧差值 | us |
| `_system_ts_sec` | SDK 的 system 时间戳 | 秒 |
| `_system_ts_delta_us` | SDK system 时间戳相邻帧差值 | us |
| `_arrival_system_sec` | 帧到达节点时采样的系统时间 | 秒 |
| `_arrival_system_delta_us` | 到达系统时间相邻帧差值 | us |
| `_arrival_steady_sec` | 帧到达节点时采样的主机 steady 时间 | 秒 |
| `_arrival_steady_delta_us` | 到达主机 steady 时间相邻帧差值 | us |
| `_publish_system_sec` | 发布图像前采样的系统时间 | 秒 |
| `_publish_system_delta_us` | 发布前系统时间相邻帧差值 | us |
| `_publish_steady_sec` | 发布图像前采样的主机 steady 时间 | 秒 |
| `_publish_steady_delta_us` | 发布前主机 steady 时间相邻帧差值 | us |
| `_arrival_to_publish_system_us` | ROS 收到帧到发布的耗时（system） | `publish_system - arrival_system` |
| `_arrival_to_publish_steady_us` | ROS 收到帧到发布的耗时（steady） | `publish_steady - arrival_steady` |
| `_sdk_delay_from_global_us` | SDK 发布延迟（global 参照） | `arrival_system - global_ts` |
| `_sdk_delay_from_system_us` | SDK 发布延迟（system 参照） | `arrival_system - sdk_system_ts` |

### 分析方法

#### 硬件丢帧判断

- 查看 `_hardware_frame_number` 是否连续。
- 查看 `_sensor_ts_delta_us` 的折线图或散点图，观察是否存在明显跳变。
- 例如在 30 fps 下，相邻帧时间差通常应接近 33333 us。

#### SDK 丢帧判断

- 查看 `_sdk_frame_index` 是否连续。
- 查看 `_device_ts_delta_us`、`_global_ts_delta_us` 和 `_system_ts_delta_us` 的折线图或散点图，观察是否存在跳变。
- 如果 SDK 帧序号或多种时钟源的相邻帧差值出现异常，可进一步定位 SDK 层是否有丢帧。

#### 延迟判断

- SDK 延迟：查看 `_sdk_delay_from_global_us` 和 `_sdk_delay_from_system_us` 的折线图或散点图，用于观察帧从底层时间戳到到达 ROS 节点之间的延迟变化。
- ROS 延迟：查看 `_arrival_to_publish_steady_us` 的折线图或散点图，用于统计 ROS 侧从 SDK 回调拿到帧到发布图像的耗时。
- 如果需要更接近真实处理耗时，优先参考 steady 时钟相关字段。

#### 同步说明

当前这份 CSV 主要用于分析单路彩色或深度流的连续性和延迟，不能直接用于统计彩色与深度之间的同步效果。

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
