# 诊断与延迟分析工具

本节介绍帧连续性、时间戳、topic statistics、端到端延迟和调试辅助工具。

## 帧丢失日志与时间戳 CSV 记录

开启 `enable_frame_drop_log` 后，相机节点会在日志中输出彩色和深度帧丢失统计，用于定位 SDK 接收阶段和 ROS 发布阶段的丢帧。设置 `frame_timestamp_csv_file` 后，相机节点会额外记录彩色和深度帧的时间戳数据到 CSV 文件，用于分析帧连续性、发布延迟和时间戳异常。

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
enable_frame_drop_log:=true \
frame_timestamp_csv_file:=/tmp/frame_timestamp.csv
```

CSV 中包含 SDK frame index、hardware frame number、sensor timestamp、device/global/system timestamp、steady arrival/publish delta、ROS 发布耗时以及 SDK delay 等字段。

### CSV 分片

每个 CSV 文件最多保存 `1,024,575` 行帧数据和 1 行表头。达到上限后，日志器会自动写入下一个带序号的文件，例如 `frame_timestamp_1.csv`、`frame_timestamp_2.csv`。

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
| `_arrival_steady_delta_us` | 到达主机 steady 时间相邻帧差值 | us |
| `_publish_steady_delta_us` | 发布前主机 steady 时间相邻帧差值 | us |
| `_arrival_to_publish_steady_us` | ROS 收到帧到发布的耗时（steady） | `publish_steady - arrival_steady` |
| `_sdk_delay_from_global_us` | SDK 发布延迟（global 参照） | `arrival_system - global_ts` |
| `_sdk_delay_from_system_us` | SDK 发布延迟（system 参照） | `arrival_system - sdk_system_ts` |

### 分析方法

#### 硬件丢帧判断

- 查看 `_hardware_frame_number` 是否连续。
- 查看 `_sensor_ts_delta_us` 的折线图或散点图，观察是否存在明显跳变。
- 例如在 30 fps 下，相邻帧时间差通常应接近 33333 us。

#### SDK/ROS 丢帧判断

- 查看 `_sdk_frame_index` 是否连续。
- 查看 `_device_ts_delta_us`、`_global_ts_delta_us` 和 `_system_ts_delta_us` 的折线图或散点图，观察是否存在跳变。
- 开启 `enable_frame_drop_log` 后，日志中的 `stage=SDK_RECEIVE` 表示 SDK 接收阶段检测到丢帧，`stage=ROS_PUBLISH` 表示 ROS 发布阶段检测到丢帧。

#### 延迟判断

- SDK 延迟：查看 `_sdk_delay_from_global_us` 和 `_sdk_delay_from_system_us` 的折线图或散点图，用于观察帧从底层时间戳到到达 ROS 节点之间的延迟变化。
- ROS 延迟：查看 `_arrival_to_publish_steady_us` 的折线图或散点图，用于统计 ROS 侧从 SDK 回调拿到帧到发布图像的耗时。
- 如果需要更接近真实处理耗时，优先参考 steady 时钟相关字段。

#### 同步说明

当前这份 CSV 主要用于分析单路彩色或深度流的连续性和延迟，不能直接用于统计彩色与深度之间的同步效果。

## topic_statistics_node

`topic_statistics_node` 使用 ROS 2 topic statistics 统计图像 topic 的 age 和 period，并在当前目录生成 `statistics.csv`。使用前需要先启动相机节点。

```bash
ros2 run orbbec_camera topic_statistics_node \
--ros-args \
-p image_topic:=/camera/color/image_raw \
-p statistics_topic:=/statistics
```

## frame_latency_node

`frame_latency_node` 订阅指定 topic，按消息 header stamp 计算端到端延迟并输出 FPS。它支持 `image`、`points`、`imu`、`metadata`、`camera_info`、`rgbd`、`imu_info`、`tf` 等 topic 类型。使用前需要先启动相机节点。

```bash
ros2 run orbbec_camera frame_latency_node \
--ros-args \
-p topic_name:=/camera/color/image_raw \
-p topic_type:=image
```

统计点云 topic：

```bash
ros2 run orbbec_camera frame_latency_node \
--ros-args \
-p topic_name:=/camera/depth/points \
-p topic_type:=points
```

## monitor_fd.sh

`monitor_fd.sh` 每秒统计 `component_container` 进程的文件描述符数量，用于排查文件描述符泄漏。使用前需要目标进程正在运行。

```bash
cd orbbec_camera/scripts
./monitor_fd.sh
```

## plot_stat.py

`plot_stat.py` 读取当前目录下的 `statistics.csv`，绘制 topic statistics 中 age 和 period 的曲线。通常与 `topic_statistics_node` 配合使用。

```bash
cd <statistics.csv所在目录>
python3 /path/to/orbbec_camera/scripts/plot_stat.py
```

## receive_pc.py

`receive_pc.py` 订阅 `/camera/depth/points`，用于快速验证 point cloud topic 是否可被 Python 节点接收。使用前需要先启动相机节点并开启点云。

```bash
python3 orbbec_camera/scripts/receive_pc.py
```
