# 性能基准测试工具

本节介绍性能基准测试工具，解释其目的、功能以及它可以帮助您测量的内容。

## common_benchmark_node.py

`common_benchmark_node.py` 是一个用于监控在 ROS 环境中运行的 Orbbec 相机性能的工具。它实时收集和记录关键相机指标，如帧率、延迟、系统资源使用和丢包率，帮助用户评估相机节点的稳定性和性能（每秒更新一次）。

功能：

- 测量发布的图像帧率和延迟（当前、最小、最大、平均）。
- 监控相机节点的 CPU/ARM 使用率（当前、最小、最大、平均）。
- 跟踪丢帧率（发布者）和丢包率（订阅者）。
- 将实时统计信息（1 Hz）打印到终端并将结果保存到 CSV 文件。
- 支持可配置的运行时长和 CSV 输出路径。

在 ROS1 中，可以测量丢帧率和丢包率，而在 ROS2 中，header 缺少 `seq` 字段，因此仅计算发布者端的丢帧率。

![common_benchmark_ros1](../image/benchmark_images/common_benchmark_ros1.png "ROS1")

![common_benchmark_ros2](../image/benchmark_images/common_benchmark_ros2.png "ROS2")

运行示例：

```bash
ros2 run orbbec_camera common_benchmark_node.py \
    --run_time 2h  \
    --csv_file /path/to/log.csv
```

参数：

- `--run_time`：监控持续时间，指定为时间字符串，如 `"10s"`、`"5m"`、`"1h"`、`"2d"`。默认为 10 秒。
- `--csv_file`：输出 CSV 文件的路径。默认情况下，它保存在工作空间目录中，名称为 `camera_monitor_log.csv`。

多相机监控示例：

```bash
ros2 run orbbec_camera common_benchmark_node.py \
--run_time 1h \
--csv_file /tmp/cam_log.csv \
--camera_names camera_01,camera_02
```

## service_benchmark_node.py

`service_benchmark_node` 工具用于监控服务调用的性能。它可以测量服务调用的成功率和执行服务所需的时间。

功能：

- 对单个服务调用进行基准测试，测量延迟和成功率。
- 对 YAML 配置文件中定义的多个服务进行基准测试。
- 可选择将基准测试结果保存到 CSV 文件。

![service benchmark](../image/benchmark_images/service_benchmark.png)

当您需要收集多个服务的数据时，建议使用 CSV 文件进行分析。

### ROS2 C++

单个服务基准测试：

```bash
ros2 run orbbec_camera service_benchmark_node \
    --ros-args \
    -p service_name:=/camera/get_depth_gain \
    -p service_type:=orbbec_camera_msgs/srv/GetInt32 \
    -p count:=10
```

多个服务基准测试（YAML 配置）：

```bash
ros2 run orbbec_camera service_benchmark_node \
    --ros-args \
    -p yaml_file:=/path/to/default_service.yaml
```

### ROS2 Python

单个服务基准测试：

```bash
ros2 run orbbec_camera service_benchmark_node.py --service /camera/get_depth_gain --count 10
```

多个服务基准测试（YAML 配置）：

```bash
ros2 run orbbec_camera service_benchmark_node.py --yaml_file /path/to/default_service.yaml
```

示例 YAML 配置位于 `orbbec_camera/scripts/default_service.yaml`：

```yaml
default_count: 10

services:
- name: /camera/get_auto_white_balance
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_color_exposure
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_color_gain
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_depth_exposure
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_depth_gain
  type: orbbec_camera_msgs/srv/GetInt32
- name: /camera/get_device_info
  type: orbbec_camera_msgs/srv/GetDeviceInfo
- name: /camera/send_software_trigger
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_auto_white_balance
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_color_ae_roi
  type: orbbec_camera_msgs/srv/SetArrays
  request: {data_param: [0,1279,0,719]}
- name: /camera/set_color_auto_exposure
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_color_exposure
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 30}
- name: /camera/set_color_flip
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_color_gain
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 20}
- name: /camera/set_color_mirror
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_color_rotation
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 90}
- name: /camera/set_depth_ae_roi
  type: orbbec_camera_msgs/srv/SetArrays
  request: {data_param: [0,1279,0,719]}
- name: /camera/set_depth_auto_exposure
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_depth_exposure
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 3000}
- name: /camera/set_depth_flip
  type: std_srvs/srv/SetBool
  request: {data: false}
- name: /camera/set_depth_gain
  type: orbbec_camera_msgs/srv/SetInt32
  request: {data: 200}
```

## ob_benchmark_node

> 此工具的目标是对各种 OrbbecSDK_ROS2 相机配置的性能进行基准测试。基准测试结果取决于使用的相机和设置。（目前仅适用于 ROS2 Humble）

您可以在 [example](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples) 中找到示例使用代码。

### 工具配置

配置文件为 `orbbec_camera/config/tools/startbenchmark/start_benchmark_params.json`。

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

## start_benchmark_node

`start_benchmark_node` 是 benchmark 流程中的订阅端，会按 `start_benchmark_params.json` 中的 `camera_name` 订阅多相机 color、depth、IR 和 point cloud topic。它通常与 benchmark launch 配合使用。

```bash
ros2 run orbbec_camera start_benchmark_node
```
