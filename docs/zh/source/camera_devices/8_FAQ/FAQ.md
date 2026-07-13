# 常见问题

### 意外崩溃

如果相机节点意外崩溃，它会在 `~/.ros/Log/<camera_name>/` 目录下生成崩溃日志，文件名类似 `camera_name_crash_stack_trace_xx.log`。请将此日志发送给支持团队或提交到 GitHub issue 以获得进一步帮助。

### 多相机无数据流

**电源供应不足**：

- 确保每个相机连接到单独的集线器。
- 使用有源集线器为每个相机提供足够的电力。

**高分辨率**：

- 尝试降低分辨率以解决数据流问题。

**增加usbfs_memory_mb值**：

- 通过运行以下命令将 `usbfs_memory_mb` 值增加到128MB（这是参考值，可根据系统需求调整）：

```
    echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

- 要使此更改永久生效，请查看[此链接](https://github.com/OpenKinect/libfreenect2/issues/807)。

### 如何采集和保存日志

请按以下步骤采集日志：

1. 将 launch 参数 `log_level` 设为 `debug`。
2. 启动 launch，记下启动时间，然后重新复现问题。
3. 将以下两类日志一起提供给技术支持：

   - **SDK 日志**：`~/.ros/Log/<camera_name>/` 中与启动 launch 时间对应的日志文件。
   - **ROS2 日志**：`~/.ros/log/` 中与启动 launch 时间对应的日志文件。

SDK 日志通常以启动时间命名。多相机或无法确定具体文件时，可提供本次测试新生成的全部日志。如果使用多相机 launch 中的固定日志名，建议复现问题前先删除或备份旧的 SDK 日志。

#### 开发者排查说明

**SDK 日志**

- SDK 日志保存在 `~/.ros/Log/<camera_name>/`。
- `log_file_name` 为空时，日志默认以节点启动时间命名，格式为 `OrbbecSDK_<YYYYMMDD_HHMMSS>.log`，例如 `~/.ros/Log/camera/OrbbecSDK_20260713_143025.log`。
- 指定 `log_file_name` 后，实际路径为 `~/.ros/Log/<camera_name>/<log_file_name>`。使用固定文件名时，多次启动可能继续写入同一个文件。
- 多相机场景下，SDK 日志按 `camera_name` 分目录保存。当前 `multi_camera.launch.py` 和 `multi_camera_synced.launch.py` 会分别指定 `camera_01.log`、`camera_02.log`。

**ROS2 日志**

- 将节点或可组合节点容器的 `output` 设为 `"log"` 后，ROS2 日志保存在 `~/.ros/log/`。
- 每次 `ros2 launch` 都会生成一个时间戳目录，其中包含 `launch.log`；同时还会生成 `component_container_<pid>_<timestamp>.log` 一类的进程日志文件。
- `launch.log` 汇总所有相机容器的输出；`component_container_<pid>_<timestamp>.log` 记录对应容器进程的运行输出。
- 多相机场景下，进程日志按容器区分。一机一容器时，可以看作一路相机对应一个进程日志；多路相机共用容器时，同一个进程日志会包含多路相机输出。
- 可根据日志中的命名空间或节点名区分相机，例如 `camera_01.camera_01`、`camera_02.camera_02`。

### 为什么有这么多启动文件？

- 不同的相机具有不同的默认分辨率和图像格式。
- 为简化使用，每个相机都有自己的启动文件。

### 多相机连接时如何指定启动某一个相机

如果启动文件未显式指定要使用的设备，在同时连接多台相机时，驱动会默认连接到其中一个（默认设备）。

可以先通过以下命令查看设备序列号：

```bash
ros2 run orbbec_camera list_devices_node
```

然后在启动时显式指定序列号，例如：

```bash
ros2 launch orbbec_camera femto_bolt.launch.py serial_number:=CL8H741005J
```

### 多相机启动或切换流时为什么需要设置延迟？

多相机系统对带宽和设备初始化时序要求较高。如果在同一时间启动或切换多个相机流，可能会引发带宽瞬时拥塞，进而导致设备初始化失败、流启动异常或丢帧等问题。为确保系统稳定性，建议注意以下几点：

- **多相机启动阶段**

  在启动多个相机时，建议在每个相机启动之间增加适当的延迟（例如 **2s**），以避免瞬时带宽过载或底层设备初始化冲突。

- **流开关与模式切换阶段**

  在调用开关流相关服务（如 `set_streams_enable`、`toggle_depth`、`toggle_color`）时，不建议同时触发多个接口调用，应在各操作之间设置合理的时间间隔（例如 **20 ms**），以保证流状态切换的可靠性。

遵循上述时序控制原则，有助于提升多相机系统在启动和运行过程中的稳定性，减少异常和不可预期行为的发生。
### femto bolt 深度流无数据

该模组运行时依赖 OpenGL 库，若系统未安装或驱动不完整，将导致深度流无数据。请先安装 OpenGL 相关库（以 Ubuntu 为例）：

```bash
  sudo apt update && sudo apt install -y mesa-utils libgl1-mesa-glx libglu1-mesa
```

  安装后可通过以下命令检查 OpenGL 是否可用：

```bash
  glxinfo -B
```

### 图像未达到预设帧率

首先需要确认图像是否确实未达到预设帧率。在 ROS 2 中可通过多种方式查看帧率，例如：

* `ros2 topic hz`
* `rqt`
* 自定义工具（如本 ROS 包提供的 `benchmark` 工具）

需要注意的是，不同工具的统计方式和 QoS 配置不同，因此得到的帧率结果可能存在差异。当发现帧率低于预期时，请优先排查是否为帧率统计工具本身导致的误差。

若确认图像帧率确实未达到预设值，可尝试以下排查步骤：

1. **降低分辨率或帧率**，判断是否由于 USB / 网络带宽受限导致帧率下降；
2. **确认相机固件版本及 ROS 包版本是否为最新**，旧版本可能存在性能或兼容性问题。

若以上方法仍无法解决问题，请联系我司 **FAE**，或在 **GitHub Issue** 中提交问题以获得进一步支持。


### 软触发模式相关问题

* **信号触发时各传感器未同时出流**
  请开启帧汇聚功能，将参数`frame_aggregate_mode`设置为`full_frame`，以保证多传感器数据在同一次触发下同步输出。

* **自动触发模式下无法达到预设帧率**
  设置 `software_trigger_period` 时，需要综合考虑实际开流帧率与曝光时间。例如，当 `color_fps` 设置为 10 FPS 时，`software_trigger_period` 不能低于以下计算值：

  ```
  software_trigger_period ≥ 1000000 / fps × N + 2 × expo
  ```

  其中：

  * `fps`：传感器帧率
  * `N`：单次触发采集的帧数量
  * `expo`：曝光时间
  * `单位`：µs

  若 `software_trigger_period` 设置过小，将导致触发频率受限，从而丢帧。
