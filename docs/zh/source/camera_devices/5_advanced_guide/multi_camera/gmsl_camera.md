# GMSL 多相机同步

> 本节介绍如何在 OrbbecSDK_ROS2 中同步多台通过 GMSL 连接的 Gemini 330 系列相机。

示例代码位于 [gmsl_multi_camera_sync](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/gmsl_multi_camera_sync)。

## 配置设备文件权限

启动相机前，先为相机同步设备授予访问权限：

```bash
sudo chmod 777 /dev/camsync
```

## 配置相机

要获取 GMSL 相机的 `usb_port`，插入相机并在终端中运行以下命令：

```bash
ros2 run orbbec_camera list_devices_node
```

例如，系统报告的 GMSL 相机 `usb_port` 可能为 `gmsl2-1`。

根据系统报告的 GMSL 链路，修改 [multi_gmsl_camera_synced.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/gmsl_multi_camera_sync/multi_gmsl_camera_synced.launch.py) 中的 `usb_port`。该示例为两台相机复用标准的 `gemini_330_series.launch.py`。

## 同步配置

GMSL 多相机同步不需要 Multi-Camera Sync Hub Pro。两台相机均使用 `secondary_synced` 模式。接收触发信号的相机先启动，启用主机端 GMSL 触发的相机在两秒后启动。

**额外的参数设置**

* `gmsl_trigger_fps`：设置硬件 SoC 触发源帧率。
* `enable_gmsl_trigger`：启用硬件 SoC 触发，同一组相机中仅在一台相机上启用。

**运行启动文件**

```bash
ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py
```

> 注意：两台相机均加载 [camera_secondary_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_secondary_params.yaml)，默认启用 color 和 depth。如需配置其他数据流，请修改该文件。

## GMSL 相机的使用限制

GMSL 相机与各种反序列化器芯片（如 MAX9296 和 MAX92716）接口。Orbbec GMSL 相机支持多个流，包括深度、彩色、IR 和 IMU 数据，但存在某些使用限制：

- GMSL 仅支持 V4L2 和 YUYV 格式；不支持 MJPG 格式。RGB 输出源自 YUYV 格式转换。
- Gemini-335Lg 的元数据通过单独的节点提供，而其他型号的元数据嵌入在视频帧中，这对用户保持透明。
- 当使用 Max96712 作为反序列化器芯片时，由于 Max96712 芯片的特性，在 secondary_synced 模式下必须提供多机同步触发信号。否则，在切换数据流时会发生数据流中断。
- 连接在同一个 MAX9296、MAX96712 LinkA/B 或 MAX96712 LinkC/D 上的两个相机有以下限制：
  - 在驱动程序版本 v1.2.02 之前，存在一个限制，即一个相机的 RGB 和另一个相机的右 IR 不能同时流式传输。在驱动程序版本 v1.2.02 之后，限制修改为一个相机的 RGB 和另一个相机的左 IR 不能同时流式传输。
  - 在驱动程序版本 v1.2.02 之前，存在一个限制，即一个相机的 DEPTH 和另一个相机的左 IR 不能同时流式传输。在驱动程序版本 v1.2.02 之后，限制修改为一个相机的 DEPTH 和另一个相机的右 IR 不能同时流式传输。
  - 两个相机的活动流的组合最大数量限制为四个（满足上述两个条件即可确保合规）。

有关更多已知限制，请参考 [Orbbec GMSL 相机的使用限制](https://github.com/orbbec/MIPI_Camera_Platform_Driver/blob/main/doc/Instructions%20for%20Using%20GMSL%20Camera.md)
