# 工具索引

本页作为 OrbbecSDK_ROS2 工具索引，只列出每个工具的用途和完整说明入口。运行 `ros2 run` 命令前，请先加载 ROS 2 和工作空间环境：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

## 工具总览

| 工具 | 使用场景 / 完整说明 | 简介 |
| --- | --- | --- |
| `list_devices_node`（推荐） | <a href="device_query_tools.html">设备查询与基础维护</a> | 枚举 Orbbec 设备并输出设备、固件、preset 和 IP 状态。 |
| `list_depth_work_mode_node` | <a href="device_query_tools.html">设备查询与基础维护</a> | 查询当前设备支持的 depth work mode。 |
| `list_camera_profile_mode_node`（推荐） | <a href="device_query_tools.html">设备查询与基础维护</a> | 查询设备支持的 stream profile、depth work mode 和 preset。 |
| `list_ob_devices.sh` | <a href="device_query_tools.html">设备查询与基础维护</a> | 从 Linux USB sysfs 扫描 Orbbec 设备。 |
| `install_udev_rules.sh` | <a href="device_query_tools.html">设备查询与基础维护</a> | 安装 USB 设备 udev 规则。 |
| `firmware_update_tool`（推荐） | <a href="firmware_update_tool.html">设备维护</a> | 升级固件或烧录 preset 文件。 |
| `ip_config_tool`（推荐） | <a href="network_config_tools.html">网络配置</a> | 配置网络相机 DHCP、persistent IP、Force IP。 |
| `common_benchmark_node.py`（推荐） | <a href="benchmark_tools.html">性能基准测试</a> | 统计帧率、延迟、CPU、内存和丢帧。 |
| `service_benchmark_node.py` | <a href="benchmark_tools.html">性能基准测试</a> | 统计 service 调用延迟和成功率。 |
| `ob_benchmark_node` | <a href="benchmark_tools.html">性能基准测试</a> | 周期性运行 benchmark 配置并记录 CPU/内存。 |
| `start_benchmark_node` | <a href="benchmark_tools.html">性能基准测试</a> | benchmark 流程中的多 topic 订阅端。 |
| `enable_frame_drop_log` / `frame_timestamp_csv_file`（推荐） | <a href="diagnostic_tools.html">性能诊断</a> | 相机节点内置的丢帧日志和时间戳 CSV 记录功能。 |
| `topic_statistics_node` | <a href="diagnostic_tools.html">性能诊断</a> | 统计图像 topic 的 age 和 period。 |
| `frame_latency_node` | <a href="diagnostic_tools.html">性能诊断</a> | 统计指定 topic 的延迟和 FPS。 |
| `monitor_fd.sh` | <a href="diagnostic_tools.html">性能诊断</a> | 监控 `component_container` 文件描述符数量。 |
| `plot_stat.py` | <a href="diagnostic_tools.html">性能诊断</a> | 绘制 `statistics.csv` 中的 topic statistics 曲线。 |
| `receive_pc.py` | <a href="diagnostic_tools.html">调试辅助</a> | 验证 point cloud topic 是否可被 Python 节点接收。 |
| `multi_save_rgbir_node`（推荐） | <a href="multi_camera_tools.html">多相机</a> | 多相机 RGB/IR 图像保存工具。 |
| `image_sync_example_node`（推荐） | <a href="multi_camera_tools.html">多相机</a> | 在线显示并统计多路图像时间戳同步情况。 |
| `SyncFramesMain.py` | <a href="multi_camera_tools.html">多相机</a> | 多相机同步验证离线分析脚本。 |
| `group_image.py` | <a href="multi_camera_tools.html">多相机</a> | 按时间戳对多相机保存图片分组。 |
| `static_transforms_publisher.py` | <a href="multi_camera_tools.html">多相机调试</a> | 发布脚本中写死的多相机静态 TF。 |
