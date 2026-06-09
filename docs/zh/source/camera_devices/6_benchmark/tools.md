# 工具索引

本页作为 OrbbecSDK_ROS2 工具索引，只列出每个工具的用途和完整说明入口。运行 `ros2 run` 命令前，请先加载 ROS 2 和工作空间环境：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
```

## 工具总览

| 工具 | 使用场景 | 简介 | 完整说明 |
| --- | --- | --- | --- |
| `list_devices_node`（推荐） | 设备查询 | 枚举 Orbbec 设备并输出设备、固件、preset 和 IP 状态。 | [设备查询与基础维护工具](device_query_tools.md) |
| `list_depth_work_mode_node` | 设备查询 | 查询当前设备支持的 depth work mode。 | [设备查询与基础维护工具](device_query_tools.md) |
| `list_camera_profile_mode_node`（推荐） | 设备查询 | 查询设备支持的 stream profile、depth work mode 和 preset。 | [设备查询与基础维护工具](device_query_tools.md) |
| `list_ob_devices.sh` | 设备查询 | 从 Linux USB sysfs 扫描 Orbbec 设备。 | [设备查询与基础维护工具](device_query_tools.md) |
| `install_udev_rules.sh` | 设备维护 | 安装 USB 设备 udev 规则。 | [设备查询与基础维护工具](device_query_tools.md) |
| `firmware_update_tool`（推荐） | 设备维护 | 升级固件或烧录 preset 文件。 | [固件升级工具](firmware_update_tool.md) |
| `ip_config_tool`（推荐） | 网络配置 | 配置网络相机 DHCP、persistent IP、Force IP。 | [网络配置工具](network_config_tools.md) |
| `common_benchmark_node.py`（推荐） | 性能基准测试 | 统计帧率、延迟、CPU、内存和丢帧。 | [性能基准测试工具](benchmark_tools.md) |
| `service_benchmark_node.py` | 性能基准测试 | 统计 service 调用延迟和成功率。 | [性能基准测试工具](benchmark_tools.md) |
| `ob_benchmark_node` | 性能基准测试 | 周期性运行 benchmark 配置并记录 CPU/内存。 | [性能基准测试工具](benchmark_tools.md) |
| `start_benchmark_node` | 性能基准测试 | benchmark 流程中的多 topic 订阅端。 | [性能基准测试工具](benchmark_tools.md) |
| `enable_frame_drop_log` / `frame_timestamp_csv_file`（推荐） | 性能诊断 | 相机节点内置的丢帧日志和时间戳 CSV 记录功能。 | [诊断与延迟分析工具](diagnostic_tools.md) |
| `topic_statistics_node` | 性能诊断 | 统计图像 topic 的 age 和 period。 | [诊断与延迟分析工具](diagnostic_tools.md) |
| `frame_latency_node` | 性能诊断 | 统计指定 topic 的延迟和 FPS。 | [诊断与延迟分析工具](diagnostic_tools.md) |
| `monitor_fd.sh` | 性能诊断 | 监控 `component_container` 文件描述符数量。 | [诊断与延迟分析工具](diagnostic_tools.md) |
| `plot_stat.py` | 性能诊断 | 绘制 `statistics.csv` 中的 topic statistics 曲线。 | [诊断与延迟分析工具](diagnostic_tools.md) |
| `receive_pc.py` | 调试辅助 | 验证 point cloud topic 是否可被 Python 节点接收。 | [诊断与延迟分析工具](diagnostic_tools.md) |
| `multi_save_rgbir_node`（推荐） | 多相机 | 多相机 RGB/IR 图像保存工具。 | [多相机辅助工具](multi_camera_tools.md) |
| `image_sync_example_node`（推荐） | 多相机 | 在线显示并统计多路图像时间戳同步情况。 | [多相机辅助工具](multi_camera_tools.md) |
| `SyncFramesMain.py` | 多相机 | 多相机同步验证离线分析脚本。 | [多相机辅助工具](multi_camera_tools.md) |
| `group_image.py` | 多相机 | 按时间戳对多相机保存图片分组。 | [多相机辅助工具](multi_camera_tools.md) |
| `static_transforms_publisher.py` | 多相机调试 | 发布脚本中写死的多相机静态 TF。 | [多相机辅助工具](multi_camera_tools.md) |
