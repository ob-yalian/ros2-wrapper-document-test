# 多相机辅助工具

本节介绍多相机图像保存、同步验证和静态 TF 调试辅助工具。

## multi_save_rgbir_node

`multi_save_rgbir_node` 根据 `multi_save_rgbir_params.json` 配置订阅多相机 RGB/IR 图像和 metadata，并通过 `start_capture` 服务触发保存。使用前需要先启动对应的多相机节点。

```bash
ros2 run orbbec_camera multi_save_rgbir_node
```

配置文件位于：

```text
orbbec_camera/config/tools/multisavergbir/multi_save_rgbir_params.json
```

触发保存 10 帧：

```bash
ros2 service call /start_capture orbbec_camera_msgs/srv/SetInt32 "{data: 10}"
```

## image_sync_example_node

`image_sync_example_node` 用于在线验证多路图像时间戳同步情况。它会订阅 1 到 8 路图像 topic，显示同步图像，并输出时间戳差和 FPS 统计。使用前需要先启动相机节点。

未设置 `sync_topics` 时，节点会自动发现 color/depth 图像 topic：

```bash
ros2 run orbbec_camera image_sync_example_node
```

也可以手动指定 topic：

```bash
ros2 run orbbec_camera image_sync_example_node \
--ros-args \
-p sync_topics:="['/camera_01/color/image_raw', '/camera_02/color/image_raw']"
```

多相机同步验证的详细说明见 [多相机同步验证工具](../5_advanced_guide/multi_camera/multi_camera_synced_verification_tool.md)。

## SyncFramesMain.py

`SyncFramesMain.py` 是多相机同步验证的离线分析脚本。它会读取输出目录中的帧数据，根据设备 PID 选择对应帧匹配脚本，并生成匹配、未匹配和异常结果。

```bash
cd orbbec_camera/examples/multi_camera_synced_verification_tool/multicamera_sync/Python
python3 SyncFramesMain.py
```

## group_image.py

`group_image.py` 按时间戳对多相机保存的图片分组，并将分组结果复制到 `grouped_images` 目录。脚本内默认图片目录为 `/home/orbbec/image/`，使用前需按实际路径修改。

```bash
python3 orbbec_camera/scripts/group_image.py
```

## static_transforms_publisher.py

`static_transforms_publisher.py` 发布脚本中写死的多相机静态 TF，用于特定多相机调试场景。使用前需要根据实际标定修改矩阵和 frame 名称。

```bash
python3 orbbec_camera/scripts/static_transforms_publisher.py
```
