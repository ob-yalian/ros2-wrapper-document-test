# ROS2 EnhancedDepthFilter 使用说明

## 适用范围

EnhancedDepthFilter 用于 Gemini 330 系列的增强深度输出。当前 SDK 版本仅在 Linux arm64 平台打包相关库；其它平台如果启用该滤波，SDK 会直接报错。

## 前置条件

1. 设备需要写入新版 LingBot License。
2. 需要准备外挂模型文件。
3. 启动时必须同时开启 color 和 depth，并启用 D2C/C2D 对齐和帧汇聚功能。

模型文件路径通过 ROS launch 参数传入：

```plaintext
enhanced_depth_model_path:=/path/to/model/file
```

如果开启增强深度但模型路径为空或文件不存在，节点会启动失败。

## 启动示例

```plaintext
ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model/file \
  enhanced_depth_confidence_threshold:=51 \
  depth_registration:=true \
  frame_aggregate_mode:=full_frame
```

## 参数说明

* `enable_enhanced_depth`: 是否启用增强深度滤波，默认 `false`。
* `enhanced_depth_model_path`: LingBot 模型文件路径，启用增强深度时必填。
* `enhanced_depth_confidence_threshold`: 置信度阈值，默认 51。
* `depth_registration`: 需要启用对齐，D2C/C2D 后的图像才能进入 EnhancedDepthFilter。
* `frame_aggregate_mode`: 帧汇聚功能，需设置`full_frame`保证同时接收到color和depth图像。

## 图像要求

D2C 时，深度对齐到 RGB：

* color 分辨率必须是 `640x480`、`1280x720` 或 `1280x800` 之一。
* depth 分辨率不限制，只要 SDK 支持 D2C。

C2D 时，RGB 对齐到 depth：

* depth 分辨率必须是 `640x480`、`1280x720` 或 `1280x800` 之一。
* color 分辨率不限制。

格式要求：

* EnhancedDepthFilter 最终输入的 color 为 `RGB`。驱动会尝试把部分其它 color 格式转换为 RGB。
* depth 最终输入为 `Y16`。SDK 可接受部分压缩/别名格式，并在内部展开为 Y16。

## 输出话题

输出的图像有：优化后的深度图像、d2c之前的深度图像、置信度图像。

优化后的深度图像发布在：

```plaintext
/camera/depth/image_raw
```

d2c之前的深度图像发布在：

```plaintext
/camera/depth/image_unaligned
```

置信度图像发布在：

```plaintext
/camera/confidence/image_raw
```

## 运行时调整

可以通过滤波服务启停 EnhancedDepthFilter 或调整 `confidence_threshold`。

```plaintext
ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: 'EnhancedDepthFilter', filter_enable: true, filter_param: [60]}"
```

不支持运行时修改 `model_path`。如需更换模型文件，请修改 launch 参数后重新启动节点。
