## 对齐深度到彩色

本节说明如何使用 ROS 2 将深度图像与彩色图像对齐以创建叠加图像。这对于需要来自不同传感器模态的同步视觉信息的应用特别有用。

### 对齐和查看深度和彩色图像的命令

1. **基本深度到彩色对齐：**

   要简单地将深度图像对齐到彩色图像，请使用以下命令：

   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true
   ```

   此命令激活深度配准功能，但不打开查看器。

2. **查看深度到彩色叠加：**

   如果您希望查看深度到彩色叠加，需要使用以下命令启用查看器：

   ```bash
   ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true enable_d2c_viewer:=true
   ```

   这将启动带有深度到彩色配准的相机节点并打开查看器以显示叠加图像。

### 运行时切换对齐模式

相机节点启动后，可以通过 `/camera/set_image_registration_mode` 服务切换对齐模式，无需重启节点。支持以下模式：

* `OFF`：关闭图像对齐。
* `HW_D2C`：使用硬件将深度对齐到彩色。
* `SW_D2C`：使用软件将深度对齐到彩色。
* `SW_C2D`：使用软件将彩色对齐到深度。

除 `OFF` 外，彩色流和深度流必须同时启用。服务会在切换过程中自动停止并重新启动数据流；切换失败时恢复原模式。

```bash
ros2 service call /camera/set_image_registration_mode orbbec_camera_msgs/srv/SetString "{data: SW_D2C}"
```

### 在 RViz2 中选择话题

要在 RViz2 中可视化对齐的图像：

1. 运行上述命令之一后启动 RViz2。
2. 选择深度到彩色叠加图像的话题。话题选择示例如下所示：

   ![深度到彩色叠加的话题选择](../../image/align_depth_color/image3.png)

### 深度到彩色叠加示例

在 RViz2 中选择适当的话题后，您将能够看到深度到彩色叠加图像。它可能看起来像这样：

![深度到彩色叠加图像](../../image/align_depth_color/image4.jpg)
