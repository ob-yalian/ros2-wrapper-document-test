## 压缩图像

OrbbecSDK ROS2 支持通过 `image_transport` 发布压缩图像话题。常用场景包括降低网络传输带宽、远程查看图像，以及在 `color_format:=MJPG` 时直接订阅压缩彩色图像以降低主机端解码开销。

### 相关启动参数

压缩图像由以下 launch 参数控制：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `color.image_raw.enable_pub_plugins` | `["image_transport/compressed", "image_transport/raw", "image_transport/theora"]` | 彩色图像发布插件。 |
| `depth.image_raw.enable_pub_plugins` | `["image_transport/compressedDepth", "image_transport/raw"]` | 深度图像发布插件。 |
| `left_ir.image_raw.enable_pub_plugins` | `["image_transport/compressed", "image_transport/raw", "image_transport/theora"]` | 左红外图像发布插件。 |
| `right_ir.image_raw.enable_pub_plugins` | `["image_transport/compressed", "image_transport/raw", "image_transport/theora"]` | 右红外图像发布插件。 |

如果只需要压缩彩色图像，可以在启动命令中保留 `compressed` 和 `raw` 插件，例如：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
color.image_raw.enable_pub_plugins:='["image_transport/compressed", "image_transport/raw"]'
```

如果只需要原始图像，可以只保留 `raw` 插件，例如：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
color.image_raw.enable_pub_plugins:='["image_transport/raw"]'
```

### 常用压缩图像话题

默认相机命名空间为 `/camera`。如果启动时修改了 `camera_name`，请将下面话题中的 `/camera` 替换为实际命名空间。

| 数据流 | 压缩图像话题 |
| --- | --- |
| 彩色图像 | `/camera/color/image_raw/compressed` |
| 深度图像 | `/camera/depth/image_raw/compressedDepth` |
| 左红外图像 | `/camera/left_ir/image_raw/compressed` |
| 右红外图像 | `/camera/right_ir/image_raw/compressed` |

查看压缩彩色图像消息：

```bash
ros2 topic echo /camera/color/image_raw/compressed --no-arr
```

查看压缩深度图像消息：

```bash
ros2 topic echo /camera/depth/image_raw/compressedDepth --no-arr
```

### `color_format:=MJPG` 场景

当彩色流使用 `color_format:=MJPG` 时，ROS wrapper 会直接发布 `/camera/color/image_raw/compressed`，订阅该话题可以避免在主机侧额外解码 MJPG 图像，通常能降低 CPU 占用。

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py color_format:=MJPG
```

此时建议订阅：

```bash
ros2 topic echo /camera/color/image_raw/compressed --no-arr
```

如果订阅 `/camera/color/image_raw`，MJPG 图像仍需要在主机侧解码，CPU 占用会更高。更多低 CPU 配置建议请参考 [降低 CPU 使用率](../5_advanced_guide/performance/lower_cpu_usage.md)。

### 排查方法

如果没有看到压缩图像话题，请先检查话题列表：

```bash
ros2 topic list | grep image_raw
```

然后确认对应的 `*.image_raw.enable_pub_plugins` 参数没有被改为只启用 `raw`，并确认系统已安装 `image_transport` 相关插件。
