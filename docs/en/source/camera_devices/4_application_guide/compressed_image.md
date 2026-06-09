## Compressed Image

OrbbecSDK ROS2 supports publishing compressed image topics through `image_transport`. This is useful for reducing network bandwidth, viewing images remotely, and subscribing to the compressed color image directly when `color_format:=MJPG` is used to reduce host-side decoding overhead.

### Related Launch Parameters

Compressed image publishing is controlled by the following launch parameters:

| Parameter | Default Value | Description |
| --- | --- | --- |
| `color.image_raw.enable_pub_plugins` | `["image_transport/compressed", "image_transport/raw", "image_transport/theora"]` | Color image publishing plugins. |
| `depth.image_raw.enable_pub_plugins` | `["image_transport/compressedDepth", "image_transport/raw"]` | Depth image publishing plugins. |
| `left_ir.image_raw.enable_pub_plugins` | `["image_transport/compressed", "image_transport/raw", "image_transport/theora"]` | Left IR image publishing plugins. |
| `right_ir.image_raw.enable_pub_plugins` | `["image_transport/compressed", "image_transport/raw", "image_transport/theora"]` | Right IR image publishing plugins. |

If you only need compressed color images, keep the `compressed` and `raw` plugins in the launch command, for example:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
color.image_raw.enable_pub_plugins:='["image_transport/compressed", "image_transport/raw"]'
```

If you only need raw images, keep only the `raw` plugin, for example:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
color.image_raw.enable_pub_plugins:='["image_transport/raw"]'
```

### Common Compressed Image Topics

The default camera namespace is `/camera`. If you change `camera_name` when launching the camera, replace `/camera` in the following topics with the actual namespace.

| Stream | Compressed Image Topic |
| --- | --- |
| Color image | `/camera/color/image_raw/compressed` |
| Depth image | `/camera/depth/image_raw/compressedDepth` |
| Left IR image | `/camera/left_ir/image_raw/compressed` |
| Right IR image | `/camera/right_ir/image_raw/compressed` |

View compressed color image messages:

```bash
ros2 topic echo /camera/color/image_raw/compressed --no-arr
```

View compressed depth image messages:

```bash
ros2 topic echo /camera/depth/image_raw/compressedDepth --no-arr
```

### `color_format:=MJPG` Scenario

When the color stream uses `color_format:=MJPG`, the ROS wrapper directly publishes `/camera/color/image_raw/compressed`. Subscribing to this topic avoids extra host-side MJPG decoding and usually reduces CPU usage.

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py color_format:=MJPG
```

In this case, subscribe to:

```bash
ros2 topic echo /camera/color/image_raw/compressed --no-arr
```

If you subscribe to `/camera/color/image_raw`, the MJPG image still needs to be decoded on the host, which increases CPU usage. For more low-CPU configuration suggestions, see [Reducing CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md).

### Troubleshooting

If you do not see compressed image topics, check the topic list first:

```bash
ros2 topic list | grep image_raw
```

Then confirm that the corresponding `*.image_raw.enable_pub_plugins` parameter has not been changed to enable only `raw`, and make sure the related `image_transport` plugins are installed.
