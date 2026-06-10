# Multi-Camera Helper Tools

This section describes multi-camera image saving, synchronization verification, and static TF debugging helper tools.

## multi_save_rgbir_node

`multi_save_rgbir_node` subscribes to multi-camera RGB/IR images and metadata according to `multi_save_rgbir_params.json`, and uses the `start_capture` service to trigger saving. Start the corresponding multi-camera nodes before using it.

```bash
ros2 run orbbec_camera multi_save_rgbir_node
```

The configuration file is:

```text
orbbec_camera/config/tools/multisavergbir/multi_save_rgbir_params.json
```

Trigger saving 10 frames:

```bash
ros2 service call /start_capture orbbec_camera_msgs/srv/SetInt32 "{data: 10}"
```

## image_sync_example_node

`image_sync_example_node` verifies multi-image timestamp synchronization online. It subscribes to 1 to 8 image topics, displays synchronized images, and prints timestamp difference and FPS statistics. Start the camera node before using it.

If `sync_topics` is not set, the node automatically discovers color/depth image topics:

```bash
ros2 run orbbec_camera image_sync_example_node
```

You can also specify topics manually:

```bash
ros2 run orbbec_camera image_sync_example_node \
--ros-args \
-p sync_topics:="['/camera_01/color/image_raw', '/camera_02/color/image_raw']"
```

For detailed multi-camera synchronization verification, see [Multi-Camera Synchronization Verification Tool](../5_advanced_guide/multi_camera/multi_camera_synced_verification_tool.md).

## SyncFramesMain.py

`SyncFramesMain.py` is the offline analysis script for multi-camera synchronization verification. It reads frame data from the output directory, selects the corresponding frame matching script based on device PID, and generates matched, unmatched, and abnormal results.

```bash
cd orbbec_camera/examples/multi_camera_synced_verification_tool/multicamera_sync/Python
python3 SyncFramesMain.py
```

## group_image.py

`group_image.py` groups saved multi-camera images by timestamp and copies grouped results to the `grouped_images` directory. The default image directory in the script is `/home/orbbec/image/`; modify it to match your environment before using it.

```bash
python3 orbbec_camera/scripts/group_image.py
```

## static_transforms_publisher.py

`static_transforms_publisher.py` publishes hard-coded static TFs for specific multi-camera debugging scenarios. Modify the matrices and frame names according to the actual calibration before using it.

```bash
python3 orbbec_camera/scripts/static_transforms_publisher.py
```
