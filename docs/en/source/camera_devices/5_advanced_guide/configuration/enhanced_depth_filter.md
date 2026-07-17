# ROS 2 EnhancedDepthFilter Usage Guide

## Scope

`EnhancedDepthFilter` provides enhanced depth output for the Gemini 330 series. The relevant libraries in the current SDK version are packaged only for Linux ARM64. If this filter is enabled on another platform, the SDK reports an error directly.

## Prerequisites

1. The new LingBot License must be written to the device.

2. An external model file must be prepared.

3. Both the color and depth streams must be enabled at startup, together with D2C/C2D alignment and frame aggregation.

Pass the model file path through a ROS launch parameter:

```plaintext
enhanced_depth_model_path:=/path/to/model/file
```

If enhanced depth is enabled but the model path is empty or the file does not exist, the node fails to start.

## Launch Example

```plaintext
ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model/file \
  enhanced_depth_confidence_threshold:=51 \
  depth_registration:=true \
  frame_aggregate_mode:=full_frame
```

## Parameters

* `enable_enhanced_depth`: Whether to enable enhanced depth filtering. The default is `false`.

* `enhanced_depth_model_path`: Path to the LingBot model file. This parameter is required when enhanced depth is enabled.

* `enhanced_depth_confidence_threshold`: Confidence threshold. The default is `51`.

* `depth_registration`: Alignment must be enabled so that D2C/C2D-aligned images can enter `EnhancedDepthFilter`.

* `frame_aggregate_mode`: Frame aggregation mode. Set it to `full_frame` to ensure that color and depth images are received together.

## Image Requirements

For D2C, where depth is aligned to RGB:

* The color resolution must be one of `640x480`, `1280x720`, or `1280x800`.

* The depth resolution is unrestricted as long as the SDK supports D2C for it.

For C2D, where RGB is aligned to depth:

* The depth resolution must be one of `640x480`, `1280x720`, or `1280x800`.

* The color resolution is unrestricted.

Format requirements:

* The final color input to `EnhancedDepthFilter` is `RGB`. The driver attempts to convert certain other color formats to RGB.

* The final depth input is `Y16`. The SDK accepts certain compressed or alias formats and expands them to Y16 internally.

## Output Topics

The output images include the enhanced depth image, the depth image before D2C, and the confidence image.

The enhanced depth image is published on:

```plaintext
/camera/depth/image_raw
```

The depth image before D2C is published on:

```plaintext
/camera/depth/image_unaligned
```

The confidence image is published on:

```plaintext
/camera/confidence/image_raw
```

## Runtime Adjustment

Use the filter service to enable or disable `EnhancedDepthFilter` or adjust `confidence_threshold`.

```plaintext
ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter "{filter_name: 'EnhancedDepthFilter', filter_enable: true, filter_param: [60]}"
```

Changing `model_path` at runtime is not supported. To use a different model file, change the launch parameter and restart the node.
