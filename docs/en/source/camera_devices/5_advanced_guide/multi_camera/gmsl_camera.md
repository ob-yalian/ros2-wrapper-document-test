# GMSL Multi-Camera Synchronization

> This section describes how to synchronize multiple GMSL-connected Gemini 330 Series cameras in OrbbecSDK_ROS2.

The example is available in [gmsl_multi_camera_sync](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main/orbbec_camera/examples/gmsl_multi_camera_sync).

## Configure device file permissions

Before starting the cameras, grant access to the camera synchronization device:

```bash
sudo chmod 777 /dev/camsync
```

## Configure the cameras

To get the `usb_port` of the GMSL camera, plug in the camera and run the following command in the terminal:

```bash
ros2 run orbbec_camera list_devices_node
```

For example, a reported GMSL camera `usb_port` may be `gmsl2-1`.

Update the `usb_port` values in [multi_gmsl_camera_synced.launch.py](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/examples/gmsl_multi_camera_sync/multi_gmsl_camera_synced.launch.py) to match the GMSL links reported by your system. The example uses the standard `gemini_330_series.launch.py` for both cameras.

## Synchronization setup

GMSL multi-camera synchronization does not require Multi-Camera Sync Hub Pro. Both cameras use `secondary_synced` mode. The receiving camera starts first; the camera that enables the host-side GMSL trigger starts two seconds later.

**Additional Parameter Settings**

* `gmsl_trigger_fps`: Sets the hardware SoC trigger source frame rate.
* `enable_gmsl_trigger`: Enables the hardware SoC trigger. Enable it on only one camera in the group.

**Run the launch**

```bash
ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py
```

> Note: Both cameras load [camera_secondary_params.yaml](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/config/camera_secondary_params.yaml), which enables color and depth by default. Modify that file to configure other streams.

## Usage Limitations of GMSL Cameras

GMSL cameras interface with various deserializer chips such as MAX9296 and MAX92716. Orbbec GMSL cameras support multiple streams including depth, color, IR, and IMU data, but certain usage limitations apply:

- GMSL only supports V4L2 and YUYV format; MJPG format is not supported. RGB output is derived from YUYV format conversion.
- Metadata for Gemini-335Lg is provided via a separate node, while metadata for other models is embedded within video frames, which remains transparent to users.
- When using the Max96712 as a deserializer chip, due to the characteristics of the Max96712 chip, a multi - machine synchronous trigger signal must be provided in the secondary_synced mode. Otherwise, data flow interruption will occur when switching the data stream
- Two cameras connected on the same MAX9296, MAX96712 LinkA/B, or MAX96712 LinkC/D have the following limitations:
  - Before driver version v1.2.02, there was a restriction that the RGB of one camera and the right IR of another camera could not stream simultaneously. After driver version v1.2.02, the restriction was modified to that the RGB of one camera and the left IR of another camera cannot stream simultaneously.
  - Before driver version v1.2.02, there was a restriction that the DEPTH of one camera and the left IR of another camera could not stream simultaneously. After driver version v1.2.02, the restriction was modified to that the DEPTH of one camera and the right IR of another camera cannot stream simultaneously.
  - The combined maximum number of active streams from both cameras is limited to four (satisfying the above two conditions ensures compliance).

For further known limitations, please refer to [Usage Limitations of Orbbec GMSL Cameras](https://github.com/orbbec/MIPI_Camera_Platform_Driver/blob/main/doc/Instructions%20for%20Using%20GMSL%20Camera.md)
