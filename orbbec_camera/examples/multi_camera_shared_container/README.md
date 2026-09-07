# Attach Cameras to a Shared Component Container

This example shows how to load two Gemini 330 Series camera components into one externally
created ROS 2 component container.

Before starting the example, update the `usb_port` values in
`multi_camera_shared_container.launch.py` to match your two cameras. The default values are
`2-1` and `2-2`.

```bash
ros2 launch orbbec_camera multi_camera_shared_container.launch.py
```

The example performs three steps:

1. Starts one multithreaded component container named `shared_orbbec_container`.
2. Includes the example-specific `gemini_330_series_shared_container.launch.py` once for each
   camera.
3. Passes `attach_to_shared_component_container=true` and the same
   `component_container_name` to both includes, so neither include creates its own container.

`use_intra_process_comms=true` enables intra-process communication for the loaded camera
components. Keep every `camera_name` unique, and make sure the shared container is running
before a camera component is loaded.

The package's main `launch/gemini_330_series.launch.py` is not modified by this example. The
example-specific launcher mirrors its Gemini 330 Series parameters and adds only the component
container selection controls needed for this demonstration.
