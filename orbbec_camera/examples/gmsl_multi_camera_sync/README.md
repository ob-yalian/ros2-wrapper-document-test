# GMSL Multi-Camera Synchronization

This example starts two GMSL-connected Gemini 330 Series cameras in hardware synchronization
mode. It uses the package's standard `gemini_330_series.launch.py` file.

Before starting the cameras, grant access to the camera synchronization device:

```bash
sudo chmod 777 /dev/camsync
```

Before starting the example, update the `usb_port` values in
`multi_gmsl_camera_synced.launch.py` if your system reports different GMSL links. The default
values are `gmsl2-1` and `gmsl2-3`.

```bash
ros2 launch orbbec_camera multi_gmsl_camera_synced.launch.py
```

Both cameras use `secondary_synced` mode. `camera_01` enables the host-side GMSL trigger at
30 fps (`gmsl_trigger_fps=3000`), while `camera_02` receives the same trigger. The receiving
camera is started first, followed by the trigger-generating camera after two seconds.
