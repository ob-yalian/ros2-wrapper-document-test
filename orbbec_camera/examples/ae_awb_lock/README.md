# AE/AWB Lock Test

This sample exposes a test-only ROS 2 action that verifies the AE/AWB capture and manual
lock-in flow through the camera driver's services.

Run the camera driver and this sample in the same namespace:

```bash
ros2 run orbbec_camera ae_awb_lock_test_node --ros-args -r __ns:=/camera
```

Send a goal and print feedback:

```bash
ros2 action send_goal \
  /camera/run_ae_awb_lock_test \
  orbbec_camera_msgs/action/RunAeAwbLockTest \
  "{timeout_ms: 10000}" \
  --feedback
```

The sample enables auto exposure and auto white balance, waits until the SDK status equals `1`,
captures the current exposure, color gain, AWB gains, and color temperature, disables the auto
controls, and writes the captured values back in this order:

1. Color exposure
2. Color gain
3. AWB R/B/G gains
4. Color temperature

The final AWB gain readback must exactly match the captured value. Other readback differences are
reported as warnings because the device may quantize those controls. On failure or cancellation,
the sample restores auto exposure and auto white balance.

Every feedback phase contains a fresh status value read from the camera service. The
`waiting_for_services` feedback is published after all required services become available, because
the status cannot be read before its service is ready.
