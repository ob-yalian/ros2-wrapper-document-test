# firmware_update_tool

`firmware_update_tool` upgrades device firmware or writes preset files from the ROS 2 command line. Before upgrading, make sure the device connection is stable. When multiple devices are connected, specify the serial number to avoid updating the wrong device.

Show help:

```bash
ros2 run orbbec_camera firmware_update_tool -- --help
```

Upgrade firmware for one device:

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number <SN> \
--firmware_path /path/to/firmware.bin
```

Write a preset file:

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number <SN> \
--preset_path /path/to/preset.bin
```

For batch updates, `--serial_number` accepts comma-separated values. Add `--continue_on_error` if later devices should still be processed after one device fails.

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number SN1,SN2 \
--firmware_path /path/to/firmware.bin \
--continue_on_error
```
