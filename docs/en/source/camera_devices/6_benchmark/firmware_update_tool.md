# firmware_update_tool Tool

`firmware_update_tool` updates device firmware or burns preset files from the ROS 2 command line. Before updating, make sure the device connection is stable. When multiple devices are connected, specify serial numbers to avoid updating the wrong device.

Show help:

```bash
ros2 run orbbec_camera firmware_update_tool -- --help
```

Update firmware for one device:

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number <SN> \
--firmware_path /path/to/firmware.bin
```

Burn a preset file:

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number <SN> \
--preset_path /path/to/preset.bin
```

For batch updates, `--serial_number` accepts comma-separated serial numbers. Add `--continue_on_error` if later devices should still be processed after one device fails.

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number SN1,SN2 \
--firmware_path /path/to/firmware.bin \
--continue_on_error
```

To enable SDK file logs and also attempt to enable firmware logs, add `--sdk_log_level`. Optional values are `debug`, `info`, `warn`, `error`, `fatal`, and `off`; the default is `off`.

```bash
ros2 run orbbec_camera firmware_update_tool -- \
--serial_number <SN> \
--preset_path /path/to/preset.bin \
--sdk_log_level debug
```
