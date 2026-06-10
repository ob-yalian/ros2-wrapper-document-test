# Device Query and Basic Maintenance Tools

This section describes device query tools that can be used without starting the camera node, plus the basic USB permission helper script.

## list_devices_node

`list_devices_node` enumerates currently connected Orbbec devices. It prints device name, serial number, USB/network information, firmware version, preset list, preset version, and network device IP configuration status.

This tool does not require the camera node to be running. It is useful for checking device connection status before launching a camera.

Since v2.8.x, this tool also prints firmware version, preset list, preset version, local network interface name for Ethernet devices, and IP source type (`NONE`, `LLA`, `DHCP`, `PERSISTENT`). If one device fails during enumeration, the tool continues enumerating the remaining devices.

```bash
ros2 run orbbec_camera list_devices_node
```

To enable SDK file logs and attempt to enable firmware logs:

```bash
ros2 run orbbec_camera list_devices_node -- --sdk_log_level debug
```

## list_depth_work_mode_node

`list_depth_work_mode_node` queries the current depth work mode and the supported depth work mode list for the current device. It does not require the camera node to be running.

```bash
ros2 run orbbec_camera list_depth_work_mode_node
```

## list_camera_profile_mode_node

`list_camera_profile_mode_node` queries supported color, depth, IR, IMU, and LiDAR stream profiles, and also prints depth work mode and preset information. It does not require the camera node to be running.

Query the default device:

```bash
ros2 run orbbec_camera list_camera_profile_mode_node
```

Query by serial number:

```bash
ros2 run orbbec_camera list_camera_profile_mode_node -- --serial_number <SN>
```

Enable SDK file logs:

```bash
ros2 run orbbec_camera list_camera_profile_mode_node -- --sdk_log_level debug
```

## list_ob_devices.sh

`list_ob_devices.sh` is a source script that scans Orbbec USB devices from Linux `/sys/bus/usb` and prints USB port, product name, and serial number. It does not depend on the SDK and does not require the camera node.

```bash
cd orbbec_camera/scripts
./list_ob_devices.sh
```

## install_udev_rules.sh

`install_udev_rules.sh` installs USB device udev rules to solve device access permission issues for normal users. This script requires `sudo`.

```bash
cd orbbec_camera/scripts
sudo bash install_udev_rules.sh
```

The script reloads udev rules after installation.
