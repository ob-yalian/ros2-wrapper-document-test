# GigE Action Command

This example starts two Gemini 335Le cameras in Group Actions synchronization mode and one
host-side Action Command sender. The sender is intentionally created once at the top level because
a GVCP Action Command can trigger multiple cameras.

## Requirements

- Gemini 335Le firmware 1.8.24 or later
- Orbbec SDK 2.10.2 or later
- Both cameras and the host on the same network

Before running the example, change the two `net_device_ip` values in
`multi_gige_action_command.launch.py` to match the cameras.

## Start the cameras and sender

```bash
ros2 launch orbbec_camera multi_gige_action_command.launch.py
```

The launch file creates these device-scoped configuration services and one network-scoped sender:

```text
/camera_01/get_action_config
/camera_01/set_action_config
/camera_02/get_action_config
/camera_02/set_action_config
/gige_action_command_node/send_action_command
```

## Configure the cameras

Configure Action Signal block 0 on both cameras with matching keys and masks:

```bash
ros2 service call /camera_01/set_action_config \
  orbbec_camera_msgs/srv/SetActionConfig \
  "{device_key: 1, selector: 0, group_key: 1, group_mask: 1}"

ros2 service call /camera_02/set_action_config \
  orbbec_camera_msgs/srv/SetActionConfig \
  "{device_key: 1, selector: 0, group_key: 1, group_mask: 1}"
```

Read the configuration back when needed:

```bash
ros2 service call /camera_01/get_action_config \
  orbbec_camera_msgs/srv/GetActionConfig \
  "{selector: 0}"
```

## Trigger the group

Send an immediate broadcast command. Every camera whose device key, group key, and group mask
match the request will be triggered:

```bash
ros2 service call /gige_action_command_node/send_action_command \
  orbbec_camera_msgs/srv/SendActionCommand \
  "{device_key: 1, group_key: 1, group_mask: 1, destination_ip: '255.255.255.255', scheduled_time: 0}"
```

`success: true` means the host dispatched the GVCP command; the protocol does not return a device
acknowledgment. A nonzero `scheduled_time` uses a GVCP/PTP timestamp, with seconds in the upper
32 bits and nanoseconds in the lower 32 bits. Scheduled triggering requires the cameras and sender
host to use synchronized time.
