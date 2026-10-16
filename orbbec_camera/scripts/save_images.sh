#!/bin/bash

# Usage: ./save_images.sh [images_per_stream]
image_count="${1:-10}"
if [[ ! "$image_count" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [positive_image_count]" >&2
  exit 1
fi

source /opt/ros/galactic/setup.bash

ros2 service call /camera_01/save_images orbbec_camera_msgs/srv/SetInt32 "{data: ${image_count}}"
ros2 service call /camera_02/save_images orbbec_camera_msgs/srv/SetInt32 "{data: ${image_count}}"
ros2 service call /camera_03/save_images orbbec_camera_msgs/srv/SetInt32 "{data: ${image_count}}"
ros2 service call /camera_04/save_images orbbec_camera_msgs/srv/SetInt32 "{data: ${image_count}}"
ros2 service call /camera_05/save_images orbbec_camera_msgs/srv/SetInt32 "{data: ${image_count}}"

wait
