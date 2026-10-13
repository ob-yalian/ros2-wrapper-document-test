# Camera parameters

Use the camera node's actual name in these commands:

```sh
ros2 param describe /camera/camera color_exposure
ros2 param get /camera/camera color_exposure
```

`describe` reports the parameter's purpose and initialization constraints. For supported
hardware properties, it also includes the connected SDK/device's scalar default, minimum,
maximum and step when that metadata can be queried. These values are queried at declaration,
after selecting the device preset; they are not a table of defaults for every module, and
are not refreshed when auto-exposure changes a measurement. Firmware, presets and SDK versions
can change them without requiring a hard-coded default update in this package.

The ROS launch default, SDK-reported default and current device value are different concepts.
A launch value of `-1` for an optional hardware property means to skip writing it, not that
the hardware default is `-1`. SDK enum ranges use the SDK representation, which may differ
from a ROS string or boolean. Structured properties such as ROI have no scalar range.
An unavailable default/range is reported as unavailable; startup continues with the basic
description. Range text does not impose the raw SDK range on launch sentinels.

Only parameters for present image streams and supported hardware properties are declared.
Capability selectors such as presets and legacy `depth_work_mode` retain their existing
module-specific behavior. Availability does not imply runtime mutability.
