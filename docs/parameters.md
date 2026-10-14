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

## Runtime controls and service compatibility

The first runtime-writable controls are:

- `color_exposure`, `color_gain`, `depth_exposure`, `depth_gain`, `ir_exposure`, `ir_gain`.
- `enable_color_auto_exposure`, `enable_depth_auto_exposure`, `enable_ir_auto_exposure`.
- `color_white_balance`, `enable_color_auto_white_balance`.

Each requires the connected device to support reading and writing its SDK property.
Other parameters retain their initialization-only behavior. The new depth AE control
only writes at startup when explicitly configured, like the existing IR/color AE controls.

`param get` returns the requested configuration: the launch value or the last accepted runtime
setting. Initialization does not replace it with a device reading. For example, launching with
`color_exposure:=-1` skips the startup exposure write and leaves the ROS parameter at `-1`.
A successful runtime setting replaces that value; a failed setting leaves it unchanged.
Parameter dumps therefore retain the launch sentinel until an explicit runtime update.

Successful runtime writes through either interface become reconnect overrides for the same
camera serial number, within the current driver process. Without a runtime override, reconnect
uses the launch configuration, including `-1`. A new process starts from its launch configuration.
Devices without a serial number do not persist overrides.

The ROS parameter is not a live exposure/gain measurement while automatic control is active.
Existing getter services continue to read the device, and `param describe` reports SDK default
and range metadata. Depth/IR and left/right controls may share hardware on some modules;
changing one can affect the other's actual value without changing its requested configuration.

Existing exposure, gain, AE, white-balance and AWB setter service names and message types
are retained. They call the node parameter API, so validation, device writes, errors and
reconnect intent use the same path as `ros2 param set`. Left/right IR services map to `ir_*`,
and left/right color services map to `color_*`, matching their existing shared SDK properties.

```sh
ros2 param set /camera/camera enable_color_auto_white_balance false
ros2 param set /camera/camera color_white_balance 4500
ros2 param get /camera/camera color_white_balance
```

To compare the requested exposure with the current device value:

```sh
ros2 param get /camera/camera color_exposure
ros2 service call /camera/get_color_exposure orbbec_camera_msgs/srv/GetInt32 "{}"
```

Choose manual values within the connected device's range. Runtime `-1` and type changes are
rejected. `-1` only skips a startup write; it does not reset an option to its SDK default or
enable automatic exposure. A manual white balance requires AWB to be disabled; an atomic batch
may disable AWB and set white balance together. Automatic controls are disabled before manual writes
and enabled afterwards. Batch requests are validated before any SDK write; they cannot mix
these controls with startup-only or unrelated ROS parameters. A failed write attempts to
restore all attempted properties, including the failing one. Rollback failure is reported
explicitly: SDK/USB operations cannot guarantee hardware atomicity after a disconnect.

The Humble implementation applies hardware updates synchronously in this package's single
on-set callback. Additional callbacks that reject the request after hardware has been written
are not supported; there is no Humble post-set hook to make arbitrary callback chains and
hardware writes transactional. Services must call the parameter API without holding the
device lock. Cleanup drains and detaches the callback before destroying the camera.

## Verification

`dynamic_params_test` covers batch prevalidation, fixed runtime types, SDK-error propagation,
launch-sentinel preservation, per-device restore and callback detachment using a fake
device apply callback. It does not exercise SDK writes.

Hardware validation should cover Gemini 330, Gemini 336 and Gemini 305 (including Dual Color Streams):
launch with `color_exposure:=-1` and verify that `param get` and parameter dumps retain `-1`
while the getter service reports the actual exposure; compare parameter and legacy service
writes/readback with AE disabled; reject out-of-range and runtime `-1` writes;
disable AWB and set WB atomically; unplug during a write; reconnect after both inherited and
explicit settings; and verify absent-stream parameters remain filtered. Also check SDK range
metadata after preset selection and initialization without readable property metadata.
