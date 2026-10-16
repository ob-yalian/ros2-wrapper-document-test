# Camera parameters

Use the camera node's actual name in these commands:

```sh
ros2 param describe /camera/camera color_exposure
ros2 param get /camera/camera color_exposure
```

`describe` reports the parameter's purpose, units, special values and when it can be changed.
For supported numeric camera settings, it also includes the camera default, range and step
when available. Boolean defaults are shown as `true` or `false`; named choices are described
without internal numeric codes. SDK property IDs and internal names are not displayed.
These values are queried at declaration, after selecting the device preset;
they are not a table of defaults for every module and are not refreshed when auto-exposure
changes a measurement. Firmware, presets and SDK versions
can change them without requiring a hard-coded default update in this package.

The ROS launch default, SDK-reported default and current device value are different concepts.
A launch value of `-1` for an optional hardware property means to skip writing it, not that
the hardware default is `-1`. Enum descriptions use the values accepted by the ROS parameter;
for example, disparity range uses 64/128/256. Structured properties such as ROI have no scalar range.
An unavailable default/range is reported as unavailable; startup continues with the basic
description. Range text does not impose the raw SDK range on launch sentinels.

Device-dependent parameters are omitted only when the device explicitly lacks the required
sensor or SDK capability. Failed capability queries retain parameters and log a warning;
an unavailable capability query is not proof that a module is unsupported. Supported
controls remain declared when their stream or feature is disabled. Availability does not
imply runtime mutability.

- Image and IMU stream controls follow the sensor list. Synchronized IMU output requires
  both an accelerometer and a gyroscope; covariance follows the corresponding sensor.
- Preset availability is queried after JSON import, which may add a custom preset.
  `device_preset` and `depth_work_mode` are both retained when both SDK interfaces are
  supported. The existing 435Le `device_preset` mapping to legacy work modes is preserved.
  Version parameters remain available on the preset path without requiring a nonempty
  reported version. Existing selection precedence and device-setting operations are unchanged.
- Color presets, depth precision, preset resolution, synchronization, legacy/new frame
  interleave, disparity offset configuration and LRM publishing use their corresponding
  support queries or property permissions. LRM requires read access to the distance property.
  Synchronization fields are retained as a group because the mode bitmap does not describe
  each field's support separately.
- Depth filter controls require a depth sensor, but are not removed merely because a filter
  is absent from the SDK's recommended list: software and existing fallback paths remain.
  Enhanced depth follows the driver's existing supported-device rules.
- Firmware logging, monitor polling, host GMSL triggering and other settings without a
  reliable capability query remain declared. Camera connection type alone does not establish
  whether the host trigger driver is available.

Device-dependent declarations are cleared on disconnect and evaluated again on reconnect.

## Runtime controls and service compatibility

Runtime-writable means that a setting can be applied while camera streams keep running.
The parameter path does not stop/restart streams or rebuild the SDK pipeline. Runtime
availability depends on the connected module, supported properties and instantiated filters.

| Controls | Parameters |
| --- | --- |
| Exposure and white balance | Color/depth/IR exposure, gain and AE; color white balance and AWB |
| Image properties | Color AE limits/priority, brightness/ROI brightness, sharpness, gamma, saturation, contrast, hue, backlight compensation, anti-flicker, denoising and power-line frequency; depth AE priority; IR AE limit and brightness aliases |
| Orientation | Present streams' `*_mirror`, `*_flip`, `*_rotation` |
| Device controls | Laser/LDP, laser power, PTP, IR long exposure, depth limits, heartbeat, IMU data correction, AE reference/strategy, disparity range/offset and sync I/O voltage, where supported |
| AE ROI | `color_ae_roi_*`, `depth_ae_roi_*`; individual coordinates can be updated, with unspecified coordinates using full-frame boundaries |
| Depth filters | Available filters' existing enable/configuration parameters, including HDR, decimation, sequence ID, threshold, spatial, temporal, hole filling and noise removal; enhanced depth where available |
| Other stream filters | Color/left-color/right-color decimation with SDK-supported uncompressed formats; available left/right IR sequence-ID filters |
| Point clouds | Enable flags, QoS, decimation, `ordered_pc`, `cloud_frame_id`; enabling colored clouds requires color/depth streams and alignment to be already active |
| Other software controls | `publish_tf`, `tf_publish_rate` (0..1000 Hz, subject to timer precision), IMU covariance, color queue capacities, `max_save_images_count` |

Scalar SDK controls retain the read/write requirement for failure recovery. Service-backed
controls use the existing service operation and its original capability checks, readbacks,
clipping, delays, SDK call order and errors. They do not acquire a new read-permission or
readback requirement just because a dynamic parameter was added. SDK rejection while
streaming is returned as a parameter-setting failure; no automatic stream restart is attempted.

Image stream enable flags, width/height/FPS/format, MJPEG quality, alignment and other initialization-only
settings remain rejected by `param set`. Their existing services can still perform their
original stream-restart operations. Presets and synchronization/profile selectors are not
opened merely because their SDK properties are writable. Services without an existing ROS
parameter counterpart, and commands such as software trigger, reset and save, are unchanged.

Gemini 330 SDK color orientation does not process MJPG frames. Dynamic mirror/flip/rotation
requests that enable a transform require an uncompressed color profile on these devices;
legacy services retain their existing behavior. Rotation parameters accept 0/90/180/270.
Changing color decimation dimensions while software alignment is active is rejected: the
SDK Align filter retains its startup stream profiles and can crash if color dimensions change.
Configure this combination at launch. With decimation disabled, updating only its scale
keeps the filter disabled until it is explicitly enabled.
MJPEG quality remains initialization-only because Gemini 336 firmware rejects writes while
streaming. SDK read/write permission alone does not establish streaming mutability.

`param get` returns the requested configuration: the launch value or the last accepted runtime
setting. Initialization does not replace it with a device reading. For example, launching with
`color_exposure:=-1` skips the startup exposure write and leaves the ROS parameter at `-1`.
A successful runtime setting replaces that value; a failed setting leaves it unchanged.
Parameter dumps therefore retain the launch sentinel until an explicit runtime update.
The device may further constrain a valid request according to the active profile; for example,
firmware can cap an AE exposure limit. The stored ROS value remains the accepted request,
and device readback may differ.

Successful parameter writes become reconnect overrides for the same camera serial number,
within the current driver process. Legacy services do not update ROS parameters or these
overrides. Without a parameter override, reconnect uses the launch configuration, including
`-1`. A new process starts from its launch configuration. Devices without a serial number do
not persist overrides.

The ROS parameter is not a live exposure/gain measurement while automatic control is active.
Existing getter services continue to read the device, and `param describe` reports SDK default
and range metadata. Depth/IR and left/right controls may share hardware on some modules;
changing one can affect the other's actual value without changing its requested configuration.

All existing service names, message types and device-setting implementations are retained.
Services do not enter the dynamic parameter callback and do not mirror their requests into
ROS parameters. Dynamic controls reuse existing service functions through local C++ calls
where applicable; they do not send a ROS service request. Parameter-specific validation is
performed before these calls, without imposing the new rules on service clients.

For example, setting `color_exposure` to `100` through the parameter interface and then
setting exposure to `200` through a legacy service leaves the ROS parameter at `100`.
The getter service reads the device value, and reconnect restores the explicit parameter
configuration `100`. Use the parameter interface consistently for configuration that must
appear in parameter dumps and be restored on reconnect.

Service request representations remain compatible. The disparity-range service and
`disparity_range_mode` both accept 64/128/256, matching the existing launch parameter.
Only the underlying SDK property uses indices 0/1/2.
ROI services keep their existing float-array order `[left, right, top, bottom]`, clipping
and readback. ROS parameters are not refreshed by legacy service calls or periodic device reads.
Left/right IR exposure controls still map to `ir_*`, and left/right color controls to `color_*`.

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
and enabled afterwards. Changing only an AE/AWB switch does not reapply previously configured
manual exposure, gain or white balance values; include those values in the same atomic request
when they must be written together. Scalar-property batches are validated before any SDK write; they
cannot mix these controls with startup-only or unrelated ROS parameters. A failed scalar
write attempts to restore all attempted properties, including the failing one. Rollback failure is reported
explicitly: SDK/USB operations cannot guarantee hardware atomicity after a disconnect.

Update one control group at a time: scalar properties, one ROI, one filter, point-cloud
publishing, or TF. An initially unspecified HDR exposure/gain configuration requires all four values
in one atomic request. Service-backed groups preserve their existing failure/partial-application
semantics rather than adding an implicit rollback that would change the service behavior.
ROS atomic parameter storage is not a hardware transaction. Filter configuration updates
preserve the effective filter-enable state unless an enable value is explicitly supplied.

The Humble implementation applies hardware updates synchronously in this package's single
on-set callback. Additional callbacks that reject the request after hardware has been written
are not supported; there is no Humble post-set hook to make arbitrary callback chains and
hardware writes transactional. Legacy services and ROS parameter services use the node's
default mutually exclusive callback group. Cleanup drains and detaches the parameter
callback before destroying the camera.

## Verification

`dynamic_params_test` covers batch prevalidation, fixed runtime types, SDK-error propagation,
launch-sentinel preservation, per-device restore and callback detachment using a fake
device apply callback. It also covers exceptions from device setters and retained parameters
being restored without another device write or permitting startup-only runtime updates.
It does not exercise SDK writes or legacy service implementations.

Hardware validation should cover Gemini 330, Gemini 336 and Gemini 305 (including Dual Color Streams):
launch with `color_exposure:=-1` and verify that `param get` and parameter dumps retain `-1`
while the getter service reports the actual exposure; compare parameter and legacy service
writes/readback with AE disabled; reject out-of-range and runtime `-1` writes;
disable AWB and set WB atomically; unplug during a write; reconnect after both inherited and
explicit settings; and verify absent-stream parameters remain filtered. Also check SDK range
metadata after preset selection and initialization without readable property metadata.

While streaming, compare legacy setters with parameter setters and verify that no stream
stop/start occurs. Start with SDK-supported values and inspect the relevant getter/status:

```sh
ros2 param set /camera/camera color_mirror true
ros2 param get /camera/camera color_mirror
ros2 service call /camera/set_color_mirror std_srvs/srv/SetBool "{data: false}"
# Still true: legacy services do not synchronize ROS parameters.
ros2 param get /camera/camera color_mirror

ros2 param set /camera/camera point_cloud_decimation_filter_factor 2
ros2 param set /camera/camera enable_point_cloud true
ros2 param set /camera/camera tf_publish_rate 10.0

# Must fail without changing the running profiles.
ros2 param set /camera/camera color_fps 15
ros2 param set /camera/camera depth_registration true
```

ROI coordinates can be updated individually, including the first update when all four values
are `-1`. As with launch configuration, unspecified left/top coordinates use zero and
unspecified right/bottom coordinates use the active stream's width/height minus one. Other
explicitly configured coordinates are retained. Unchanged ROS parameters and their reconnect
configuration retain `-1`; only the SDK request resolves these values to frame boundaries.

```sh
ros2 param set /camera/camera color_ae_roi_left 10
ros2 param set /camera/camera depth_ae_roi_top 20
# Remains -1 if it has not been explicitly configured.
ros2 param get /camera/camera color_ae_roi_right
```

For filters, test enable/configuration requests separately and together; check
`depth_filters/status` and output frame dimensions. For example, on a device exposing a
decimation filter:

```sh
ros2 param set /camera/camera decimation_filter_scale 2
ros2 param set /camera/camera enable_decimation_filter true
ros2 service call /camera/set_filter orbbec_camera_msgs/srv/SetFilter \
  "{filter_name: DecimationFilter, filter_enable: false, filter_config: [{name: scale, value: '2'}]}"
# Still true and 2: inspect depth_filters/status for the effective service result.
ros2 param get /camera/camera enable_decimation_filter
ros2 param get /camera/camera decimation_filter_scale
```
