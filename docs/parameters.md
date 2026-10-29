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
Property metadata is queried at declaration, after selecting the device preset;
these values are not a table of defaults for every module and are not refreshed when auto-exposure
changes a measurement. Firmware, presets and SDK versions
can change them without requiring a hard-coded default update in this package.

The ROS launch default, SDK-reported default and current device value are different concepts.
A launch value of `-1` for an optional hardware property means to skip writing it, not that
the hardware default is `-1`. Enum descriptions use the values accepted by the ROS parameter;
for example, disparity range uses 64/128/256. Structured properties such as ROI have no scalar range.
An unavailable default/range is reported as unavailable; startup continues with the basic
description. Range text does not impose the raw SDK range on launch sentinels.

Filter descriptions also include SDK defaults, ranges and steps when available. These are
read from existing filter instances after startup configuration, or from the corresponding
camera properties for property-based filters. Missing instances or schemas are reported as
unavailable; metadata queries do not create or enable filters. String modes show named choices.
Descriptions are a startup snapshot, including when a filter is first enabled later at runtime.
The SDK range can include values reserved by the ROS interface, such as `-1`; the stated
ROS startup-only rules still apply. Range metadata does not change parameter validation.

`depth_filter_config` is reserved and currently has no effect.

For devices with fixed depth precision levels, `depth_precision` accepts `1mm`, `0.8mm`,
`0.5mm`, `0.4mm`, `0.2mm`, `0.1mm` or `0.05mm`, subject to device support. Empty skips setting;
unrecognized strings log an error and leave the setting unchanged. The floating-point depth
unit path continues to use the device's supported range.

On Humble, updating filter descriptions during initialization requires re-declaring those
device parameters. This preserves their values, callbacks and reconnect overrides without
calling a device setter; parameter-event subscribers may observe the re-declarations.

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

## Legacy launch arguments

The following obsolete launch names were not read by the driver. Use the current names in
launch commands and YAML files; the old names are not aliases.

| Obsolete name | Current name |
| --- | --- |
| `use_hardware_time` | `time_domain`; the affected OpenNI launch files default to `system` |
| `flip_color`, `flip_depth`, `flip_ir` | `color_mirror`, `depth_mirror`, `ir_mirror`, matching the old mirror behavior |
| `enable_soft_filter` | `enable_noise_removal_filter` |
| `soft_filter_max_diff` | `noise_removal_filter_min_diff` |
| `soft_filter_speckle_size` | `noise_removal_filter_max_size` |

Unused `product_id`, `vendor_id` and `industry_mode` launch arguments have been removed.
Select a camera with the existing `serial_number` or `usb_port` options.

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

`param get` reads mapped camera properties from the SDK on every request, including exposure,
gain, AE/AWB, white balance, orientation, ROI and device controls. Thus `color_exposure:=-1`
still skips the startup write, but a query returns the SDK exposure property. Firmware clipping
and changes made through legacy services are visible when reflected in that property. While
automatic exposure is enabled, some devices report a different exposure or gain in frame
metadata; use the corresponding metadata topic for the values associated with each frame.

Filter queries read the existing SDK instance or corresponding device property. An absent
software filter reports disabled; its configuration is unavailable until an instance exists.
Reading never creates a filter or changes its enable state. Presets, depth precision and
synchronization settings use SDK readback; stream profiles use the selected SDK profiles.
Software-only settings use driver state or their stored configuration.

Unreadable properties, disconnected devices and SDK failures return `NOT_SET` (the CLI prints
`Parameter not set`) and log the reason. They do not fall back to the launch or last requested
value. Other values in the same query remain available. Interleave bank settings also report
unavailable because reading a different bank would require writing the device selector.

The driver provides the standard parameter service endpoints with a live `get_parameters`
handler. List, type, description and write requests continue to use rclcpp's parameter backend.
In-process `node->get_parameter()` retains the requested configuration for initialization,
partial updates and reconnect recovery; external clients and compatible getter services use
the shared live reader. Reads do not write hardware, update reconnect intent or publish parameter
events. Events still describe explicit parameter writes rather than camera-internal changes.
Parameter dumps contain readback values and therefore do not preserve launch sentinels. On Humble,
`NOT_SET` is dumped as YAML `null` but is interpreted as a string when read as a ROS parameter
override. Remove unavailable values before reusing a dump. Some image transport plugin names
also produce an empty YAML key in `ros2 param dump`; that key must be corrected before the ROS
parameter-file parser can read the file.

Successful parameter writes become reconnect overrides for the same camera serial number,
within the current driver process. Legacy services do not update ROS parameters or these
overrides. Without a parameter override, reconnect uses the launch configuration, including
`-1`. A new process starts from its launch configuration. Devices without a serial number do
not persist overrides.

`param describe` continues to report SDK default and range metadata. Depth/IR and left/right
controls may share hardware on some modules; querying either mapped property observes the SDK
value without replacing either parameter's requested reconnect configuration.

All existing service names, message types and device-setting implementations are retained.
Exposure, gain, white balance, AWB, laser enable, PTP and point-cloud decimation getter services
use the same live reader as `param get`, through a local C++ call. Read failures return the
existing service's `success=false` and error message. Status and information queries without
equivalent parameters retain their semantics: in particular, `get_ldp_status` combines LDP
enable and protection status, whereas `enable_ldp` reports only the device's enable property.
Legacy setter services retain their existing path and reconnect behavior. Dynamic controls
continue to reuse their local implementations where applicable.

For example, setting `color_exposure` to `100` through the parameter interface and then
setting exposure to `200` through a legacy service makes both `param get` and the getter
service read the device's resulting exposure. Reconnect still restores the explicit parameter
configuration `100`: querying the camera does not turn a service write or AE adjustment into
a new reconnect override.

Service request representations remain compatible. The disparity-range service and
`disparity_range_mode` both accept 64/128/256, matching the existing launch parameter.
Only the underlying SDK property uses indices 0/1/2.
ROI services keep their existing float-array order `[left, right, top, bottom]`, clipping
and readback. Parameter queries read back ROI coordinates without periodic polling.
Left/right IR exposure controls still map to `ir_*`, and left/right color controls to `color_*`.

```sh
ros2 param set /camera/camera enable_color_auto_white_balance false
ros2 param set /camera/camera color_white_balance 4500
ros2 param get /camera/camera color_white_balance
```

To query exposure through both interfaces (automatic exposure can change it between calls):

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
Live-read tests cover changing device readings, failures without cached fallback, type checks,
reader detachment and unchanged reconnect intent. Service tests cover remote reads, mixed
successful/unavailable results, and the standard list/type/description/write endpoints.
They do not exercise real SDK writes or legacy service implementations.

Hardware validation should cover Gemini 330, Gemini 336 and Gemini 305 (including Dual Color Streams):
launch with `color_exposure:=-1` and verify that `param get`, parameter dumps and the getter
service report device exposure without a startup write; compare parameter and legacy service
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
# Reads the device's resulting mirror setting (false when the write takes effect).
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
configuration retain `-1`; SDK requests resolve these values to frame boundaries, and external
parameter queries return the resulting camera ROI.

```sh
ros2 param set /camera/camera color_ae_roi_left 10
ros2 param set /camera/camera depth_ae_roi_top 20
# Reads the actual right boundary reported by the camera.
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
# Reads the resulting filter state and scale (false and 2 when the write takes effect).
ros2 param get /camera/camera enable_decimation_filter
ros2 param get /camera/camera decimation_filter_scale
```
