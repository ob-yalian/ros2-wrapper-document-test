# OrbbecSDK ROS2 Examples

This directory contains practical launch and tool examples. Full usage,
configuration, and troubleshooting are maintained on the official documentation
site so that instructions have a single source of truth.

For command-line maintenance and diagnostic utilities, see the official
[Tool Index](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/6_benchmark/tools.html).

## Example Index

| Source | Purpose | Experience Level | Official Guide |
| :---: | --- | :---: | --- |
| [Multi-camera network](./multi_net_camera) | Launch multiple supported Orbbec network cameras. | ⭐️ | [Network camera guide](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/configuration/net_camera.html) |
| [GigE Action Command](./gige_action_command) | Configure and trigger a Gemini 335Le camera group over GVCP. | ⭐️⭐️ | [README](./gige_action_command/README.md) |
| [GMSL camera](./gmsl_multi_camera_sync) | Synchronize two GMSL-connected Gemini 330 Series cameras. | ⭐️⭐️ | [README](./gmsl_multi_camera_sync/README.md) |
| [Shared component container](./multi_camera_shared_container) | Load multiple Gemini 330 Series cameras into one component container. | ⭐️⭐️ | [README](./multi_camera_shared_container/README.md) |
| [AE/AWB lock test](./ae_awb_lock) | Verify AE/AWB convergence, frame-metadata capture, and manual lock-in through ROS 2. | ⭐️⭐️ | [README](./ae_awb_lock/README.md) |
| [Benchmark](./benchmark) | Benchmark the performance of different camera configurations. | ⭐️⭐️ | [Performance benchmark tools](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/6_benchmark/benchmark_tools.html) |
| [Multi-camera synchronization verification](./multi_camera_synced_verification_tool) | Verify synchronization accuracy across multiple cameras. | ⭐️⭐️⭐️ | [Synchronization verification guide](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/5_advanced_guide/multi_camera/multi_camera_synced_verification_tool.html) |
