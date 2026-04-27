## 注册脚本（必需）

为了让 Orbbec 相机在 Linux 上被正确识别，请安装 udev 规则。

### 二进制安装

```bash
sudo cp /opt/ros/$ROS_DISTRO/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 从源码构建

```bash
cd  ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

此步骤对于 Linux 用户是**必需的**。

`注意：` 如果不执行此脚本，由于权限问题，打开设备将会失败。您需要使用 sudo（管理员权限）运行示例程序。

### 新增设备规则

v2.8.x 版本的 udev 规则新增了 Gemini 305 系列和 Gemini 338 系列相关 PID，包括 Gemini 305 (`0840/0841`)、Gemini 305G (`0842`)、Gemini 301G (`0843`)、Gemini 338L/338Le/338Lg、Gemini 331L 等。升级 ROS 包后请重新安装 udev 规则并重新插拔设备，确保系统应用最新权限规则。
