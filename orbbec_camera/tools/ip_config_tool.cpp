#include "rclcpp/rclcpp.hpp"
#include <orbbec_camera/ob_camera_node_driver.h>
#include <orbbec_camera/utils.h>
#include <iostream>
#include <sstream>
#include <string>

using namespace ob;

bool parseIpString(const std::string &ip_str, uint8_t ip[4]) {
  std::stringstream ss(ip_str);
  std::string item;
  int i = 0;
  while (std::getline(ss, item, '.')) {
    if (i >= 4) return false;
    try {
      int num = std::stoi(item);
      if (num < 0 || num > 255) return false;
      ip[i++] = static_cast<uint8_t>(num);
    } catch (...) {
      return false;
    }
  }
  return i == 4;
}

bool isParamProvided(int argc, char **argv, const std::string &key) {
  const std::string pattern = key + ":=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg.find(pattern) != std::string::npos) {
      return true;
    }
  }
  return false;
}

void printHelp() {
  std::cout << "Usage:\n"
            << "  ros2 run orbbec_camera ip_config_tool --ros-args [params]\n"
            << "  (legacy alias: set_device_ip)\n\n"
            << "Parameters:\n"
            << "  -p old_ip:=<ip>            Current device IP (default: 192.168.1.10)\n"
            << "  -p port:=<port>            Device port (default: 8090)\n"
            << "  -p enable_lla:=<bool>      Set LLA switch directly (true: enable, false: disable, default: false)\n"
            << "                              Note: LLA is applied only when this parameter is explicitly provided.\n"
            << "  -p enable_set_ip:=<bool>   Enable set-ip operation (default: false)\n"
            << "  -p dhcp:=<bool>            DHCP flag for set-ip/force-ip config (default: false)\n"
            << "  -p new_ip:=<ip>            Static IP for set-ip/force-ip (default: 192.168.1.200)\n"
            << "  -p mask:=<ip>              Subnet mask for set-ip/force-ip (default: 255.255.255.0)\n"
            << "  -p gateway:=<ip>           Gateway for set-ip/force-ip (default: 192.168.1.1)\n"
            << "  -p enable_force_ip:=<bool> Enable force-ip operation (default: false)\n"
            << "  -p force_ip_mac:=<mac>     Target MAC for force-ip (optional, e.g. 54:14:FD:06:07:DA)\n\n"
            << "Examples:\n"
            << "\n"
            << "  [LLA]\n"
            << "    enable:  ros2 run orbbec_camera ip_config_tool --ros-args -p old_ip:=192.168.1.10 -p enable_lla:=true\n"
            << "    disable: ros2 run orbbec_camera ip_config_tool --ros-args -p old_ip:=192.168.1.10 -p enable_lla:=false\n"
            << "\n"
            << "  [Set IP]\n"
            << "    DHCP:    ros2 run orbbec_camera ip_config_tool --ros-args \\\n"
            << "             -p old_ip:=192.168.1.10 -p enable_set_ip:=true -p dhcp:=true\n"
            << "    Static:  ros2 run orbbec_camera ip_config_tool --ros-args \\\n"
            << "             -p old_ip:=192.168.1.10 -p enable_set_ip:=true -p dhcp:=false \\\n"
            << "             -p new_ip:=192.168.1.200 -p mask:=255.255.255.0 -p gateway:=192.168.1.1\n"
            << "\n"
            << "  [Force IP]\n"
            << "    by MAC:  ros2 run orbbec_camera ip_config_tool --ros-args \\\n"
            << "             -p enable_force_ip:=true \\\n"
            << "             -p force_ip_mac:=54:14:FD:06:07:DA -p dhcp:=false \\\n"
            << "             -p new_ip:=192.168.1.200 -p mask:=255.255.255.0 -p gateway:=192.168.1.1\n";
}

int main(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "-h" || arg == "--help") {
      printHelp();
      return 0;
    }
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ip_config_tool");
  auto logger = node->get_logger();

  std::string device_ip_str = node->declare_parameter<std::string>("old_ip", "192.168.1.10");
  int port = node->declare_parameter<int>("port", 8090);

  bool enable_lla = node->declare_parameter<bool>("enable_lla", false);
  bool do_lla = isParamProvided(argc, argv, "enable_lla");

  bool enable_set_ip = node->declare_parameter<bool>("enable_set_ip", false);
  bool dhcp = node->declare_parameter<bool>("dhcp", false);
  std::string new_ip_str = node->declare_parameter<std::string>("new_ip", "192.168.1.200");
  std::string mask_str = node->declare_parameter<std::string>("mask", "255.255.255.0");
  std::string gateway_str = node->declare_parameter<std::string>("gateway", "192.168.1.1");

  bool enable_force_ip = node->declare_parameter<bool>("enable_force_ip", false);
  std::string force_ip_mac = node->declare_parameter<std::string>("force_ip_mac", "");

  if (!do_lla && !enable_set_ip && !enable_force_ip) {
    RCLCPP_ERROR(logger,
                 "No operation enabled. Please enable at least one of: enable_lla, enable_set_ip, enable_force_ip.");
    rclcpp::shutdown();
    return 1;
  }

  OBNetIpConfig ip_config{};
  ip_config.dhcp = dhcp ? 1 : 0;

  if ((enable_set_ip || enable_force_ip) && !dhcp) {
    if (!parseIpString(new_ip_str, ip_config.address)) {
      RCLCPP_ERROR(logger, "Invalid new_ip format: %s", new_ip_str.c_str());
      rclcpp::shutdown();
      return 1;
    }
    if (!parseIpString(mask_str, ip_config.mask)) {
      RCLCPP_ERROR(logger, "Invalid mask format: %s", mask_str.c_str());
      rclcpp::shutdown();
      return 1;
    }
    if (!parseIpString(gateway_str, ip_config.gateway)) {
      RCLCPP_ERROR(logger, "Invalid gateway format: %s", gateway_str.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  try {
    ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
    auto context = std::make_shared<ob::Context>();

    if (do_lla || enable_set_ip) {
      RCLCPP_INFO(logger, "Connecting to device %s:%d ...", device_ip_str.c_str(), port);
      auto device = context->createNetDevice(device_ip_str.c_str(), port);

      if (do_lla) {
        if (device->isPropertySupported(OB_PROP_DEVICE_NETWORK_LLA_BOOL, OB_PERMISSION_READ_WRITE)) {
          device->setBoolProperty(OB_PROP_DEVICE_NETWORK_LLA_BOOL, enable_lla);
          RCLCPP_INFO(logger, "LLA set successfully. target=%s", enable_lla ? "enabled" : "disabled");
        } else {
          RCLCPP_WARN(logger, "LLA property is not supported on this device.");
        }
      } else {
        RCLCPP_INFO(logger, "LLA operation skipped (enable_lla not explicitly provided).");
      }

      if (enable_set_ip) {
        RCLCPP_INFO(logger, "Applying set-ip configuration...");
        device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG,
                                  reinterpret_cast<const uint8_t *>(&ip_config), sizeof(ip_config));

        RCLCPP_INFO(logger, "Set-ip configuration applied successfully.");
        if (dhcp) {
          RCLCPP_INFO(logger, "Set-ip target mode: DHCP.");
        } else {
          RCLCPP_INFO(logger, "Set-ip target static IP: %d.%d.%d.%d", ip_config.address[0],
                      ip_config.address[1], ip_config.address[2], ip_config.address[3]);
          RCLCPP_INFO(logger, "Set-ip target mask: %d.%d.%d.%d", ip_config.mask[0], ip_config.mask[1],
                      ip_config.mask[2], ip_config.mask[3]);
          RCLCPP_INFO(logger, "Set-ip target gateway: %d.%d.%d.%d", ip_config.gateway[0], ip_config.gateway[1],
                      ip_config.gateway[2], ip_config.gateway[3]);
        }
      } else {
        RCLCPP_INFO(logger, "Set-ip operation skipped (enable_set_ip=false).");
      }
    }

    if (enable_force_ip) {
      std::string mac = force_ip_mac;
      if (mac.empty()) {
        auto list = context->queryDeviceList();
        size_t ethernet_count = 0;
        std::string single_ethernet_mac;
        std::string mac_by_old_ip;

        for (size_t i = 0; i < list->deviceCount(); ++i) {
          if (std::string(list->getConnectionType(i)) != "Ethernet") {
            continue;
          }
          ++ethernet_count;
          const std::string cur_mac = list->getUid(i);
          const std::string cur_ip = list->getIpAddress(i);
          if (ethernet_count == 1) {
            single_ethernet_mac = cur_mac;
          }
          if (!device_ip_str.empty() && cur_ip == device_ip_str) {
            mac_by_old_ip = cur_mac;
          }
        }

        if (!mac_by_old_ip.empty()) {
          mac = mac_by_old_ip;
          RCLCPP_INFO(logger, "force_ip_mac not provided, selected MAC by old_ip(%s): %s",
                      device_ip_str.c_str(), mac.c_str());
        } else if (ethernet_count == 1) {
          mac = single_ethernet_mac;
          RCLCPP_INFO(logger, "force_ip_mac not provided, selected the only Ethernet device MAC: %s",
                      mac.c_str());
        } else {
          RCLCPP_ERROR(logger,
                       "force_ip_mac is required when multiple Ethernet devices exist and old_ip does not match any device.");
          rclcpp::shutdown();
          return 1;
        }
      }

      RCLCPP_INFO(logger, "Applying force-ip to MAC %s ...", mac.c_str());
      if (context->forceIp(mac.c_str(), ip_config)) {
        RCLCPP_INFO(logger, "Force-ip operation applied successfully.");
        if (dhcp) {
          RCLCPP_INFO(logger, "Force-ip target mode: DHCP.");
        } else {
          RCLCPP_INFO(logger, "Force-ip target static IP: %s", new_ip_str.c_str());
          RCLCPP_INFO(logger, "Force-ip target mask: %s", mask_str.c_str());
          RCLCPP_INFO(logger, "Force-ip target gateway: %s", gateway_str.c_str());
        }
      } else {
        RCLCPP_ERROR(logger, "Force-ip failed (SDK returned false).");
        rclcpp::shutdown();
        return 1;
      }
    }

  } catch (ob::Error &e) {
    RCLCPP_ERROR(logger, "ip_config_tool: %s", e.getMessage());
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "ip_config_tool: %s", e.what());
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    RCLCPP_ERROR(logger, "ip_config_tool: unknown error");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
