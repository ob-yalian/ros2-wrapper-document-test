#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "orbbec_camera/ob_camera_node.h"
#include "orbbec_camera/utils.h"
#include "orbbec_camera_msgs/msg/metadata.hpp"

namespace orbbec_camera {
namespace tools {
namespace {

const std::array<std::string, 6> kSupportedStreamNames = {
    "color", "left_color", "right_color", "ir", "left_ir", "right_ir",
};

bool isSupportedStreamName(const std::string &stream_name) {
  return std::find(kSupportedStreamNames.begin(), kSupportedStreamNames.end(), stream_name) !=
         kSupportedStreamNames.end();
}

bool isColorCaptureStreamName(const std::string &stream_name) {
  return stream_name == "color" || stream_name == "left_color" || stream_name == "right_color";
}

std::string cameraNamespace(const std::string &camera_name) {
  if (!camera_name.empty() && camera_name.front() == '/') {
    return camera_name;
  }
  return "/" + camera_name;
}

}  // namespace

struct StreamCapture {
  std::vector<cv::Mat> images;
  std::vector<std::string> current_timestamps;
  std::vector<std::string> receive_timestamps;
  std::vector<std::string> exposures;
  std::vector<std::string> gains;

  void clear() {
    images.clear();
    current_timestamps.clear();
    receive_timestamps.clear();
    exposures.clear();
    gains.clear();
  }
};

class MultiCameraSubscriber : public rclcpp::Node {
 public:
  explicit MultiCameraSubscriber(const rclcpp::NodeOptions &options)
      : Node("MultiCameraSubscriber", options) {
    initializeDeviceInfo();
    loadParameters();
    for (size_t i = 0; i < usb_ports_.size(); ++i) {
      usb_index_map_[usb_ports_[i]] = static_cast<int>(i);
    }
    for (const auto &entry : serial_numbers_) {
      RCLCPP_INFO(get_logger(), "usb_port: %s, serial: %s", entry.first.c_str(),
                  entry.second.c_str());
    }
    capture_control_srv_ = this->create_service<orbbec_camera_msgs::srv::SetInt32>(
        "start_capture", std::bind(&MultiCameraSubscriber::controlCaptureCallback, this,
                                   std::placeholders::_1, std::placeholders::_2));
  }

 private:
  void initializeDeviceInfo() {
    try {
      auto context = std::make_unique<ob::Context>();
      context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
      auto list = context->queryDeviceList();
      for (size_t i = 0; i < list->deviceCount(); ++i) {
        auto device_info = list->getDevice(i)->getDeviceInfo();
        const auto usb_port = parseUsbPort(device_info->uid());
        serial_numbers_[usb_port] = device_info->serialNumber();
        has_gemini330_device_ = has_gemini330_device_ || isGemini330Series(device_info->getPid());
      }
    } catch (const ob::Error &e) {
      RCLCPP_ERROR_STREAM(get_logger(), orbbec_camera::formatObErrorWithStatus(e));
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "unknown error while querying devices");
    }
  }

  bool isGemini330Series(uint32_t pid) const {
    return pid == GEMINI_335_PID || pid == GEMINI_330_PID || pid == GEMINI_336_PID ||
           pid == GEMINI_335L_PID || pid == GEMINI_330L_PID || pid == GEMINI_336L_PID ||
           pid == GEMINI_335LG_PID || pid == GEMINI_336LG_PID || pid == GEMINI_335LE_PID ||
           pid == GEMINI_336LE_PID || pid == CUSTOM_ADVANTECH_GEMINI_336_PID ||
           pid == CUSTOM_ADVANTECH_GEMINI_336L_PID || pid == GEMINI_338_PID ||
           pid == GEMINI_338LG_PID || pid == GEMINI_338LE_PID || pid == GEMINI_338L_PID ||
           pid == GEMINI_331L_PID;
  }

  void loadParameters() {
    std::ifstream file(
        "install/orbbec_camera/share/orbbec_camera/config/tools/multisavergbir/"
        "multi_save_rgbir_params.json");
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open JSON file.");
      return;
    }

    nlohmann::json json_data;
    file >> json_data;
    const auto &params = json_data["save_rgbir_params"];
    const auto time_domain = params["time_domain"].get<std::string>();
    time_domain_suffix_ =
        time_domain == "device" ? "_d" : (time_domain == "global" ? "_g" : "_unknown");
    usb_ports_ = params["usb_ports"].get<std::vector<std::string>>();
    camera_names_ = params["camera_name"].get<std::vector<std::string>>();

    if (params.contains("stream_names")) {
      for (const auto &stream_name : params["stream_names"].get<std::vector<std::string>>()) {
        if (!isSupportedStreamName(stream_name)) {
          throw std::invalid_argument("Unsupported stream name in multi_save_rgbir config: " +
                                      stream_name);
        }
        if (std::find(configured_stream_names_.begin(), configured_stream_names_.end(),
                      stream_name) == configured_stream_names_.end()) {
          configured_stream_names_.push_back(stream_name);
        }
      }
    }
  }

  std::vector<std::string> discoverStreamNames(const std::string &camera_name) const {
    std::vector<std::string> stream_names;
    const auto names_and_types = this->get_topic_names_and_types();
    const std::string prefix = cameraNamespace(camera_name) + "/";
    for (const auto &stream_name : kSupportedStreamNames) {
      const std::string topic = prefix + stream_name + "/image_raw";
      const auto topic_it = names_and_types.find(topic);
      if (topic_it == names_and_types.end()) {
        continue;
      }
      const auto &types = topic_it->second;
      if (std::find(types.begin(), types.end(), "sensor_msgs/msg/Image") != types.end()) {
        stream_names.push_back(stream_name);
      }
    }
    return stream_names;
  }

  std::vector<std::string> selectStreamNames(const std::string &camera_name) const {
    if (!configured_stream_names_.empty()) {
      return configured_stream_names_;
    }
    auto stream_names = discoverStreamNames(camera_name);
    if (!stream_names.empty()) {
      return stream_names;
    }

    RCLCPP_WARN(get_logger(),
                "No supported image topics discovered for %s; using legacy RGB/IR topics",
                camera_name.c_str());
    return {has_gemini330_device_ ? "left_ir" : "ir", "color"};
  }

  void initializeTopics() {
    captures_.resize(camera_names_.size());
    callback_called_.assign(camera_names_.size(), false);
    const auto custom_qos =
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

    for (size_t camera_index = 0; camera_index < camera_names_.size(); ++camera_index) {
      const auto stream_names = selectStreamNames(camera_names_[camera_index]);
      const std::string prefix = cameraNamespace(camera_names_[camera_index]) + "/";
      auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      callback_groups_.push_back(callback_group);
      rclcpp::SubscriptionOptions options;
      options.callback_group = callback_group;

      for (const auto &stream_name : stream_names) {
        captures_[camera_index].emplace(stream_name, StreamCapture{});
        const std::string image_topic = prefix + stream_name + "/image_raw";
        const std::string metadata_topic = prefix + stream_name + "/metadata";
        RCLCPP_INFO(get_logger(), "Subscribing to %s", image_topic.c_str());

        image_subscribers_.push_back(this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, custom_qos,
            [this, camera_index,
             stream_name](const std::shared_ptr<const sensor_msgs::msg::Image> image) {
              imageCallback(image, camera_index, stream_name);
            },
            options));
        metadata_subscribers_.push_back(
            this->create_subscription<orbbec_camera_msgs::msg::Metadata>(
                metadata_topic, custom_qos,
                [this, camera_index, stream_name](
                    const std::shared_ptr<const orbbec_camera_msgs::msg::Metadata> metadata) {
                  metadataCallback(metadata, camera_index, stream_name);
                },
                options));
      }
    }
  }

  std::string currentDateTime() const {
    const auto now = std::chrono::system_clock::now();
    const auto now_time = std::chrono::system_clock::to_time_t(now);
    const std::tm time_info = *std::localtime(&now_time);
    std::ostringstream output;
    output << std::put_time(&time_info, "%Y%m%d%H%M%S");
    return output.str();
  }

  std::string generateFolderName(const std::string &serial_number, size_t serial_index) const {
    const std::string path = "multicamera_sync/output/" + current_date_time_ +
                             "/TotalModeFrames/SN" + serial_number + "_Index" +
                             std::to_string(serial_index);
    std::filesystem::create_directories(path);
    return path;
  }

  std::string receiveTimestamp() const {
    const auto now = this->get_clock()->now();
    const int64_t seconds = now.seconds();
    const int64_t milliseconds = now.nanoseconds() % 1000000000 / 1000000;
    return std::to_string(seconds) + std::to_string(milliseconds);
  }

  std::string imageTimestamp(const sensor_msgs::msg::Image::ConstSharedPtr &image) const {
    const int64_t milliseconds = image->header.stamp.nanosec / 1000000;
    std::ostringstream timestamp;
    timestamp << image->header.stamp.sec << std::setw(3) << std::setfill('0') << milliseconds;
    return timestamp.str();
  }

  bool captureReady(size_t camera_index) const {
    if (camera_index >= captures_.size() || captures_[camera_index].empty()) {
      return false;
    }
    return std::all_of(
        captures_[camera_index].begin(), captures_[camera_index].end(), [this](const auto &entry) {
          return entry.second.images.size() >= static_cast<size_t>(saving_images_number_);
        });
  }

  std::string metadataSuffix(const StreamCapture &capture, size_t frame_index) const {
    std::string suffix;
    if (frame_index < capture.exposures.size()) {
      suffix += "_e" + capture.exposures[frame_index];
    }
    if (frame_index < capture.gains.size()) {
      suffix += "_d" + capture.gains[frame_index];
    }
    return suffix;
  }

  void saveImages(size_t camera_index) {
    if (!captureReady(camera_index)) {
      return;
    }
    if (camera_index >= usb_ports_.size()) {
      RCLCPP_ERROR(get_logger(), "Missing USB port configuration for camera index %zu",
                   camera_index);
      return;
    }
    const auto serial_it = serial_numbers_.find(usb_ports_[camera_index]);
    if (serial_it == serial_numbers_.end()) {
      RCLCPP_ERROR(get_logger(), "No serial number found for USB port %s",
                   usb_ports_[camera_index].c_str());
      return;
    }
    const auto usb_index_it = usb_index_map_.find(usb_ports_[camera_index]);
    const size_t usb_index = usb_index_it == usb_index_map_.end()
                                 ? camera_index
                                 : static_cast<size_t>(usb_index_it->second);
    const std::string &serial_number = serial_it->second;
    const std::string folder = generateFolderName(serial_number, usb_index);
    callback_called_[camera_index] = true;

    for (const auto &entry : captures_[camera_index]) {
      const std::string &stream_name = entry.first;
      const auto &capture = entry.second;
      for (size_t i = 0; i < static_cast<size_t>(saving_images_number_); ++i) {
        if (capture.images[i].empty()) {
          continue;
        }
        const std::string filename = folder + "/" + stream_name + "_SN" + serial_number + "_Index" +
                                     std::to_string(usb_index) + time_domain_suffix_ +
                                     capture.current_timestamps[i] + "_f" + std::to_string(i) +
                                     "_s" + capture.receive_timestamps[i] +
                                     metadataSuffix(capture, i) + "_.jpg";
        cv::imwrite(filename, capture.images[i]);
      }
    }

    for (auto &entry : captures_[camera_index]) {
      entry.second.clear();
    }
    const bool all_cameras_complete = std::all_of(callback_called_.begin(), callback_called_.end(),
                                                  [](bool value) { return value; });
    if (all_cameras_complete) {
      RCLCPP_INFO(get_logger(), "Capture completed for all cameras");
      saving_images_number_ = 0;
      callback_called_.assign(camera_names_.size(), false);
    }
  }

  void controlCaptureCallback(
      const std::shared_ptr<orbbec_camera_msgs::srv::SetInt32::Request> request,
      std::shared_ptr<orbbec_camera_msgs::srv::SetInt32::Response> response) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    if (request->data <= 0) {
      response->success = false;
      response->message = "capture image count must be greater than zero";
      return;
    }
    if (!topics_initialized_) {
      initializeTopics();
      topics_initialized_ = true;
    }
    if (captures_.empty() || std::any_of(captures_.begin(), captures_.end(),
                                         [](const auto &streams) { return streams.empty(); })) {
      response->success = false;
      response->message = "no supported image streams are configured";
      return;
    }

    for (auto &camera_captures : captures_) {
      for (auto &entry : camera_captures) {
        entry.second.clear();
      }
    }
    callback_called_.assign(camera_names_.size(), false);
    current_date_time_ = currentDateTime();
    saving_images_number_ = request->data;
    response->success = true;
    response->message = "capture started";
    RCLCPP_INFO(get_logger(), "Capturing %d image(s) from each configured stream",
                saving_images_number_);
  }

  void imageCallback(const std::shared_ptr<const sensor_msgs::msg::Image> image,
                     size_t camera_index, const std::string &stream_name) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    if (saving_images_number_ <= 0 || callback_called_[camera_index]) {
      return;
    }
    auto &capture = captures_[camera_index].at(stream_name);
    cv::Mat output = cv_bridge::toCvCopy(image, image->encoding)->image;
    if (isColorCaptureStreamName(stream_name) &&
        image->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::Mat converted;
      cv::cvtColor(output, converted, cv::COLOR_RGB2BGR);
      output = converted;
    }
    capture.images.push_back(output);
    capture.current_timestamps.push_back(imageTimestamp(image));
    capture.receive_timestamps.push_back(receiveTimestamp());
    RCLCPP_INFO(get_logger(), "%s[%zu]: %zu/%d", stream_name.c_str(), camera_index,
                capture.images.size(), saving_images_number_);
    saveImages(camera_index);
  }

  void metadataCallback(const std::shared_ptr<const orbbec_camera_msgs::msg::Metadata> metadata,
                        size_t camera_index, const std::string &stream_name) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    if (saving_images_number_ <= 0 || callback_called_[camera_index]) {
      return;
    }
    try {
      const auto json_data = nlohmann::json::parse(metadata->json_data);
      auto &capture = captures_[camera_index].at(stream_name);
      if (json_data.contains("exposure")) {
        capture.exposures.push_back(json_data["exposure"].dump());
      }
      if (json_data.contains("gain")) {
        capture.gains.push_back(json_data["gain"].dump());
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed to parse %s metadata: %s", stream_name.c_str(), e.what());
    }
  }

  std::mutex capture_mutex_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;
  std::vector<rclcpp::Subscription<orbbec_camera_msgs::msg::Metadata>::SharedPtr>
      metadata_subscribers_;
  rclcpp::Service<orbbec_camera_msgs::srv::SetInt32>::SharedPtr capture_control_srv_;

  std::map<std::string, int> usb_index_map_;
  std::map<std::string, std::string> serial_numbers_;
  std::vector<std::string> usb_ports_;
  std::vector<std::string> camera_names_;
  std::vector<std::string> configured_stream_names_;
  std::vector<std::map<std::string, StreamCapture>> captures_;
  std::vector<bool> callback_called_;
  std::string time_domain_suffix_;
  std::string current_date_time_;
  int saving_images_number_ = 0;
  bool topics_initialized_ = false;
  bool has_gemini330_device_ = false;
};

}  // namespace tools
}  // namespace orbbec_camera

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::tools::MultiCameraSubscriber)
