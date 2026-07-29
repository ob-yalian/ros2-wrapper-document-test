// Copyright (c) 2026 Orbbec Inc. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "orbbec_camera_msgs/action/run_ae_awb_lock_test.hpp"
#include "orbbec_camera_msgs/srv/get_awb_gain.hpp"
#include "orbbec_camera_msgs/srv/get_int32.hpp"
#include "orbbec_camera_msgs/srv/set_awb_gain.hpp"
#include "orbbec_camera_msgs/srv/set_int32.hpp"

namespace {

using namespace std::chrono_literals;

constexpr int32_t kAeAwbConverged = 1;
constexpr uint32_t kDefaultTimeoutMs = 10000;
constexpr auto kPollInterval = 100ms;
constexpr auto kServiceWaitTimeout = 1s;
constexpr auto kServiceCallTimeout = 2s;

class CanceledError : public std::runtime_error {
 public:
  CanceledError() : std::runtime_error("goal canceled") {}
};

class AeAwbLockTestNode : public rclcpp::Node {
 public:
  using RunAeAwbLockTest = orbbec_camera_msgs::action::RunAeAwbLockTest;
  using GoalHandle = rclcpp_action::ServerGoalHandle<RunAeAwbLockTest>;
  using GetInt32 = orbbec_camera_msgs::srv::GetInt32;
  using SetInt32 = orbbec_camera_msgs::srv::SetInt32;
  using GetAwbGain = orbbec_camera_msgs::srv::GetAwbGain;
  using SetAwbGain = orbbec_camera_msgs::srv::SetAwbGain;
  using SetBool = std_srvs::srv::SetBool;

  AeAwbLockTestNode() : Node("ae_awb_lock_test_node") {
    get_status_client_ = create_client<GetInt32>("get_color_ae_awb_status");
    get_awb_gain_client_ = create_client<GetAwbGain>("get_color_awb_gain");
    set_awb_gain_client_ = create_client<SetAwbGain>("set_color_awb_gain");
    get_exposure_client_ = create_client<GetInt32>("get_color_exposure");
    set_exposure_client_ = create_client<SetInt32>("set_color_exposure");
    get_color_gain_client_ = create_client<GetInt32>("get_color_gain");
    set_color_gain_client_ = create_client<SetInt32>("set_color_gain");
    get_white_balance_client_ = create_client<GetInt32>("get_white_balance");
    set_white_balance_client_ = create_client<SetInt32>("set_white_balance");
    set_auto_exposure_client_ = create_client<SetBool>("set_color_auto_exposure");
    set_auto_white_balance_client_ = create_client<SetBool>("set_auto_white_balance");

    action_server_ = rclcpp_action::create_server<RunAeAwbLockTest>(
        this, "run_ae_awb_lock_test",
        std::bind(&AeAwbLockTestNode::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&AeAwbLockTestNode::handleCancel, this, std::placeholders::_1),
        std::bind(&AeAwbLockTestNode::handleAccepted, this, std::placeholders::_1));
  }

  ~AeAwbLockTestNode() override {
    shutting_down_.store(true);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  struct ActiveGoalGuard {
    explicit ActiveGoalGuard(std::atomic_bool& active) : active_(active) {}
    ~ActiveGoalGuard() { active_.store(false); }
    std::atomic_bool& active_;
  };

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const RunAeAwbLockTest::Goal> goal) {
    (void)uuid;
    (void)goal;
    bool expected = false;
    if (!goal_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejecting AE/AWB test goal: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread(&AeAwbLockTestNode::execute, this, goal_handle);
  }

  template <typename ServiceT>
  void waitForService(const typename rclcpp::Client<ServiceT>::SharedPtr& client,
                      const std::string& service_name) {
    if (!client->wait_for_service(kServiceWaitTimeout)) {
      throw std::runtime_error(service_name + " is not available");
    }
  }

  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr callService(
      const typename rclcpp::Client<ServiceT>::SharedPtr& client,
      const typename ServiceT::Request::SharedPtr& request, const std::string& service_name) {
    auto future = client->async_send_request(request);
    if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
      throw std::runtime_error(service_name + " timed out");
    }
    auto response = future.get();
    if (!response->success) {
      throw std::runtime_error(service_name + " failed: " + response->message);
    }
    return response;
  }

  void waitForRequiredServices() {
    waitForService<GetInt32>(get_status_client_, "get_color_ae_awb_status");
    waitForService<GetAwbGain>(get_awb_gain_client_, "get_color_awb_gain");
    waitForService<SetAwbGain>(set_awb_gain_client_, "set_color_awb_gain");
    waitForService<GetInt32>(get_exposure_client_, "get_color_exposure");
    waitForService<SetInt32>(set_exposure_client_, "set_color_exposure");
    waitForService<GetInt32>(get_color_gain_client_, "get_color_gain");
    waitForService<SetInt32>(set_color_gain_client_, "set_color_gain");
    waitForService<GetInt32>(get_white_balance_client_, "get_white_balance");
    waitForService<SetInt32>(set_white_balance_client_, "set_white_balance");
    waitForService<SetBool>(set_auto_exposure_client_, "set_color_auto_exposure");
    waitForService<SetBool>(set_auto_white_balance_client_, "set_auto_white_balance");
  }

  int32_t getIntValue(const rclcpp::Client<GetInt32>::SharedPtr& client,
                      const std::string& service_name) {
    return callService<GetInt32>(client, std::make_shared<GetInt32::Request>(), service_name)->data;
  }

  int32_t getAeAwbStatus() { return getIntValue(get_status_client_, "get_color_ae_awb_status"); }

  void setIntValue(const rclcpp::Client<SetInt32>::SharedPtr& client,
                   const std::string& service_name, int32_t value) {
    auto request = std::make_shared<SetInt32::Request>();
    request->data = value;
    callService<SetInt32>(client, request, service_name);
  }

  GetAwbGain::Response::SharedPtr getAwbGain() {
    return callService<GetAwbGain>(get_awb_gain_client_, std::make_shared<GetAwbGain::Request>(),
                                   "get_color_awb_gain");
  }

  void setAwbGain(uint16_t r_gain, uint16_t b_gain, uint16_t g_gain) {
    auto request = std::make_shared<SetAwbGain::Request>();
    request->r_gain = r_gain;
    request->b_gain = b_gain;
    request->g_gain = g_gain;
    callService<SetAwbGain>(set_awb_gain_client_, request, "set_color_awb_gain");
  }

  void setBoolValue(const rclcpp::Client<SetBool>::SharedPtr& client,
                    const std::string& service_name, bool value) {
    auto request = std::make_shared<SetBool::Request>();
    request->data = value;
    callService<SetBool>(client, request, service_name);
  }

  void setAutoMode(bool enabled) {
    setBoolValue(set_auto_exposure_client_, "set_color_auto_exposure", enabled);
    setBoolValue(set_auto_white_balance_client_, "set_auto_white_balance", enabled);
  }

  void restoreAutoMode() noexcept {
    try {
      setBoolValue(set_auto_exposure_client_, "set_color_auto_exposure", true);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to restore auto exposure: %s", e.what());
    }
    try {
      setBoolValue(set_auto_white_balance_client_, "set_auto_white_balance", true);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to restore auto white balance: %s", e.what());
    }
  }

  void throwIfCanceled(const std::shared_ptr<GoalHandle>& goal_handle) const {
    if (shutting_down_.load() || goal_handle->is_canceling()) {
      throw CanceledError();
    }
  }

  void publishFeedback(const std::shared_ptr<GoalHandle>& goal_handle, const std::string& phase,
                       int32_t status) {
    auto feedback = std::make_shared<RunAeAwbLockTest::Feedback>();
    feedback->phase = phase;
    feedback->ae_awb_status = status;
    goal_handle->publish_feedback(feedback);
  }

  void warnIfChanged(const char* name, int64_t captured, int64_t actual) {
    if (captured != actual) {
      RCLCPP_WARN(get_logger(), "%s changed from %lld to %lld after writeback", name,
                  static_cast<long long>(captured), static_cast<long long>(actual));
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    ActiveGoalGuard active_guard(goal_active_);
    auto result = std::make_shared<RunAeAwbLockTest::Result>();
    bool workflow_started = false;

    try {
      waitForRequiredServices();
      publishFeedback(goal_handle, "waiting_for_services", getAeAwbStatus());
      throwIfCanceled(goal_handle);

      workflow_started = true;
      setAutoMode(true);
      publishFeedback(goal_handle, "enabling_auto", getAeAwbStatus());

      const uint32_t requested_timeout = goal_handle->get_goal()->timeout_ms;
      const uint32_t timeout_ms = requested_timeout == 0 ? kDefaultTimeoutMs : requested_timeout;
      const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

      int32_t status = 0;
      do {
        throwIfCanceled(goal_handle);
        if (std::chrono::steady_clock::now() >= deadline) {
          throw std::runtime_error("timed out waiting for AE/AWB convergence");
        }
        status = getAeAwbStatus();
        publishFeedback(goal_handle, "waiting_for_convergence", status);
        if (status != kAeAwbConverged) {
          std::this_thread::sleep_for(kPollInterval);
        }
      } while (status != kAeAwbConverged);

      throwIfCanceled(goal_handle);
      publishFeedback(goal_handle, "capturing_parameters", getAeAwbStatus());
      result->captured_exposure = getIntValue(get_exposure_client_, "get_color_exposure");
      result->captured_color_gain = getIntValue(get_color_gain_client_, "get_color_gain");
      auto captured_awb_gain = getAwbGain();
      result->captured_awb_r_gain = captured_awb_gain->r_gain;
      result->captured_awb_b_gain = captured_awb_gain->b_gain;
      result->captured_awb_g_gain = captured_awb_gain->g_gain;
      result->captured_color_temperature =
          getIntValue(get_white_balance_client_, "get_white_balance");

      throwIfCanceled(goal_handle);
      publishFeedback(goal_handle, "disabling_auto", getAeAwbStatus());
      setAutoMode(false);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "applying_exposure", getAeAwbStatus());
      setIntValue(set_exposure_client_, "set_color_exposure", result->captured_exposure);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "applying_color_gain", getAeAwbStatus());
      setIntValue(set_color_gain_client_, "set_color_gain", result->captured_color_gain);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "applying_awb_gain", getAeAwbStatus());
      setAwbGain(result->captured_awb_r_gain, result->captured_awb_b_gain,
                 result->captured_awb_g_gain);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "applying_color_temperature", getAeAwbStatus());
      setIntValue(set_white_balance_client_, "set_white_balance",
                  result->captured_color_temperature);
      throwIfCanceled(goal_handle);

      publishFeedback(goal_handle, "verifying", getAeAwbStatus());
      result->actual_exposure = getIntValue(get_exposure_client_, "get_color_exposure");
      result->actual_color_gain = getIntValue(get_color_gain_client_, "get_color_gain");
      auto actual_awb_gain = getAwbGain();
      result->actual_awb_r_gain = actual_awb_gain->r_gain;
      result->actual_awb_b_gain = actual_awb_gain->b_gain;
      result->actual_awb_g_gain = actual_awb_gain->g_gain;
      result->actual_color_temperature =
          getIntValue(get_white_balance_client_, "get_white_balance");
      throwIfCanceled(goal_handle);

      if (result->captured_awb_r_gain != result->actual_awb_r_gain ||
          result->captured_awb_b_gain != result->actual_awb_b_gain ||
          result->captured_awb_g_gain != result->actual_awb_g_gain) {
        throw std::runtime_error("AWB gain readback does not match the captured value");
      }

      warnIfChanged("Exposure", result->captured_exposure, result->actual_exposure);
      warnIfChanged("Color gain", result->captured_color_gain, result->actual_color_gain);
      warnIfChanged("Color temperature", result->captured_color_temperature,
                    result->actual_color_temperature);

      result->success = true;
      result->message = "AE/AWB capture and lock-in completed";
      publishFeedback(goal_handle, "completed", getAeAwbStatus());
      goal_handle->succeed(result);
    } catch (const CanceledError&) {
      if (workflow_started) {
        restoreAutoMode();
      }
      result->success = false;
      result->message = "AE/AWB test canceled";
      goal_handle->canceled(result);
    } catch (const std::exception& e) {
      if (workflow_started) {
        restoreAutoMode();
      }
      result->success = false;
      result->message = e.what();
      RCLCPP_ERROR(get_logger(), "AE/AWB test failed: %s", e.what());
      goal_handle->abort(result);
    }
  }

  rclcpp_action::Server<RunAeAwbLockTest>::SharedPtr action_server_;

  rclcpp::Client<GetInt32>::SharedPtr get_status_client_;
  rclcpp::Client<GetAwbGain>::SharedPtr get_awb_gain_client_;
  rclcpp::Client<SetAwbGain>::SharedPtr set_awb_gain_client_;
  rclcpp::Client<GetInt32>::SharedPtr get_exposure_client_;
  rclcpp::Client<SetInt32>::SharedPtr set_exposure_client_;
  rclcpp::Client<GetInt32>::SharedPtr get_color_gain_client_;
  rclcpp::Client<SetInt32>::SharedPtr set_color_gain_client_;
  rclcpp::Client<GetInt32>::SharedPtr get_white_balance_client_;
  rclcpp::Client<SetInt32>::SharedPtr set_white_balance_client_;
  rclcpp::Client<SetBool>::SharedPtr set_auto_exposure_client_;
  rclcpp::Client<SetBool>::SharedPtr set_auto_white_balance_client_;

  std::atomic_bool goal_active_{false};
  std::atomic_bool shutting_down_{false};
  std::mutex worker_mutex_;
  std::thread worker_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AeAwbLockTestNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
