/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <rclcpp/parameter_service.hpp>
#include <stdexcept>

#include "orbbec_camera/dynamic_params.h"
#include "orbbec_camera/property_range_cache.h"
#include "orbbec_camera/parameter_sensor_support.h"

namespace orbbec_camera {
namespace {

TEST(ParameterSensorSupport, DualColorExcludesDepthAndIrButRetainsSharedColorAndDeviceControls) {
  const std::set<OBSensorType> sensors = {OB_SENSOR_COLOR_LEFT, OB_SENSOR_COLOR_RIGHT};
  for (const auto &name : {"depth_exposure", "ir_gain", "depth_ae_roi_left", "ir_ae_max_exposure",
                           "interleave_frame_enable", "hdr_index0_depth_gain",
                           "laser_index1_depth_gain", "depth_precision", "enable_point_cloud",
                           "depth_registration", "enable_depth", "color_width", "enable_laser"}) {
    EXPECT_FALSE(supportsParameterSensors(name, sensors, true)) << name;
  }
  for (const auto &name :
       {"color_exposure", "enable_color_auto_exposure", "color_ae_roi_left", "left_color_mirror",
        "right_color_width", "enable_right_color", "enable_left_color_decimation_filter",
        "device_preset", "depth_work_mode", "sync_mode", "depth_delay_us", "enable_ptp_config",
        "monitor_poll_interval_sec"}) {
    EXPECT_TRUE(supportsParameterSensors(name, sensors, true)) << name;
  }
}

TEST(ParameterSensorSupport, SensorPresenceDoesNotDependOnEnabledStreams) {
  const std::set<OBSensorType> sensors = {OB_SENSOR_DEPTH, OB_SENSOR_IR_LEFT, OB_SENSOR_IR_RIGHT,
                                          OB_SENSOR_COLOR};
  for (const auto &name : {"depth_exposure", "ir_exposure", "depth_precision", "ir_ae_max_exposure",
                           "interleave_frame_enable", "enable_color", "color_gain",
                           "depth_registration", "enable_laser"}) {
    EXPECT_TRUE(supportsParameterSensors(name, sensors, true)) << name;
  }
  EXPECT_FALSE(supportsParameterSensors("enable_left_color", sensors, true));
  EXPECT_FALSE(supportsParameterSensors("enable_accel_data_correction", sensors, true));
}

TEST(ParameterSensorSupport, MissingSensorsAndUnknownCapabilitiesHaveDifferentSemantics) {
  EXPECT_FALSE(supportsParameterSensors("depth_exposure", {}, true));
  EXPECT_TRUE(supportsParameterSensors("depth_exposure", {}, false));
  EXPECT_TRUE(supportsParameterSensors("device_preset", {}, true));
  EXPECT_TRUE(supportsParameterSensors("enable_accel_data_correction", {OB_SENSOR_ACCEL}, true));
  EXPECT_TRUE(supportsParameterSensors("enable_laser", {OB_SENSOR_LIDAR}, true));
  EXPECT_TRUE(supportsParameterSensors("ir_ae_max_exposure", {OB_SENSOR_DEPTH}, true));
  EXPECT_FALSE(supportsParameterSensors("ir_gain", {OB_SENSOR_DEPTH}, true));
}

TEST(PropertyRangeCache, RepeatedAccessReusesOneSuccessfulQuery) {
  PropertyRangeCache<int> cache;
  int reads = 0;
  const auto query = [&] { return ++reads; };
  EXPECT_EQ(cache.get(OB_PROP_COLOR_EXPOSURE_INT, query), 1);
  EXPECT_EQ(cache.get(OB_PROP_COLOR_EXPOSURE_INT, query), 1);
  EXPECT_EQ(reads, 1);
}

TEST(PropertyRangeCache, StateInvalidationRefreshesTheNextAccessOnly) {
  PropertyRangeCache<int> cache;
  int reads = 0;
  const auto query = [&] { return ++reads; };
  EXPECT_EQ(cache.get(OB_PROP_COLOR_EXPOSURE_INT, query), 1);
  cache.clear();
  EXPECT_EQ(reads, 1);
  EXPECT_EQ(cache.get(OB_PROP_COLOR_EXPOSURE_INT, query), 2);
  EXPECT_EQ(cache.get(OB_PROP_COLOR_EXPOSURE_INT, query), 2);
  EXPECT_EQ(reads, 2);
}

TEST(PropertyRangeCache, DifferentPropertiesAndCameraInstancesDoNotShareEntries) {
  PropertyRangeCache<int> first;
  PropertyRangeCache<int> second;
  EXPECT_EQ(first.get(OB_PROP_COLOR_EXPOSURE_INT, [] { return 10; }), 10);
  EXPECT_EQ(first.get(OB_PROP_COLOR_GAIN_INT, [] { return 20; }), 20);
  EXPECT_EQ(second.get(OB_PROP_COLOR_EXPOSURE_INT, [] { return 30; }), 30);
}

TEST(PropertyRangeCache, FailedQueriesDoNotPoisonTheCache) {
  PropertyRangeCache<int> cache;
  EXPECT_THROW(cache.get(OB_PROP_COLOR_EXPOSURE_INT,
                         []() -> int { throw std::runtime_error("Device disconnected"); }),
               std::runtime_error);
  EXPECT_EQ(cache.get(OB_PROP_COLOR_EXPOSURE_INT, [] { return 40; }), 40);
  EXPECT_EQ(cache.get(OB_PROP_COLOR_EXPOSURE_INT, [] { return 50; }), 40);
}

class DynamicParametersTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("exposure", -1)});
    node = std::make_shared<rclcpp::Node>("dynamic_parameter_test", options);
    manager = std::make_unique<Parameters>(node.get());
    descriptor.dynamic_typing = true;
    manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor);
    auto startup_descriptor = descriptor;
    startup_descriptor.read_only = true;
    manager->setParam("width", rclcpp::ParameterValue(640), {}, startup_descriptor);
    enableUpdates();
  }

  void enableUpdates() {
    manager->registerDynamicParameters({{"exposure", rclcpp::ParameterType::PARAMETER_INTEGER}},
                                       [this](const std::vector<rclcpp::Parameter> &) {
                                         ++writes;
                                         rcl_interfaces::msg::SetParametersResult result;
                                         result.successful = accept;
                                         result.reason = accept ? "" : "simulated SDK failure";
                                         return result;
                                       });
  }

  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<Parameters> manager;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  int writes = 0;
  bool accept = true;
};

TEST_F(DynamicParametersTest, IdentifiesOnlyRegisteredDynamicParameters) {
  EXPECT_TRUE(manager->isDynamicParameter("exposure"));
  EXPECT_FALSE(manager->isDynamicParameter("width"));
  EXPECT_FALSE(manager->isDynamicParameter("missing"));
  manager->clearDynamicParameters();
  EXPECT_FALSE(manager->isDynamicParameter("exposure"));
}

TEST_F(DynamicParametersTest, RejectsWholeMixedRequestBeforeWriting) {
  const auto result = node->set_parameters_atomically(
      {rclcpp::Parameter("exposure", 40), rclcpp::Parameter("width", 1280)});
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(writes, 0);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
}

TEST_F(DynamicParametersTest, RejectsTypeChangesAndUndeclaration) {
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("exposure", true)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("exposure", 4.5)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("exposure")).successful);
  EXPECT_EQ(writes, 0);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
}

TEST_F(DynamicParametersTest, FailedDeviceWriteDoesNotChangeValueOrReconnectIntent) {
  accept = false;
  const auto result = node->set_parameter(rclcpp::Parameter("exposure", 40));
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "simulated SDK failure");
  EXPECT_EQ(writes, 1);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
}

TEST_F(DynamicParametersTest, ThrowingDeviceSetterDoesNotChangeValueOrReconnectIntent) {
  manager->registerDynamicParameters(
      {{"exposure", rclcpp::ParameterType::PARAMETER_INTEGER}},
      [this](const auto &) -> rcl_interfaces::msg::SetParametersResult {
        ++writes;
        throw std::runtime_error("simulated SDK exception");
      });
  const auto result = node->set_parameter(rclcpp::Parameter("exposure", 40));
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "simulated SDK exception");
  EXPECT_EQ(writes, 1);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
}

TEST_F(DynamicParametersTest, RejectsStartupOnlyUpdatesWithoutCallingDeviceSetter) {
  EXPECT_TRUE(node->describe_parameter("width").read_only);
  EXPECT_FALSE(node->describe_parameter("exposure").read_only);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("width", 1280)).successful);
  EXPECT_EQ(writes, 0);
  EXPECT_EQ(node->get_parameter("width").as_int(), 640);
  EXPECT_FALSE(manager->hasRuntimeOverride("width"));
}

TEST_F(DynamicParametersTest, PreservesLaunchSentinelAcrossReconnect) {
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(writes, 0);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
  manager->clearDynamicParameters();
  EXPECT_TRUE(node->has_parameter("exposure"));
  const auto initial = manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor);
  EXPECT_EQ(initial.get<int64_t>(), -1);
  enableUpdates();
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(writes, 0);
}

TEST_F(DynamicParametersTest, ReconnectUsesRetainedRosValueWithoutAnotherDeviceWrite) {
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("exposure", 40)).successful);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 40);
  EXPECT_TRUE(manager->hasRuntimeOverride("exposure"));
  accept = false;
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("exposure", 41)).successful);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 40);
  manager->clearDynamicParameters();
  EXPECT_TRUE(node->has_parameter("exposure"));
  EXPECT_EQ(
      manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor).get<int64_t>(), 40);
  EXPECT_TRUE(manager->hasRuntimeOverride("exposure"));
  EXPECT_EQ(writes, 2);
  accept = true;
  enableUpdates();
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("exposure", 42)).successful);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 42);
}

TEST_F(DynamicParametersTest, RemovedParameterDoesNotRecoverAHiddenRuntimeValue) {
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("exposure", 40)).successful);
  manager->clearDynamicParameters();
  manager->removeParam("exposure");
  EXPECT_EQ(
      manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor).get<int64_t>(), -1);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
  EXPECT_EQ(writes, 1);
}

TEST_F(DynamicParametersTest, DetachingCallbackRejectsFurtherHardwareUpdates) {
  manager->clearDynamicParameters();
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("exposure", 40)).successful);
  EXPECT_EQ(writes, 0);
}

TEST_F(DynamicParametersTest, RejectsMixingHardwareAndRosClockUpdates) {
  const auto result = node->set_parameters_atomically(
      {rclcpp::Parameter("exposure", 40), rclcpp::Parameter("use_sim_time", true)});
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(writes, 0);
  EXPECT_FALSE(node->get_parameter("use_sim_time").as_bool());
}

TEST_F(DynamicParametersTest, RetainsSoftwareConfigurationAcrossReconnect) {
  manager->setParam("queue_limit", rclcpp::ParameterValue(640));
  manager->registerDynamicParameters({{"queue_limit", rclcpp::ParameterType::PARAMETER_INTEGER}},
                                     [this](const auto &) {
                                       ++writes;
                                       rcl_interfaces::msg::SetParametersResult result;
                                       result.successful = true;
                                       return result;
                                     });
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("queue_limit", 1280)).successful);
  manager->clearDynamicParameters();
  EXPECT_EQ(manager->setParam("queue_limit", rclcpp::ParameterValue(640)).get<int64_t>(), 1280);
  EXPECT_EQ(node->get_parameter("queue_limit").as_int(), 1280);
  EXPECT_EQ(writes, 1);
  EXPECT_TRUE(manager->hasRuntimeOverride("queue_limit"));
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("queue_limit", 1920)).successful);
}

TEST_F(DynamicParametersTest, StartupReadOnlyDescriptorSurvivesReconnect) {
  manager->clearDynamicParameters();
  EXPECT_EQ(manager->setParam("width", rclcpp::ParameterValue(320)).get<int64_t>(), 640);
  enableUpdates();
  EXPECT_TRUE(node->describe_parameter("width").read_only);
  EXPECT_FALSE(node->describe_parameter("exposure").read_only);
  EXPECT_EQ(writes, 0);
}

TEST_F(DynamicParametersTest, StartupParameterIsReadOnlyAtDeclaration) {
  auto startup_descriptor = descriptor;
  startup_descriptor.read_only = true;
  manager->setParam("align_mode", rclcpp::ParameterValue("HW"), {}, startup_descriptor);
  EXPECT_TRUE(node->describe_parameter("align_mode").read_only);
  EXPECT_NO_THROW(enableUpdates());
  EXPECT_EQ(node->get_parameter("align_mode").as_string(), "HW");
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("align_mode", "SW")).successful);
  EXPECT_EQ(writes, 0);
}

TEST_F(DynamicParametersTest, CallbackParameterRemainsWritable) {
  int configured = 0;
  manager->setParam("callback_control", rclcpp::ParameterValue(1),
                    [&configured](const auto &value) { configured = value.as_int(); });
  enableUpdates();
  EXPECT_FALSE(node->describe_parameter("callback_control").read_only);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("callback_control", 2)).successful);
  EXPECT_EQ(configured, 2);
  EXPECT_EQ(writes, 0);
}

TEST_F(DynamicParametersTest, DescriptionRefreshPreservesSentinelAndDoesNotWriteDevice) {
  const auto original = node->describe_parameter("exposure");
  manager->updateDescription("exposure", "Exposure. SDK range: [1, 100].");
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(node->describe_parameter("exposure").description, "Exposure. SDK range: [1, 100].");
  EXPECT_EQ(node->describe_parameter("exposure").additional_constraints,
            original.additional_constraints);
  EXPECT_EQ(writes, 0);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("exposure", 40)).successful);
  EXPECT_EQ(writes, 1);
}

TEST_F(DynamicParametersTest, DescriptionRefreshPreservesRuntimeValueAndReconnectIntent) {
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("exposure", 40)).successful);
  manager->updateDescription("exposure", "Exposure with SDK metadata.");
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 40);
  EXPECT_EQ(writes, 1);
  EXPECT_TRUE(manager->hasRuntimeOverride("exposure"));
  manager->clearDynamicParameters();
  EXPECT_EQ(
      manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor).get<int64_t>(), 40);
  EXPECT_EQ(writes, 1);
}

TEST_F(DynamicParametersTest, FailedDescriptionRefreshRestoresOriginalDeclaration) {
  const auto original = node->describe_parameter("exposure");
  bool reject_once = true;
  auto callback = node->add_on_set_parameters_callback([&reject_once](const auto &) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = !reject_once;
    reject_once = false;
    result.reason = "simulated metadata rejection";
    return result;
  });
  EXPECT_THROW(manager->updateDescription("exposure", "New description."), std::exception);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(node->describe_parameter("exposure").description, original.description);
  EXPECT_EQ(writes, 0);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
}

TEST_F(DynamicParametersTest, ReadsDeviceEveryTimeWithoutReplacingLaunchIntent) {
  int actual = 120;
  int reads = 0;
  manager->registerParameterReader("exposure", [&] {
    ++reads;
    return rclcpp::ParameterValue(actual);
  });
  EXPECT_EQ(manager->getParameter("exposure").as_int(), 120);
  actual = 135;
  EXPECT_EQ(manager->getParameter("exposure").as_int(), 135);
  EXPECT_EQ(reads, 2);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(writes, 0);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
}

TEST_F(DynamicParametersTest, DeviceReadbackDoesNotReplaceSuccessfulReconnectIntent) {
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("exposure", 40)).successful);
  manager->registerParameterReader("exposure", [] { return rclcpp::ParameterValue(30); });
  EXPECT_EQ(manager->getParameter("exposure").as_int(), 30);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 40);
  manager->clearDynamicParameters();
  EXPECT_EQ(
      manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor).get<int64_t>(), 40);
  EXPECT_EQ(writes, 1);
}

TEST_F(DynamicParametersTest, FailedReadDoesNotSubstituteConfiguredValue) {
  manager->registerParameterReader("exposure", []() -> rclcpp::ParameterValue {
    throw std::runtime_error("simulated read failure");
  });
  EXPECT_THROW(manager->getParameter("exposure"), std::runtime_error);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(manager->getParameter("width").as_int(), 640);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
}

TEST_F(DynamicParametersTest, DetachedReaderNeverReturnsStaleValueOrCapturesOldCamera) {
  int reads = 0;
  manager->registerParameterReader("exposure", [&] {
    ++reads;
    return rclcpp::ParameterValue(200);
  });
  manager->clearDynamicParameters();
  EXPECT_THROW(manager->getParameter("exposure"), std::runtime_error);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  manager->registerParameterReader("exposure", [] { return rclcpp::ParameterValue(300); });
  EXPECT_EQ(manager->getParameter("exposure").as_int(), 300);
  EXPECT_EQ(reads, 0);
}

TEST_F(DynamicParametersTest, RejectsInvalidReadbackTypeAndPendingReader) {
  manager->registerParameterReader("exposure", [] { return rclcpp::ParameterValue(true); });
  EXPECT_THROW(manager->getParameter("exposure"), std::runtime_error);
  manager->registerParameterReader("exposure", {});
  EXPECT_THROW(manager->getParameter("exposure"), std::runtime_error);
}

TEST_F(DynamicParametersTest, LinkedReadbackUsesDeviceUnitsWithoutRepeatingHardwareWrites) {
  manager->setParam("color_exposure", rclcpp::ParameterValue(1), {}, descriptor);
  int depth_actual = 100;
  manager->registerDynamicParameters(
      {{"exposure", rclcpp::ParameterType::PARAMETER_INTEGER},
       {"color_exposure", rclcpp::ParameterType::PARAMETER_INTEGER}},
      [&](const auto &values) {
        ++writes;
        for (const auto &value : values) {
          depth_actual = value.as_int() * (value.get_name() == "color_exposure" ? 100 : 1);
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
  manager->registerParameterReader("exposure",
                                   [&] { return rclcpp::ParameterValue(depth_actual); });
  manager->registerParameterReader("color_exposure",
                                   [&] { return rclcpp::ParameterValue(depth_actual / 100); });
  manager->registerParameterAliases({{"exposure", 1}, {"color_exposure", 100}});
  auto result = manager->setParametersAtomically({rclcpp::Parameter("color_exposure", 20)});
  ASSERT_TRUE(result.successful) << result.reason;
  EXPECT_EQ(writes, 1);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 2000);
  EXPECT_EQ(node->get_parameter("color_exposure").as_int(), 20);
  EXPECT_TRUE(manager->hasRuntimeOverride("exposure"));
  result = manager->setParametersAtomically(
      {rclcpp::Parameter("exposure", 2001), rclcpp::Parameter("color_exposure", 20)});
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(writes, 1);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 2000);
  result = manager->setParametersAtomically(
      {rclcpp::Parameter("exposure", 3000), rclcpp::Parameter("color_exposure", 30)});
  EXPECT_TRUE(result.successful) << result.reason;
  EXPECT_EQ(writes, 2);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 3000);
}

TEST_F(DynamicParametersTest, SuccessfulWriteReportsReadbackFailureWithoutClaimingWriteFailed) {
  manager->registerParameterReader("exposure", []() -> rclcpp::ParameterValue {
    throw std::runtime_error("transport failed after successful write");
  });
  const auto result = manager->setParametersAtomically({rclcpp::Parameter("exposure", 40)});
  EXPECT_TRUE(result.successful);
  EXPECT_NE(result.reason.find("synchronization failed"), std::string::npos);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 40);
  EXPECT_EQ(writes, 1);
}

TEST_F(DynamicParametersTest, FailedWriteDoesNotRefreshLinkedCacheOrMarkRuntimeIntent) {
  manager->setParam("ir_exposure", rclcpp::ParameterValue(100), {}, descriptor);
  int reads = 0;
  manager->registerParameterReader("ir_exposure", [&] {
    ++reads;
    return rclcpp::ParameterValue(200);
  });
  manager->registerParameterDependencies({"exposure", "ir_exposure"});
  accept = false;
  EXPECT_FALSE(manager->setParametersAtomically({rclcpp::Parameter("exposure", 40)}).successful);
  EXPECT_EQ(reads, 0);
  EXPECT_EQ(node->get_parameter("ir_exposure").as_int(), 100);
  EXPECT_FALSE(manager->hasRuntimeOverride("ir_exposure"));
}

TEST_F(DynamicParametersTest, InitialSynchronizationDoesNotSetHardwareOrReconnectOverride) {
  manager->registerParameterReader("exposure", [] { return rclcpp::ParameterValue(123); });
  EXPECT_TRUE(manager->synchronizeParameters().empty());
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 123);
  EXPECT_EQ(writes, 0);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
}

TEST_F(DynamicParametersTest, DeviceReadbackCanReplaceStaleDescriptorBoundsWithoutHardwareWrite) {
  auto bounded = descriptor;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 0;
  range.to_value = 10;
  range.step = 1;
  bounded.integer_range.push_back(range);
  manager->setParam("gain", rclcpp::ParameterValue(5), {}, bounded);
  manager->registerParameterReader("gain", [] { return rclcpp::ParameterValue(25); });
  EXPECT_TRUE(manager->synchronizeParameters({"gain"}).empty());
  EXPECT_EQ(node->get_parameter("gain").as_int(), 25);
  EXPECT_TRUE(node->describe_parameter("gain").integer_range.empty());
  EXPECT_EQ(writes, 0);
  EXPECT_FALSE(manager->hasRuntimeOverride("gain"));
}

class LiveParameterServicesTest : public DynamicParametersTest {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node = std::make_shared<rclcpp::Node>("live_parameter_test", "/camera", options);
    manager = std::make_unique<Parameters>(node.get());
    descriptor.dynamic_typing = true;
    manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor);
    manager->setParam("width", rclcpp::ParameterValue(640));
    enableUpdates();
    manager->startParameterServices();
    client_node = std::make_shared<rclcpp::Node>("live_parameter_client");
    executor.add_node(node);
    executor.add_node(client_node);
  }

  template <typename Service>
  std::shared_ptr<typename Service::Response> call(const std::string &name,
                                                   const typename Service::Request &request) {
    auto client = client_node->create_client<Service>("/camera/live_parameter_test/" + name);
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      throw std::runtime_error("Parameter service unavailable: " + name);
    }
    auto future = client->async_send_request(std::make_shared<typename Service::Request>(request));
    if (executor.spin_until_future_complete(future, std::chrono::seconds(2)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Parameter service timed out: " + name);
    }
    return future.get();
  }

  std::shared_ptr<rclcpp::Node> client_node;
  rclcpp::executors::SingleThreadedExecutor executor;
};

TEST_F(LiveParameterServicesTest, RemoteGetReadsStoredConfigurationWithoutAccessingDevice) {
  int actual = 120;
  int reads = 0;
  bool readable = true;
  manager->registerParameterReader("exposure", [&] {
    ++reads;
    if (!readable) throw std::runtime_error("simulated disconnected camera");
    return rclcpp::ParameterValue(actual);
  });
  rcl_interfaces::srv::GetParameters::Request request;
  request.names = {"width", "exposure", "missing", "exposure"};
  auto response = call<rcl_interfaces::srv::GetParameters>("get_parameters", request);
  ASSERT_EQ(response->values.size(), 4u);
  EXPECT_EQ(response->values[0].integer_value, 640);
  EXPECT_EQ(response->values[1].integer_value, -1);
  EXPECT_EQ(response->values[2].type, rclcpp::ParameterType::PARAMETER_NOT_SET);
  EXPECT_EQ(response->values[3].integer_value, -1);
  actual = 150;
  response = call<rcl_interfaces::srv::GetParameters>("get_parameters", request);
  EXPECT_EQ(response->values[1].integer_value, -1);
  readable = false;
  response = call<rcl_interfaces::srv::GetParameters>("get_parameters", request);
  ASSERT_EQ(response->values.size(), 4u);
  EXPECT_EQ(response->values[0].integer_value, 640);
  EXPECT_EQ(response->values[1].integer_value, -1);
  EXPECT_EQ(response->values[3].integer_value, -1);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(writes, 0);
  EXPECT_EQ(reads, 0);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
}

TEST_F(LiveParameterServicesTest, DisconnectRetainsStoredConfigurationButDisablesDeviceWrites) {
  manager->registerParameterReader("exposure", [] { return rclcpp::ParameterValue(120); });
  manager->clearDynamicParameters();
  rcl_interfaces::srv::ListParameters::Request list;
  const auto listed = call<rcl_interfaces::srv::ListParameters>("list_parameters", list);
  EXPECT_NE(std::find(listed->result.names.begin(), listed->result.names.end(), "exposure"),
            listed->result.names.end());
  rcl_interfaces::srv::GetParameters::Request get;
  get.names = {"exposure"};
  const auto response = call<rcl_interfaces::srv::GetParameters>("get_parameters", get);
  ASSERT_EQ(response->values.size(), 1u);
  EXPECT_EQ(response->values[0].integer_value, -1);
  rcl_interfaces::srv::SetParametersAtomically::Request set;
  set.parameters = {rclcpp::Parameter("exposure", 40).to_parameter_msg()};
  EXPECT_FALSE(call<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically", set)
                   ->result.successful);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(writes, 0);
  enableUpdates();
  EXPECT_TRUE(call<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically", set)
                  ->result.successful);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 40);
  EXPECT_EQ(writes, 1);
}

TEST_F(LiveParameterServicesTest, StandardWriteDescribeListAndTypeServicesRemainAvailable) {
  rcl_interfaces::srv::SetParameters::Request set;
  set.parameters = {rclcpp::Parameter("exposure", 40).to_parameter_msg(),
                    rclcpp::Parameter("width", 1280).to_parameter_msg()};
  const auto response = call<rcl_interfaces::srv::SetParameters>("set_parameters", set);
  ASSERT_EQ(response->results.size(), 2u);
  EXPECT_TRUE(response->results[0].successful);
  EXPECT_FALSE(response->results[1].successful);
  EXPECT_EQ(writes, 1);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 40);
  rcl_interfaces::srv::SetParametersAtomically::Request atomic;
  atomic.parameters = set.parameters;
  EXPECT_FALSE(
      call<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically", atomic)
          ->result.successful);
  EXPECT_EQ(writes, 1);
  atomic.parameters = {rclcpp::Parameter("exposure", 41).to_parameter_msg()};
  EXPECT_TRUE(
      call<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically", atomic)
          ->result.successful);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 41);
  EXPECT_EQ(writes, 2);
  rcl_interfaces::srv::DescribeParameters::Request describe;
  describe.names = {"exposure"};
  const auto descriptions =
      call<rcl_interfaces::srv::DescribeParameters>("describe_parameters", describe);
  ASSERT_EQ(descriptions->descriptors.size(), 1u);
  EXPECT_EQ(descriptions->descriptors[0].name, "exposure");
  rcl_interfaces::srv::GetParameterTypes::Request types;
  types.names = {"exposure", "width"};
  const auto typed = call<rcl_interfaces::srv::GetParameterTypes>("get_parameter_types", types);
  ASSERT_EQ(typed->types.size(), 2u);
  EXPECT_EQ(typed->types[0], rclcpp::ParameterType::PARAMETER_INTEGER);
  rcl_interfaces::srv::ListParameters::Request list;
  const auto listed = call<rcl_interfaces::srv::ListParameters>("list_parameters", list);
  EXPECT_NE(std::find(listed->result.names.begin(), listed->result.names.end(), "exposure"),
            listed->result.names.end());
}

TEST_F(LiveParameterServicesTest, ReadOnlyParameterUsesStoredValueWhileDeviceReaderRemainsLive) {
  auto read_only = descriptor;
  read_only.read_only = true;
  manager->setParam("read_only_gain", rclcpp::ParameterValue(-1), {}, read_only);
  int actual = 12;
  manager->registerParameterReader("read_only_gain",
                                   [&actual] { return rclcpp::ParameterValue(actual); });

  rcl_interfaces::srv::GetParameters::Request get;
  get.names = {"read_only_gain"};
  auto response = call<rcl_interfaces::srv::GetParameters>("get_parameters", get);
  ASSERT_EQ(response->values.size(), 1u);
  EXPECT_EQ(response->values[0].integer_value, -1);
  actual = 15;
  response = call<rcl_interfaces::srv::GetParameters>("get_parameters", get);
  EXPECT_EQ(response->values[0].integer_value, -1);
  EXPECT_EQ(manager->getParameter("read_only_gain").as_int(), 15);
  EXPECT_EQ(node->get_parameter("read_only_gain").as_int(), -1);

  rcl_interfaces::srv::DescribeParameters::Request describe;
  describe.names = {"read_only_gain"};
  const auto described =
      call<rcl_interfaces::srv::DescribeParameters>("describe_parameters", describe);
  ASSERT_EQ(described->descriptors.size(), 1u);
  EXPECT_TRUE(described->descriptors[0].read_only);
  EXPECT_FALSE(manager->isDynamicParameter("read_only_gain"));

  rcl_interfaces::srv::SetParametersAtomically::Request set;
  set.parameters = {rclcpp::Parameter("exposure", 40).to_parameter_msg(),
                    rclcpp::Parameter("read_only_gain", 20).to_parameter_msg()};
  EXPECT_FALSE(call<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically", set)
                   ->result.successful);
  EXPECT_EQ(writes, 0);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(actual, 15);
  EXPECT_FALSE(manager->hasRuntimeOverride("read_only_gain"));

  manager->clearDynamicParameters();
  EXPECT_TRUE(node->has_parameter("read_only_gain"));
  EXPECT_THROW(manager->getParameter("read_only_gain"), std::runtime_error);
  manager->setParam("read_only_gain", rclcpp::ParameterValue(-1), {}, read_only);
  actual = 18;
  manager->registerParameterReader("read_only_gain",
                                   [&actual] { return rclcpp::ParameterValue(actual); });
  EXPECT_EQ(manager->getParameter("read_only_gain").as_int(), 18);
  EXPECT_TRUE(node->describe_parameter("read_only_gain").read_only);
}

TEST_F(LiveParameterServicesTest,
       WriteOnlyControlStoresConfigurationWithoutClaimingDeviceReadback) {
  manager->setParam("write_only_control", rclcpp::ParameterValue(-1), {}, descriptor);
  manager->registerParameterReader("write_only_control", []() -> rclcpp::ParameterValue {
    throw std::runtime_error("Camera property is not readable.");
  });
  manager->registerDynamicParameters(
      {{"write_only_control", rclcpp::ParameterType::PARAMETER_INTEGER}}, [this](const auto &) {
        ++writes;
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

  rcl_interfaces::srv::SetParametersAtomically::Request set;
  set.parameters = {rclcpp::Parameter("write_only_control", 40).to_parameter_msg()};
  EXPECT_TRUE(call<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically", set)
                  ->result.successful);
  EXPECT_EQ(writes, 1);
  EXPECT_EQ(node->get_parameter("write_only_control").as_int(), 40);
  EXPECT_THROW(manager->getParameter("write_only_control"), std::runtime_error);

  rcl_interfaces::srv::GetParameters::Request get;
  get.names = {"write_only_control"};
  const auto response = call<rcl_interfaces::srv::GetParameters>("get_parameters", get);
  ASSERT_EQ(response->values.size(), 1u);
  EXPECT_EQ(response->values[0].integer_value, 40);
}

TEST_F(LiveParameterServicesTest, UnsupportedPropertyCannotBeReadOrWritten) {
  EXPECT_FALSE(node->has_parameter("unsupported_property"));
  rcl_interfaces::srv::GetParameters::Request get;
  get.names = {"unsupported_property"};
  const auto response = call<rcl_interfaces::srv::GetParameters>("get_parameters", get);
  ASSERT_EQ(response->values.size(), 1u);
  EXPECT_EQ(response->values[0].type, rclcpp::ParameterType::PARAMETER_NOT_SET);

  rcl_interfaces::srv::SetParametersAtomically::Request set;
  set.parameters = {rclcpp::Parameter("unsupported_property", 40).to_parameter_msg()};
  EXPECT_FALSE(call<rcl_interfaces::srv::SetParametersAtomically>("set_parameters_atomically", set)
                   ->result.successful);
  EXPECT_FALSE(node->has_parameter("unsupported_property"));
  EXPECT_EQ(writes, 0);
}

TEST_F(DynamicParametersTest, RejectsInstallingDuplicateDefaultParameterServices) {
  EXPECT_THROW(manager->startParameterServices(), std::logic_error);
}

}  // namespace
}  // namespace orbbec_camera
