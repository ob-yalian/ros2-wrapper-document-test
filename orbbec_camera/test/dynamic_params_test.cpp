#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <rclcpp/parameter_service.hpp>
#include <stdexcept>

#include "orbbec_camera/dynamic_params.h"

namespace orbbec_camera {
namespace {

class DynamicParametersTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("exposure", -1)});
    node = std::make_shared<rclcpp::Node>("dynamic_parameter_test", options);
    manager = std::make_unique<Parameters>(node.get());
    manager->setDeviceContext("camera-A");
    descriptor.dynamic_typing = true;
    manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor);
    manager->setParam("width", rclcpp::ParameterValue(640));
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
  manager->removeParam("exposure");
  manager->setDeviceContext("camera-A");
  const auto initial = manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor);
  EXPECT_EQ(initial.get<int64_t>(), -1);
  enableUpdates();
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(writes, 0);
}

TEST_F(DynamicParametersTest, RestoresOnlyExplicitSuccessfulUpdatesForSameDevice) {
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("exposure", 40)).successful);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 40);
  EXPECT_TRUE(manager->hasRuntimeOverride("exposure"));
  accept = false;
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("exposure", 41)).successful);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 40);
  manager->clearDynamicParameters();
  manager->removeParam("exposure");
  manager->setDeviceContext("camera-A");
  EXPECT_EQ(
      manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor).get<int64_t>(), 40);
  manager->removeParam("exposure");
  manager->setDeviceContext("camera-B");
  EXPECT_EQ(
      manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor).get<int64_t>(), -1);
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

TEST_F(DynamicParametersTest, RetainedCommonParametersDoNotLeakBetweenDevices) {
  manager->registerDynamicParameters({{"width", rclcpp::ParameterType::PARAMETER_INTEGER}},
                                     [this](const auto &) {
                                       ++writes;
                                       rcl_interfaces::msg::SetParametersResult result;
                                       result.successful = true;
                                       return result;
                                     });
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("width", 1280)).successful);
  manager->clearDynamicParameters();
  manager->setDeviceContext("camera-B");
  EXPECT_EQ(manager->setParam("width", rclcpp::ParameterValue(640)).get<int64_t>(), 640);
  EXPECT_EQ(node->get_parameter("width").as_int(), 640);
  EXPECT_EQ(writes, 1);
  EXPECT_FALSE(manager->hasRuntimeOverride("width"));
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("width", 1920)).successful);
  manager->clearDynamicParameters();
  manager->setDeviceContext("camera-A");
  EXPECT_EQ(manager->setParam("width", rclcpp::ParameterValue(640)).get<int64_t>(), 1280);
  EXPECT_EQ(node->get_parameter("width").as_int(), 1280);
  EXPECT_EQ(writes, 1);
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
  manager->removeParam("exposure");
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
  manager->removeParam("exposure");
  manager->setDeviceContext("camera-A");
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
  manager->setDeviceContext("camera-B");
  EXPECT_THROW(manager->getParameter("exposure"), std::runtime_error);
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

class LiveParameterServicesTest : public DynamicParametersTest {
 protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.start_parameter_services(false);
    node = std::make_shared<rclcpp::Node>("live_parameter_test", "/camera", options);
    manager = std::make_unique<Parameters>(node.get());
    manager->setDeviceContext("camera-A");
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

TEST_F(LiveParameterServicesTest, RemoteGetReadsDeviceAndPreservesBatchOrderOnFailure) {
  int actual = 120;
  bool readable = true;
  manager->registerParameterReader("exposure", [&] {
    if (!readable) throw std::runtime_error("simulated disconnected camera");
    return rclcpp::ParameterValue(actual);
  });
  rcl_interfaces::srv::GetParameters::Request request;
  request.names = {"width", "exposure", "missing", "exposure"};
  auto response = call<rcl_interfaces::srv::GetParameters>("get_parameters", request);
  ASSERT_EQ(response->values.size(), 4u);
  EXPECT_EQ(response->values[0].integer_value, 640);
  EXPECT_EQ(response->values[1].integer_value, 120);
  EXPECT_EQ(response->values[2].type, rclcpp::ParameterType::PARAMETER_NOT_SET);
  EXPECT_EQ(response->values[3].integer_value, 120);
  actual = 150;
  response = call<rcl_interfaces::srv::GetParameters>("get_parameters", request);
  EXPECT_EQ(response->values[1].integer_value, 150);
  readable = false;
  response = call<rcl_interfaces::srv::GetParameters>("get_parameters", request);
  ASSERT_EQ(response->values.size(), 4u);
  EXPECT_EQ(response->values[0].integer_value, 640);
  EXPECT_EQ(response->values[1].type, rclcpp::ParameterType::PARAMETER_NOT_SET);
  EXPECT_EQ(response->values[3].type, rclcpp::ParameterType::PARAMETER_NOT_SET);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
  EXPECT_EQ(writes, 0);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
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

TEST_F(DynamicParametersTest, RejectsInstallingDuplicateDefaultParameterServices) {
  EXPECT_THROW(manager->startParameterServices(), std::logic_error);
}

}  // namespace
}  // namespace orbbec_camera
