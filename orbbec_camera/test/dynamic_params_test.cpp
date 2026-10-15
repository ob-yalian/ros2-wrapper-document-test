#include <gtest/gtest.h>

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

TEST_F(DynamicParametersTest, ServiceAppliesOnceAndMirrorsWithoutCallingDynamicSetter) {
  int service_writes = 0;
  const auto result = manager->applyServiceParameters({rclcpp::Parameter("exposure", 200)}, [&] {
    ++service_writes;
    rcl_interfaces::msg::SetParametersResult applied;
    applied.successful = true;
    applied.reason = "legacy success";
    return applied;
  });
  EXPECT_TRUE(result.successful);
  EXPECT_EQ(result.reason, "legacy success");
  EXPECT_EQ(service_writes, 1);
  EXPECT_EQ(writes, 0);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 200);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
}

TEST_F(DynamicParametersTest, FailedServicePreservesItsErrorAndRosConfiguration) {
  int service_writes = 0;
  const auto result = manager->applyServiceParameters({rclcpp::Parameter("exposure", 200)}, [&] {
    ++service_writes;
    rcl_interfaces::msg::SetParametersResult applied;
    applied.reason = "legacy SDK failure";
    return applied;
  });
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(result.reason, "legacy SDK failure");
  EXPECT_EQ(service_writes, 1);
  EXPECT_EQ(writes, 0);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), -1);
}

TEST_F(DynamicParametersTest, ServiceKeepsItsValidationAndDoesNotEnableExternalStartupUpdates) {
  const auto result = manager->applyServiceParameters({rclcpp::Parameter("width", 1280)}, [] {
    rcl_interfaces::msg::SetParametersResult applied;
    applied.successful = true;
    return applied;
  });
  EXPECT_TRUE(result.successful);
  EXPECT_EQ(node->get_parameter("width").as_int(), 1280);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("width", 1920)).successful);
  EXPECT_FALSE(manager->hasRuntimeOverride("width"));
  manager->clearDynamicParameters();
  manager->setDeviceContext("camera-A");
  EXPECT_EQ(manager->setParam("width", rclcpp::ParameterValue(640)).get<int64_t>(), 640);
  EXPECT_EQ(node->get_parameter("width").as_int(), 640);
}

TEST_F(DynamicParametersTest, MissingParameterDoesNotPreventAnExistingServiceOperation) {
  int service_writes = 0;
  const auto result = manager->applyServiceParameters({rclcpp::Parameter("unsupported", 200)}, [&] {
    ++service_writes;
    rcl_interfaces::msg::SetParametersResult applied;
    applied.successful = true;
    return applied;
  });
  EXPECT_TRUE(result.successful);
  EXPECT_EQ(service_writes, 1);
  EXPECT_FALSE(node->has_parameter("unsupported"));
}

TEST_F(DynamicParametersTest, ServiceDoesNotReplaceTheExistingExplicitReconnectIntent) {
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("exposure", 40)).successful);
  ASSERT_TRUE(manager
                  ->applyServiceParameters({rclcpp::Parameter("exposure", 200)},
                                           [] {
                                             rcl_interfaces::msg::SetParametersResult applied;
                                             applied.successful = true;
                                             return applied;
                                           })
                  .successful);
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 200);
  manager->clearDynamicParameters();
  manager->removeParam("exposure");
  manager->setDeviceContext("camera-A");
  EXPECT_EQ(manager->setParam("exposure", rclcpp::ParameterValue(-1)).get<int64_t>(), 40);
}

TEST_F(DynamicParametersTest, RetainedCommonParametersDoNotLeakBetweenDevices) {
  manager->registerDynamicParameters({{"width", rclcpp::ParameterType::PARAMETER_INTEGER}},
                                     [](const auto &) {
                                       rcl_interfaces::msg::SetParametersResult result;
                                       result.successful = true;
                                       return result;
                                     });
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("width", 1280)).successful);
  manager->clearDynamicParameters();
  manager->setDeviceContext("camera-B");
  EXPECT_EQ(manager->setParam("width", rclcpp::ParameterValue(640)).get<int64_t>(), 640);
  EXPECT_EQ(node->get_parameter("width").as_int(), 640);
  manager->clearDynamicParameters();
  manager->setDeviceContext("camera-A");
  EXPECT_EQ(manager->setParam("width", rclcpp::ParameterValue(640)).get<int64_t>(), 1280);
}

TEST_F(DynamicParametersTest, RosSynchronizationExceptionDoesNotChangeCompletedServiceResult) {
  auto service_node = std::make_shared<rclcpp::Node>("service_parameter_test");
  // ROS invokes the newest callback first: throw after the service operation completed.
  auto rejecting_callback = service_node->add_on_set_parameters_callback([](const auto &values) {
    if (!values.empty() && values.front().as_int() == 1280) {
      throw std::runtime_error("simulated ROS callback failure");
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  });
  Parameters service_manager(service_node.get());
  service_manager.setParam("width", rclcpp::ParameterValue(640));
  int service_writes = 0;
  const auto result =
      service_manager.applyServiceParameters({rclcpp::Parameter("width", 1280)}, [&] {
        ++service_writes;
        rcl_interfaces::msg::SetParametersResult applied;
        applied.successful = true;
        applied.reason = "legacy success";
        return applied;
      });
  EXPECT_TRUE(result.successful);
  EXPECT_EQ(result.reason, "legacy success");
  EXPECT_EQ(service_writes, 1);
  EXPECT_EQ(service_node->get_parameter("width").as_int(), 640);
}

}  // namespace
}  // namespace orbbec_camera
