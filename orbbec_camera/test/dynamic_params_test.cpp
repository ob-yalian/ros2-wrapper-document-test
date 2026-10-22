#include <gtest/gtest.h>

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

}  // namespace
}  // namespace orbbec_camera
