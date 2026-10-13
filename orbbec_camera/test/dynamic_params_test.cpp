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

TEST_F(DynamicParametersTest, ReadbackPreservesSkipIntentAcrossReconnect) {
  manager->syncParameters({rclcpp::Parameter("exposure", 30)});
  EXPECT_EQ(node->get_parameter("exposure").as_int(), 30);
  EXPECT_EQ(writes, 0);
  EXPECT_FALSE(manager->hasRuntimeOverride("exposure"));
  manager->clearDynamicParameters();
  manager->removeParam("exposure");
  manager->setDeviceContext("camera-A");
  const auto initial = manager->setParam("exposure", rclcpp::ParameterValue(-1), {}, descriptor);
  EXPECT_EQ(initial.get<int64_t>(), -1);
}

TEST_F(DynamicParametersTest, RestoresOnlyExplicitSuccessfulUpdatesForSameDevice) {
  ASSERT_TRUE(node->set_parameter(rclcpp::Parameter("exposure", 40)).successful);
  EXPECT_TRUE(manager->hasRuntimeOverride("exposure"));
  manager->syncParameters({rclcpp::Parameter("exposure", 41)});
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

}  // namespace
}  // namespace orbbec_camera
