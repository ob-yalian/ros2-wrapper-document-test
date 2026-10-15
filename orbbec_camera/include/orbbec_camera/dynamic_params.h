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

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <set>

#include "ros_param_backend.h"

namespace orbbec_camera {
class Parameters {
 public:
  explicit Parameters(rclcpp::Node* node);
  ~Parameters() noexcept;
  rclcpp::ParameterValue setParam(const std::string& param_name,
                                  rclcpp::ParameterValue initial_value,
                                  const std::function<void(const rclcpp::Parameter&)>& func =
                                      std::function<void(const rclcpp::Parameter&)>(),
                                  const rcl_interfaces::msg::ParameterDescriptor& descriptor =
                                      rcl_interfaces::msg::ParameterDescriptor());

  template <class T>
  void setParamT(std::string param_name, rclcpp::ParameterValue initial_value, T& param,
                 std::function<void(const rclcpp::Parameter&)> func =
                     std::function<void(const rclcpp::Parameter&)>(),
                 rcl_interfaces::msg::ParameterDescriptor descriptor =
                     rcl_interfaces::msg::ParameterDescriptor());
  template <class T>
  void setParamValue(T& param, const T& value);  // function updates the parameter value both
                                                 // locally and in the parameters server
  void removeParam(const std::string& param_name);

  using BatchSetter = std::function<rcl_interfaces::msg::SetParametersResult(
      const std::vector<rclcpp::Parameter>&)>;
  // Run an existing service operation once and mirror its successful configuration.
  // The service keeps its own validation, device writes and response semantics.
  rcl_interfaces::msg::SetParametersResult applyServiceParameters(
      const std::vector<rclcpp::Parameter>& values,
      const std::function<rcl_interfaces::msg::SetParametersResult()>& apply);
  void setDeviceContext(const std::string& device_key);
  void registerDynamicParameters(const std::map<std::string, rclcpp::ParameterType>& types,
                                 BatchSetter setter);
  // Drains any in-flight callback before releasing its camera captures.
  void clearDynamicParameters();
  bool hasRuntimeOverride(const std::string& name) const;
  void rememberRuntimeConfiguration(const std::vector<rclcpp::Parameter>& values);

 private:
  mutable std::recursive_mutex callback_mutex_;
  std::map<std::string, rclcpp::ParameterType> dynamic_types_;
  BatchSetter batch_setter_;
  std::string device_key_;
  std::map<std::string, std::map<std::string, rclcpp::ParameterValue>> runtime_overrides_;
  std::map<std::string, rclcpp::ParameterValue> initial_values_;
  std::set<std::string> service_updates_;
  std::set<std::string> runtime_parameter_names_;
  rclcpp::Node* node_;
  rclcpp::Logger logger_;
  std::map<std::string, std::vector<std::function<void(const rclcpp::Parameter&)> > >
      param_functions_;
  std::map<void*, std::string> param_names_;
  ParametersBackend params_backend_;
};
}  // namespace orbbec_camera
