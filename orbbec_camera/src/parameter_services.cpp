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

#include "orbbec_camera/dynamic_params.h"

#include <rclcpp/parameter_service.hpp>
#include <stdexcept>

namespace orbbec_camera {
namespace {
template <typename Service, typename Callback>
rclcpp::ServiceBase::SharedPtr createParameterService(rclcpp::Node *node, const std::string &method,
                                                      Callback callback) {
  return node->create_service<Service>(
      std::string(node->get_name()) + "/" + method,
      [callback, logger = node->get_logger()](
          const std::shared_ptr<typename Service::Request> request,
          std::shared_ptr<typename Service::Response> response) {
        try {
          callback(*request, *response);
        } catch (const std::exception &e) {
          RCLCPP_WARN_STREAM(logger, "Parameter service failed: " << e.what());
        }
      },
      rmw_qos_profile_parameters);
}
}  // namespace

void Parameters::startParameterServices() {
  if (node_->get_node_options().start_parameter_services()) {
    throw std::logic_error("Disable default parameter services before installing live readback.");
  }
  if (!parameter_services_.empty()) return;
  const auto backend = node_->get_node_parameters_interface();
  parameter_services_.push_back(createParameterService<rcl_interfaces::srv::GetParameters>(
      node_, "get_parameters", [this](const auto &request, auto &response) {
        for (const auto &name : request.names) {
          try {
            response.values.push_back(getParameter(name).get_value_message());
          } catch (const std::exception &e) {
            RCLCPP_WARN_STREAM(logger_, "Could not read parameter " << name << ": " << e.what());
            // Keep one response per requested name, without substituting stale configuration.
            response.values.push_back(rclcpp::Parameter(name).get_value_message());
          } catch (...) {
            RCLCPP_WARN_STREAM(logger_, "Could not read parameter " << name << ": unknown error");
            response.values.push_back(rclcpp::Parameter(name).get_value_message());
          }
        }
      }));
  parameter_services_.push_back(createParameterService<rcl_interfaces::srv::GetParameterTypes>(
      node_, "get_parameter_types", [backend](const auto &request, auto &response) {
        response.types = backend->get_parameter_types(request.names);
      }));
  parameter_services_.push_back(createParameterService<rcl_interfaces::srv::DescribeParameters>(
      node_, "describe_parameters", [backend](const auto &request, auto &response) {
        response.descriptors = backend->describe_parameters(request.names);
      }));
  parameter_services_.push_back(createParameterService<rcl_interfaces::srv::ListParameters>(
      node_, "list_parameters", [backend](const auto &request, auto &response) {
        response.result = backend->list_parameters(request.prefixes, request.depth);
      }));
  parameter_services_.push_back(createParameterService<rcl_interfaces::srv::SetParameters>(
      node_, "set_parameters", [backend](const auto &request, auto &response) {
        for (const auto &value : request.parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          try {
            result =
                backend->set_parameters_atomically({rclcpp::Parameter::from_parameter_msg(value)});
          } catch (const std::exception &e) {
            result.successful = false;
            result.reason = e.what();
          }
          response.results.push_back(result);
        }
      }));
  parameter_services_.push_back(
      createParameterService<rcl_interfaces::srv::SetParametersAtomically>(
          node_, "set_parameters_atomically", [backend](const auto &request, auto &response) {
            try {
              std::vector<rclcpp::Parameter> values;
              for (const auto &value : request.parameters) {
                values.push_back(rclcpp::Parameter::from_parameter_msg(value));
              }
              response.result = backend->set_parameters_atomically(values);
            } catch (const std::exception &e) {
              response.result.successful = false;
              response.result.reason = e.what();
            }
          }));
}
}  // namespace orbbec_camera
