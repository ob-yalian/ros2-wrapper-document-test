/*******************************************************************************
 * Copyright (c) 2026 Orbbec 3D Technology, Inc
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

#include "orbbec_camera/gige_action_command_node.h"

#include <exception>
#include <functional>
#include <sstream>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"

namespace orbbec_camera {
namespace {

std::string formatObError(const ob::Error& error) {
  std::ostringstream stream;
  stream << (error.getMessage() ? error.getMessage() : "Unknown OB error")
         << " status:" << static_cast<int>(error.getStatus());
  return stream.str();
}

}  // namespace

GigEActionCommandNode::GigEActionCommandNode(const rclcpp::NodeOptions& node_options)
    : Node("gige_action_command_node", node_options), context_(std::make_unique<ob::Context>()) {
  context_->enableNetDeviceEnumeration(true);
  send_action_command_service_ = create_service<orbbec_camera_msgs::srv::SendActionCommand>(
      "~/send_action_command", std::bind(&GigEActionCommandNode::sendActionCommandCallback, this,
                                         std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(get_logger(), "GigE Action Command service is ready");
}

void GigEActionCommandNode::sendActionCommandCallback(
    const std::shared_ptr<orbbec_camera_msgs::srv::SendActionCommand::Request> request,
    std::shared_ptr<orbbec_camera_msgs::srv::SendActionCommand::Response> response) {
  if (!request) {
    response->success = false;
    response->message = "Invalid request";
    return;
  }

  const std::string destination_ip =
      request->destination_ip.empty() ? "255.255.255.255" : request->destination_ip;
  try {
    response->success =
        context_->sendActionCommand(request->device_key, request->group_key, request->group_mask,
                                    destination_ip.c_str(), request->scheduled_time);
    response->message =
        response->success ? "Action Command dispatched" : "SDK failed to send Action Command";
  } catch (const ob::Error& error) {
    response->success = false;
    response->message = formatObError(error);
  } catch (const std::exception& error) {
    response->success = false;
    response->message = error.what();
  } catch (...) {
    response->success = false;
    response->message = "Unknown error";
  }
}

}  // namespace orbbec_camera

RCLCPP_COMPONENTS_REGISTER_NODE(orbbec_camera::GigEActionCommandNode)
