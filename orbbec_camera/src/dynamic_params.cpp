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
#include "orbbec_camera/parameter_descriptions.h"

namespace orbbec_camera {
namespace {
struct ParameterRestore;
thread_local ParameterRestore *parameter_restore = nullptr;

struct ParameterRestore {
  const Parameters *manager;
  const rclcpp::Parameter &value;
  ParameterRestore *previous;

  ParameterRestore(const Parameters *owner, const rclcpp::Parameter &parameter)
      : manager(owner), value(parameter), previous(parameter_restore) {
    parameter_restore = this;
  }
  ~ParameterRestore() { parameter_restore = previous; }
};
}  // namespace

Parameters::Parameters(rclcpp::Node *node)
    : node_(node), logger_(node_->get_logger()), params_backend_(node) {
  params_backend_.addOnSetParametersCallback([this](
                                                 const std::vector<rclcpp::Parameter> &parameters) {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // Reconnect restores retained ROS values before the camera applies its initial configuration.
    if (parameter_restore && parameter_restore->manager == this && parameters.size() == 1 &&
        parameter_restore->value == parameters.front()) {
      return result;
    }
    bool has_dynamic = false;
    // Reject the entire request before invoking any callback with side effects.
    for (const auto &parameter : parameters) {
      const auto dynamic = dynamic_types_.find(parameter.get_name());
      if (dynamic != dynamic_types_.end()) {
        has_dynamic = true;
        if (parameter.get_type() != dynamic->second) {
          result.successful = false;
          result.reason = "Invalid type for " + parameter.get_name();
          return result;
        }
        continue;
      }
      const auto function_it = param_functions_.find(parameter.get_name());
      if (function_it != param_functions_.end() && function_it->second.empty()) {
        result.successful = false;
        result.reason = "Parameter " + parameter.get_name() + " can not be changed in runtime.";
        return result;
      }
    }
    try {
      if (has_dynamic) {
        for (const auto &parameter : parameters) {
          if (dynamic_types_.count(parameter.get_name()) == 0) {
            result.successful = false;
            result.reason = "Hardware updates cannot be mixed with other parameters.";
            return result;
          }
        }
        result = batch_setter_(parameters);
        if (result.successful && !device_key_.empty()) {
          for (const auto &parameter : parameters) {
            runtime_overrides_[device_key_][parameter.get_name()] = parameter.get_parameter_value();
          }
        }
      } else {
        for (const auto &parameter : parameters) {
          const auto it = param_functions_.find(parameter.get_name());
          if (it != param_functions_.end()) {
            for (const auto &func : it->second) {
              func(parameter);
            }
          }
        }
      }
    } catch (const std::exception &e) {
      result.successful = false;
      result.reason = e.what();
    }
    return result;
  });
}

Parameters::~Parameters() noexcept {
  for (auto const &param : param_functions_) {
    try {
      node_->undeclare_parameter(param.first);
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
      // ignore
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, e.what());
    }
  }
}

rclcpp::ParameterValue Parameters::setParam(
    const std::string &param_name, rclcpp::ParameterValue initial_value,
    const std::function<void(const rclcpp::Parameter &)> &func,
    const rcl_interfaces::msg::ParameterDescriptor &descriptor) {
  bool restore_runtime = false;
  bool restore_initial = false;
  {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    const auto device = runtime_overrides_.find(device_key_);
    if (!device_key_.empty() && device != runtime_overrides_.end()) {
      const auto value = device->second.find(param_name);
      if (value != device->second.end()) {
        initial_value = value->second;
        restore_runtime = true;
      }
    }
    if (!restore_runtime && runtime_parameter_names_.count(param_name) != 0) {
      const auto original = initial_values_.find(param_name);
      if (original != initial_values_.end()) {
        initial_value = original->second;
        restore_initial = true;
      }
    }
  }
  rclcpp::ParameterValue result_value(initial_value);
  try {
    if (!node_->has_parameter(param_name)) {
      auto description = descriptor;
      const auto defaults = describeCameraParameter(param_name);
      if (description.description.empty()) {
        description.description = defaults.description;
      }
      if (description.additional_constraints.empty()) {
        description.additional_constraints = defaults.additional_constraints;
      }
      result_value =
          node_->declare_parameter(param_name, initial_value, description, restore_runtime);
    } else {
      result_value = node_->get_parameter(param_name).get_parameter_value();
      if ((restore_initial || restore_runtime) && result_value != initial_value) {
        const rclcpp::Parameter value(param_name, initial_value);
        ParameterRestore restore(this, value);
        const auto restored = node_->set_parameter(value);
        if (restored.successful) {
          result_value = initial_value;
        } else {
          RCLCPP_WARN_STREAM(
              logger_, "Could not restore parameter " << param_name << ": " << restored.reason);
        }
      }
    }
  } catch (const std::exception &e) {
    std::stringstream range;
    for (auto val : descriptor.floating_point_range) {
      range << val.from_value << ", " << val.to_value;
    }
    for (auto val : descriptor.integer_range) {
      range << val.from_value << ", " << val.to_value;
    }
    RCLCPP_WARN_STREAM(
        logger_,
        "Could not set param: " << param_name << " with "
                                << rclcpp::Parameter(param_name, initial_value).value_to_string()
                                << "Range: [" << range.str() << "]"
                                << ": " << e.what());
    return initial_value;
  }

  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  initial_values_.emplace(param_name, result_value);
  if (func) {
    param_functions_[param_name].push_back(func);
  } else {
    param_functions_[param_name] = std::vector<std::function<void(const rclcpp::Parameter &)>>();
  }
  if (result_value != initial_value && func) {
    func(rclcpp::Parameter(param_name, result_value));
  }
  return result_value;
}

template <class T>
void Parameters::setParamT(std::string param_name, rclcpp::ParameterValue initial_value, T &param,
                           std::function<void(const rclcpp::Parameter &)> func,
                           rcl_interfaces::msg::ParameterDescriptor descriptor) {
  // NOTICE: callback function is set AFTER the parameter is declared!!!
  if (!node_->has_parameter(param_name))
    param = node_->declare_parameter(param_name, initial_value, descriptor).get<T>();
  else {
    param = node_->get_parameter(param_name).get_parameter_value().get<T>();
  }
  param_functions_[param_name].push_back(
      [&param, func](const rclcpp::Parameter &parameter) { param = parameter.get_value<T>(); });
  if (func) {
    param_functions_[param_name].push_back(func);
  }
  param_names_[&param] = param_name;
}

template <class T>
void Parameters::setParamValue(T &param, const T &value) {
  param = value;
  try {
    std::string param_name = param_names_.at(&param);

    rcl_interfaces::msg::SetParametersResult results =
        node_->set_parameter(rclcpp::Parameter(param_name, value));
    if (!results.successful) {
      RCLCPP_WARN_STREAM(logger_, "Parameter: " << param_name << " was not set:" << results.reason);
    }
  } catch (const std::out_of_range &e) {
    RCLCPP_WARN_STREAM(logger_, "Parameter was not internally declared.");
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e) {
    std::string param_name = param_names_.at(&param);
    RCLCPP_WARN_STREAM(logger_, "Parameter: " << param_name << " was not declared:" << e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, e.what());
  }
}

void Parameters::removeParam(const std::string &param_name) {
  node_->undeclare_parameter(param_name);
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  param_functions_.erase(param_name);
  dynamic_types_.erase(param_name);
}

void Parameters::updateDescription(const std::string &param_name, const std::string &description) {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  if (!node_->has_parameter(param_name)) return;
  const auto previous = node_->describe_parameter(param_name);
  if (!previous.dynamic_typing || previous.read_only || previous.description == description) return;
  const auto value = node_->get_parameter(param_name);
  auto descriptor = previous;
  descriptor.description = description;
  // Humble has no descriptor-update API. Re-declare only device parameters during
  // initialization, preserving the value, callbacks and reconnect intent.
  ParameterRestore restore(this, value);
  node_->undeclare_parameter(param_name);
  try {
    node_->declare_parameter(param_name, value.get_parameter_value(), descriptor, true);
  } catch (...) {
    if (!node_->has_parameter(param_name)) {
      node_->declare_parameter(param_name, value.get_parameter_value(), previous, true);
    }
    throw;
  }
}

void Parameters::setDeviceContext(const std::string &device_key) {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  dynamic_types_.clear();
  batch_setter_ = {};
  device_key_ = device_key;
}

void Parameters::registerDynamicParameters(
    const std::map<std::string, rclcpp::ParameterType> &types, BatchSetter setter) {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  dynamic_types_ = types;
  for (const auto &entry : types) {
    runtime_parameter_names_.insert(entry.first);
  }
  batch_setter_ = std::move(setter);
}

void Parameters::clearDynamicParameters() {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  dynamic_types_.clear();
  batch_setter_ = {};
}

bool Parameters::hasRuntimeOverride(const std::string &name) const {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  const auto device = runtime_overrides_.find(device_key_);
  return !device_key_.empty() && device != runtime_overrides_.end() &&
         device->second.count(name) != 0;
}

void Parameters::rememberRuntimeConfiguration(const std::vector<rclcpp::Parameter> &values) {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  if (!device_key_.empty()) {
    for (const auto &value : values) {
      runtime_overrides_[device_key_][value.get_name()] = value.get_parameter_value();
    }
  }
}

template void Parameters::setParamT<bool>(std::string param_name,
                                          rclcpp::ParameterValue initial_value, bool &param,
                                          std::function<void(const rclcpp::Parameter &)> func,
                                          rcl_interfaces::msg::ParameterDescriptor descriptor);

template void Parameters::setParamT<int>(std::string param_name,
                                         rclcpp::ParameterValue initial_value, int &param,
                                         std::function<void(const rclcpp::Parameter &)> func,
                                         rcl_interfaces::msg::ParameterDescriptor descriptor);

template void Parameters::setParamT<double>(std::string param_name,
                                            rclcpp::ParameterValue initial_value, double &param,
                                            std::function<void(const rclcpp::Parameter &)> func,
                                            rcl_interfaces::msg::ParameterDescriptor descriptor);

template void Parameters::setParamValue<int>(int &param, const int &value);

template void Parameters::setParamValue<bool>(bool &param, const bool &value);

template void Parameters::setParamValue<double>(double &param, const double &value);
}  // namespace orbbec_camera
