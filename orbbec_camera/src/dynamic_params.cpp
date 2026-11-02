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

#include <cmath>
#include <limits>
#include <stdexcept>

namespace orbbec_camera {
namespace {
bool integerRangeContains(const rclcpp::ParameterValue &value,
                          const rcl_interfaces::msg::IntegerRange &range) {
  if (value.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) return false;
  const auto current = value.get<int64_t>();
  if (current < range.from_value || current > range.to_value) return false;
  if (range.step == 0) return true;
  if (range.step > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) return false;
  return (current - range.from_value) % static_cast<int64_t>(range.step) == 0;
}

bool floatingPointRangeContains(const rclcpp::ParameterValue &value,
                                const rcl_interfaces::msg::FloatingPointRange &range) {
  if (value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) return false;
  const auto current = value.get<double>();
  if (!std::isfinite(current) || current < range.from_value || current > range.to_value) {
    return false;
  }
  if (range.step <= 0.0) return true;
  const auto steps = (current - range.from_value) / range.step;
  return std::abs(steps - std::round(steps)) <= 1e-9;
}

void removeInvalidDescriptorRanges(rcl_interfaces::msg::ParameterDescriptor &descriptor,
                                   const rclcpp::ParameterValue &value) {
  if (!descriptor.integer_range.empty() &&
      !integerRangeContains(value, descriptor.integer_range.front())) {
    descriptor.integer_range.clear();
  }
  if (!descriptor.floating_point_range.empty() &&
      !floatingPointRangeContains(value, descriptor.floating_point_range.front())) {
    descriptor.floating_point_range.clear();
  }
}

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
  params_backend_.addOnSetParametersCallback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        // Updating a descriptor re-declares the same value without applying it to the device.
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
            if (result.successful) {
              for (const auto &parameter : parameters) {
                runtime_updated_parameters_.insert(parameter.get_name());
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
  parameter_services_.clear();
  for (auto const &param : param_functions_) {
    try {
      node_->undeclare_parameter(param.first);
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &e) {
      // ignore
    } catch (const rclcpp::exceptions::ParameterImmutableException &e) {
      // Read-only declarations are released with the driver node.
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(logger_, e.what());
    }
  }
}

rclcpp::ParameterValue Parameters::setParam(
    const std::string &param_name, rclcpp::ParameterValue initial_value,
    const std::function<void(const rclcpp::Parameter &)> &func,
    const rcl_interfaces::msg::ParameterDescriptor &descriptor, bool ignore_override) {
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
      auto descriptor_value = initial_value;
      if (!ignore_override) {
        const auto overrides = node_->get_node_parameters_interface()->get_parameter_overrides();
        const auto override = overrides.find(param_name);
        if (override != overrides.end() &&
            override->second.get_type() == initial_value.get_type()) {
          descriptor_value = override->second;
        }
      }
      removeInvalidDescriptorRanges(description, descriptor_value);
      result_value =
          node_->declare_parameter(param_name, initial_value, description, ignore_override);
    } else {
      // The driver node survives camera reconnects; retain its configured parameter value.
      result_value = node_->get_parameter(param_name).get_parameter_value();
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
  runtime_updated_parameters_.erase(param_name);
}

void Parameters::updateDescription(const std::string &param_name, const std::string &description) {
  if (!node_->has_parameter(param_name)) return;
  auto descriptor = node_->describe_parameter(param_name);
  descriptor.description = description;
  updateDescription(param_name, descriptor);
}

void Parameters::updateDescription(
    const std::string &param_name,
    const rcl_interfaces::msg::ParameterDescriptor &requested_descriptor) {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  if (!node_->has_parameter(param_name)) return;
  const auto previous = node_->describe_parameter(param_name);
  if (!previous.dynamic_typing || previous.read_only) return;
  const auto value = node_->get_parameter(param_name);
  auto requested = requested_descriptor;
  removeInvalidDescriptorRanges(requested, value.get_parameter_value());
  auto descriptor = previous;
  descriptor.description = requested.description;
  if (!requested.integer_range.empty()) {
    descriptor.integer_range = requested.integer_range;
  }
  if (!requested.floating_point_range.empty()) {
    descriptor.floating_point_range = requested.floating_point_range;
  }
  if (descriptor.description == previous.description &&
      descriptor.integer_range == previous.integer_range &&
      descriptor.floating_point_range == previous.floating_point_range) {
    return;
  }
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

void Parameters::updateParameterValue(const std::string &param_name,
                                      const rclcpp::ParameterValue &value) {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  if (!node_->has_parameter(param_name)) return;
  const auto current = node_->get_parameter(param_name);
  if (current.get_type() != value.get_type()) {
    throw std::runtime_error("Invalid value type for " + param_name);
  }
  if (current.get_parameter_value() == value) return;
  const auto descriptor = node_->describe_parameter(param_name);
  if (descriptor.read_only) return;
  const rclcpp::Parameter replacement(param_name, value);
  ParameterRestore restore(this, replacement);
  node_->undeclare_parameter(param_name);
  try {
    node_->declare_parameter(param_name, value, descriptor, true);
  } catch (...) {
    if (!node_->has_parameter(param_name)) {
      node_->declare_parameter(param_name, current.get_parameter_value(), descriptor, true);
    }
    throw;
  }
}

void Parameters::registerDynamicParameters(
    const std::map<std::string, rclcpp::ParameterType> &types, BatchSetter setter) {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  dynamic_types_ = types;
  batch_setter_ = std::move(setter);
}

void Parameters::clearDynamicParameters() {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  dynamic_types_.clear();
  batch_setter_ = {};
  parameter_readers_.clear();
}

void Parameters::registerParameterReader(const std::string &name, ParameterReader reader) {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  device_readback_names_.insert(name);
  parameter_readers_[name] = std::move(reader);
}

rclcpp::Parameter Parameters::getParameter(const std::string &name) const {
  // Do not hold callback_mutex_ while acquiring rclcpp's parameter mutex: setters
  // acquire these locks in the opposite order before entering our callback.
  const auto configured = node_->get_parameter(name);
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  const auto reader = parameter_readers_.find(name);
  if (reader == parameter_readers_.end() || !reader->second) {
    if (device_readback_names_.count(name) != 0) {
      throw std::runtime_error("Camera is initializing or disconnected: " + name);
    }
    return configured;
  }
  const auto value = reader->second();
  if (value.get_type() != configured.get_type()) {
    throw std::runtime_error("Invalid readback type for " + name);
  }
  return rclcpp::Parameter(name, value);
}

bool Parameters::isDynamicParameter(const std::string &name) const {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  return dynamic_types_.count(name) != 0;
}

bool Parameters::hasRuntimeOverride(const std::string &name) const {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  return runtime_updated_parameters_.count(name) != 0;
}

void Parameters::markRuntimeParameters(const std::vector<rclcpp::Parameter> &values) {
  std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
  for (const auto &value : values) {
    runtime_updated_parameters_.insert(value.get_name());
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
