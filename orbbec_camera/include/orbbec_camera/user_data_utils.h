#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace orbbec_camera {

constexpr uint32_t kDeviceUserDataCapacity = 16384;
constexpr std::size_t kUserDataHeaderSize = sizeof(uint32_t) + 32;

bool decodeUserData(const std::vector<uint8_t>& buffer, uint32_t size, std::string& data);
bool parseUserCalibration(const std::string& data, std::array<double, 29>& values);

}  // namespace orbbec_camera
