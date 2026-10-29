#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <sstream>

#include "orbbec_camera/user_data_utils.h"

namespace orbbec_camera {
namespace {

std::vector<uint8_t> abcPacket() {
  // Network byte order length, MD5("abc"), payload. Trailing storage is permitted.
  const std::string md5 = "900150983cd24fb0d6963f7d28e17f72";
  std::vector<uint8_t> buffer{0, 0, 0, 3};
  buffer.insert(buffer.end(), md5.begin(), md5.end());
  buffer.insert(buffer.end(), {'a', 'b', 'c'});
  return buffer;
}

TEST(UserData, DecodesAndIgnoresUnusedStorage) {
  auto buffer = abcPacket();
  buffer.resize(kDeviceUserDataCapacity, 0);
  std::string data;
  ASSERT_TRUE(decodeUserData(buffer, buffer.size(), data));
  EXPECT_EQ(data, "abc");
}

TEST(UserData, RejectsTruncatedHeaderAndPayload) {
  const auto buffer = abcPacket();
  for (uint32_t size = 0; size < buffer.size(); ++size) {
    std::string data = "stale data";
    EXPECT_FALSE(decodeUserData(buffer, size, data)) << size;
    EXPECT_TRUE(data.empty());
  }
}

TEST(UserData, DecodesMaximumPayload) {
  const std::string md5 = "2db4b4ed1146cc572d9fee3227ad6f8d";
  std::vector<uint8_t> buffer{0, 0, 0x3f, 0xdc};
  buffer.insert(buffer.end(), md5.begin(), md5.end());
  buffer.resize(kDeviceUserDataCapacity, 'a');
  std::string data;
  ASSERT_TRUE(decodeUserData(buffer, buffer.size(), data));
  EXPECT_EQ(data, std::string(16348, 'a'));
}

TEST(UserData, RejectsInvalidReportedSize) {
  const auto buffer = abcPacket();
  std::string data;
  EXPECT_FALSE(decodeUserData(buffer, buffer.size() + 1, data));
  EXPECT_FALSE(decodeUserData(buffer, std::numeric_limits<uint32_t>::max(), data));
}

TEST(UserData, RejectsInvalidLengthAndDigest) {
  std::string data;
  auto buffer = abcPacket();
  std::fill_n(buffer.begin(), 4, 0xff);
  EXPECT_FALSE(decodeUserData(buffer, buffer.size(), data));
  std::fill_n(buffer.begin(), 4, 0);
  EXPECT_FALSE(decodeUserData(buffer, buffer.size(), data));
  buffer = abcPacket();
  buffer.back() = 'd';
  EXPECT_FALSE(decodeUserData(buffer, buffer.size(), data));
}

std::string calibrationData(size_t count) {
  std::ostringstream stream;
  for (size_t i = 0; i < count; ++i) stream << i << " ";
  return stream.str();
}

TEST(UserCalibration, AcceptsOnlyCompleteFinitePayload) {
  std::array<double, 29> values{};
  EXPECT_TRUE(parseUserCalibration(calibrationData(29), values));
  EXPECT_EQ(values.front(), 0);
  EXPECT_EQ(values.back(), 28);
  EXPECT_FALSE(parseUserCalibration(calibrationData(28), values));
  EXPECT_FALSE(parseUserCalibration(calibrationData(30), values));
  EXPECT_FALSE(parseUserCalibration(calibrationData(29) + "junk", values));
  EXPECT_FALSE(parseUserCalibration(calibrationData(28) + "nan", values));
  EXPECT_FALSE(parseUserCalibration(calibrationData(28) + "inf", values));
  EXPECT_FALSE(parseUserCalibration(calibrationData(28) + "1e9999", values));
}

}  // namespace
}  // namespace orbbec_camera
