#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "orbbec_camera/utils.h"

namespace orbbec_camera {
namespace {

TEST(NetDeviceParameters, AcceptsUnsetAndValidPortBoundaries) {
  EXPECT_NO_THROW(validateNetDeviceParameters("", 0));
  EXPECT_NO_THROW(validateNetDeviceParameters("192.168.1.10", 1));
  EXPECT_NO_THROW(validateNetDeviceParameters("192.168.1.10", 8090));
  EXPECT_NO_THROW(validateNetDeviceParameters("192.168.1.10", 65535));
}

TEST(NetDeviceParameters, RejectsPartialConfiguration) {
  EXPECT_THROW(validateNetDeviceParameters("192.168.1.10", 0), std::invalid_argument);
  EXPECT_THROW(validateNetDeviceParameters("", 8090), std::invalid_argument);
}

TEST(NetDeviceParameters, RejectsPortsBeforeNarrowing) {
  for (const int64_t port :
       {-1LL, 65536LL, 4294967296LL, static_cast<long long>(std::numeric_limits<int64_t>::max())}) {
    EXPECT_THROW(validateNetDeviceParameters("192.168.1.10", port), std::invalid_argument);
  }
}

}  // namespace
}  // namespace orbbec_camera
