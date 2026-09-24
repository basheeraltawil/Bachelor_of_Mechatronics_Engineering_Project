#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "robocraft_hardware/robocraft_i2c_system.hpp"

using namespace robocraft_hardware::protocol;

TEST(Protocol, Crc8MatchesSmbusCheckValue)
{
  const char * s = "123456789";
  EXPECT_EQ(crc8(reinterpret_cast<const uint8_t *>(s), 9), 0xF4);
}

TEST(Protocol, CommandLayout)
{
  auto f = encode_command(7, 12.34 * M_PI / 180.0, -56.78 * M_PI / 180.0, 90, true, true, false);
  EXPECT_EQ(f[0], kCmdHeader);
  EXPECT_EQ(f[1], 7);
  EXPECT_EQ(static_cast<int16_t>((f[2] << 8) | f[3]), 1234);
  EXPECT_EQ(static_cast<int16_t>((f[4] << 8) | f[5]), -5678);
  EXPECT_EQ(f[6], 90);
  EXPECT_EQ(f[7], 0x03);
  EXPECT_EQ(f[8], crc8(f.data(), 8));
}

TEST(Protocol, StateRejectsCorruption)
{
  uint8_t f[kStateLen] = {kStateHeader, 3, 0x04, 0xD2, 0xFB, 0x2E, 0x09, 0};
  f[7] = crc8(f, 7);
  uint8_t seq, st;
  double q1, q2;
  ASSERT_TRUE(decode_state(f, seq, q1, q2, st));
  EXPECT_NEAR(q1 * 180.0 / M_PI, 12.34, 1e-9);
  EXPECT_NEAR(q2 * 180.0 / M_PI, -12.34, 1e-9);
  f[3] ^= 0x10;
  EXPECT_FALSE(decode_state(f, seq, q1, q2, st));
}
