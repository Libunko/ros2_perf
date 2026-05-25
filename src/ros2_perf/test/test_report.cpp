// Copyright 2026 The ros2_perf Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cstring>

#include "ros2_perf/report.hpp"

TEST(Report, LossRatePctEmpty) { EXPECT_DOUBLE_EQ(ros2_perf::loss_rate_pct(0, 0), 0.0); }

TEST(Report, LossRatePctHalf) { EXPECT_DOUBLE_EQ(ros2_perf::loss_rate_pct(1, 1), 50.0); }

TEST(Report, FmtUsBufSmall)
{
  char buf[32];
  ros2_perf::fmt_us_buf(500000, buf, sizeof(buf));
  EXPECT_STREQ(buf, "500.0");
}

TEST(Report, FmtMbpsBuf)
{
  char buf[32];
  ros2_perf::fmt_mbps_buf(125000.0, buf, sizeof(buf));
  EXPECT_STREQ(buf, "1.0");
}
