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

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <random>
#include <vector>

#include "ros2_perf/stats.hpp"

namespace
{

void add_all(ros2_perf::LatencyStat & ls, const std::vector<int64_t> & v)
{
  for (auto x : v) {
    ls.add(x);
  }
}

}  // namespace

TEST(LatencyStat, EmptySummary)
{
  ros2_perf::LatencyStat ls;
  auto s = ls.summary_total();
  EXPECT_EQ(s.count, 0u);
  EXPECT_EQ(s.min_ns, 0);
  EXPECT_EQ(s.max_ns, 0);
  EXPECT_DOUBLE_EQ(s.mean_ns, 0.0);
}

TEST(LatencyStat, SinglePoint)
{
  ros2_perf::LatencyStat ls;
  ls.add(1234);
  auto s = ls.summary_total();
  EXPECT_EQ(s.count, 1u);
  EXPECT_EQ(s.min_ns, 1234);
  EXPECT_EQ(s.max_ns, 1234);
  EXPECT_DOUBLE_EQ(s.mean_ns, 1234.0);
  EXPECT_DOUBLE_EQ(s.stddev_ns, 0.0);
  EXPECT_EQ(s.p50_ns, 1234);
  EXPECT_EQ(s.p99_ns, 1234);
}

TEST(LatencyStat, KnownSamples)
{
  // 1..100. Mean=50.5, min=1, max=100. Nearest-rank percentile of N=100:
  //   p50 = idx round(0.50 * 99) = 50 -> value 51
  //   p90 = idx round(0.90 * 99) = 89 -> value 90
  //   p99 = idx round(0.99 * 99) = 98 -> value 99
  //   p95 = idx round(0.95 * 99) = 94 -> value 95
  ros2_perf::LatencyStat ls;
  std::vector<int64_t> v(100);
  std::iota(v.begin(), v.end(), 1);
  add_all(ls, v);
  auto s = ls.summary_total();
  EXPECT_EQ(s.count, 100u);
  EXPECT_EQ(s.min_ns, 1);
  EXPECT_EQ(s.max_ns, 100);
  EXPECT_NEAR(s.mean_ns, 50.5, 1e-9);
  EXPECT_EQ(s.p50_ns, 51);
  EXPECT_EQ(s.p90_ns, 90);
  EXPECT_EQ(s.p95_ns, 95);
  EXPECT_EQ(s.p99_ns, 99);
}

TEST(LatencyStat, MonotonicPercentiles)
{
  ros2_perf::LatencyStat ls;
  std::mt19937_64 rng(0xCAFEBABEull);
  std::uniform_int_distribution<int64_t> dist(0, 100000);
  for (int i = 0; i < 10000; ++i) {
    ls.add(dist(rng));
  }
  auto s = ls.summary_total();
  EXPECT_EQ(s.count, 10000u);
  EXPECT_LE(s.min_ns, s.p50_ns);
  EXPECT_LE(s.p50_ns, s.p90_ns);
  EXPECT_LE(s.p90_ns, s.p95_ns);
  EXPECT_LE(s.p95_ns, s.p99_ns);
  EXPECT_LE(s.p99_ns, s.p999_ns);
  EXPECT_LE(s.p999_ns, s.max_ns);
}

TEST(LatencyStat, IgnoresNegativeSamples)
{
  ros2_perf::LatencyStat ls;
  ls.add(-1);
  ls.add(-100);
  ls.add(5);
  auto s = ls.summary_total();
  EXPECT_EQ(s.count, 1u);
  EXPECT_EQ(s.min_ns, 5);
}

TEST(LatencyStat, WindowResets)
{
  ros2_perf::LatencyStat ls;
  add_all(ls, {10, 20, 30});
  auto w1 = ls.summary_window_and_reset();
  EXPECT_EQ(w1.count, 3u);

  auto w2 = ls.summary_window_and_reset();
  EXPECT_EQ(w2.count, 0u);

  // Total samples should still all be present.
  auto t = ls.summary_total();
  EXPECT_EQ(t.count, 3u);
}

TEST(LatencyStat, TotalCapDropsLateSamples)
{
  // Document the current behavior: once the total buffer is full,
  // additional samples are dropped from the percentile pool (but mean
  // / stddev / min / max keep tracking all samples via Welford).
  ros2_perf::LatencyStat ls(/*total_cap=*/4, /*window_cap=*/4);
  add_all(ls, {1, 2, 3, 4, 1000, 1000});
  auto s = ls.summary_total();
  // count is the size of the (capped) sample buffer used to compute
  // percentiles, not the lifetime sample count.
  EXPECT_EQ(s.count, 4u);
  EXPECT_LE(s.p99_ns, 4);
}

TEST(FmtUs, ProducesNonEmpty)
{
  EXPECT_FALSE(ros2_perf::fmt_us(static_cast<int64_t>(500)).empty());
  EXPECT_FALSE(ros2_perf::fmt_us(1500000.0).empty());
  EXPECT_FALSE(ros2_perf::fmt_mbps(1234567.0).empty());
}
