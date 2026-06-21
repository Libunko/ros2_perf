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

#ifndef ROS2_PERF__STATS_HPP_
#define ROS2_PERF__STATS_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace ros2_perf
{

// Online statistics for a stream of latency samples in nanoseconds.
// Mean, stddev, min, max, and percentiles are computed on demand from
// stored samples via Welford's algorithm plus a single sort.
//
// Two sample buffers are maintained:
//   * total:   capped at total_cap (default 200k) so memory stays
//              bounded on long runs. Once the cap is reached, additional
//              samples are dropped; summary stats describe only the
//              stored samples (the first total_cap received).
//   * window:  capped at window_cap; cleared at every print interval;
//              used for the per-second percentile line.
//
// The lock is taken on each add(). Hot-path use is fine because lat data
// only updates once per received message, not per-byte.
class LatencyStat
{
public:
  explicit LatencyStat(size_t total_cap = 200000, size_t window_cap = 200000);

  // ns must be non-negative.
  void add(int64_t ns);

  struct Summary
  {
    uint64_t count = 0;
    int64_t min_ns = 0;
    int64_t max_ns = 0;
    double mean_ns = 0.0;
    double stddev_ns = 0.0;
    int64_t p50_ns = 0;
    int64_t p90_ns = 0;
    int64_t p95_ns = 0;
    int64_t p99_ns = 0;
    int64_t p999_ns = 0;
  };

  // Snapshot of total samples.
  Summary summary_total() const;

  // Snapshot of the per-window samples, then clears the window.
  Summary summary_window_and_reset();

private:
  static Summary compute_summary(std::vector<int64_t> samples);

  mutable std::mutex m_;
  size_t total_cap_;
  size_t window_cap_;
  std::vector<int64_t> total_samples_;
  std::vector<int64_t> window_samples_;
};

// Pretty-print microseconds with a few decimal digits depending on
// magnitude (mimics ddsperf's compact output style).
std::string fmt_us(double ns);
std::string fmt_us(int64_t ns);

// Pretty-print a bandwidth in Mb/s.
std::string fmt_mbps(double bytes_per_s);

}  // namespace ros2_perf

#endif  // ROS2_PERF__STATS_HPP_
