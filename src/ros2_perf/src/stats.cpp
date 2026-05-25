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

#include "ros2_perf/stats.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace ros2_perf
{

LatencyStat::LatencyStat(size_t total_cap, size_t window_cap)
: total_cap_(total_cap), window_cap_(window_cap)
{
  total_samples_.reserve(std::min<size_t>(total_cap_, 4096));
  window_samples_.reserve(std::min<size_t>(window_cap_, 4096));
}

void LatencyStat::add(int64_t ns)
{
  if (ns < 0) {
    return;
  }
  std::lock_guard<std::mutex> g(m_);
  if (total_samples_.size() < total_cap_) {
    total_samples_.push_back(ns);
  }
  if (window_samples_.size() < window_cap_) {
    window_samples_.push_back(ns);
  }
}

namespace
{

int64_t pct_sorted(const std::vector<int64_t> & v, double p)
{
  if (v.empty()) {
    return 0;
  }
  // Nearest-rank style: index = round(p * (N-1)).
  const double idx_f = p * static_cast<double>(v.size() - 1);
  size_t idx = static_cast<size_t>(idx_f + 0.5);
  if (idx >= v.size()) {
    idx = v.size() - 1;
  }
  return v[idx];
}

}  // namespace

LatencyStat::Summary LatencyStat::compute_summary(std::vector<int64_t> samples)
{
  Summary s;
  s.count = samples.size();
  if (samples.empty()) {
    return s;
  }
  // Single Welford pass: min, max, mean, M2 (for stddev).
  int64_t mn = samples.front();
  int64_t mx = samples.front();
  long double mean = 0.0L;
  long double m2 = 0.0L;
  uint64_t n = 0;
  for (auto v : samples) {
    ++n;
    if (v < mn) {
      mn = v;
    }
    if (v > mx) {
      mx = v;
    }
    const long double x = static_cast<long double>(v);
    const long double delta = x - mean;
    mean += delta / static_cast<long double>(n);
    const long double delta2 = x - mean;
    m2 += delta * delta2;
  }
  s.min_ns = mn;
  s.max_ns = mx;
  s.mean_ns = static_cast<double>(mean);
  const long double var = (n > 1) ? (m2 / static_cast<long double>(n)) : 0.0L;
  s.stddev_ns = static_cast<double>(std::sqrt(static_cast<double>(var)));

  // One sort buys us every percentile we want, including arbitrary
  // future ones. For typical sample counts (<= 200k) this is well below
  // a millisecond and far cheaper than 5 separate nth_element passes
  // that polluted each other's partial ordering.
  std::sort(samples.begin(), samples.end());
  s.p50_ns = pct_sorted(samples, 0.50);
  s.p90_ns = pct_sorted(samples, 0.90);
  s.p95_ns = pct_sorted(samples, 0.95);
  s.p99_ns = pct_sorted(samples, 0.99);
  s.p999_ns = pct_sorted(samples, 0.999);
  return s;
}

LatencyStat::Summary LatencyStat::summary_total() const
{
  std::vector<int64_t> copy;
  {
    std::lock_guard<std::mutex> g(m_);
    copy = total_samples_;
  }
  return compute_summary(std::move(copy));
}

LatencyStat::Summary LatencyStat::summary_window_and_reset()
{
  std::vector<int64_t> copy;
  {
    std::lock_guard<std::mutex> g(m_);
    copy.swap(window_samples_);
    window_samples_.reserve(std::min<size_t>(window_cap_, 4096));
  }
  return compute_summary(std::move(copy));
}

std::string fmt_us(double ns)
{
  char buf[32];
  const double us = ns / 1000.0;
  if (us < 1000.0) {
    std::snprintf(buf, sizeof(buf), "%.1f", us);
  } else if (us < 1e6) {
    std::snprintf(buf, sizeof(buf), "%.0f", us);
  } else {
    std::snprintf(buf, sizeof(buf), "%.3e", us);
  }
  return buf;
}

std::string fmt_us(int64_t ns) { return fmt_us(static_cast<double>(ns)); }

std::string fmt_mbps(double bytes_per_s)
{
  char buf[32];
  const double mbps = bytes_per_s * 8.0 / 1e6;
  if (mbps < 1000.0) {
    std::snprintf(buf, sizeof(buf), "%.1f", mbps);
  } else {
    std::snprintf(buf, sizeof(buf), "%.0f", mbps);
  }
  return buf;
}

}  // namespace ros2_perf
