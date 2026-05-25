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

#ifndef ROS2_PERF__REPORT_HPP_
#define ROS2_PERF__REPORT_HPP_

#include <cstddef>
#include <cstdint>

#include "ros2_perf/stats.hpp"

namespace ros2_perf
{

double loss_rate_pct(uint64_t lost, uint64_t received);

void fmt_us_buf(int64_t ns, char * buf, size_t cap);
void fmt_mbps_buf(double bytes_per_s, char * buf, size_t cap);

struct LatencyFmtBufs
{
  char min[32];
  char mean[32];
  char p50[32];
  char p90[32];
  char p95[32];
  char p99[32];
  char p999[32];
  char max[32];
  char stddev[32];
};

void fill_latency_fmt_bufs(const LatencyStat::Summary & s, LatencyFmtBufs & out);

void report_sub_tick_throughput(
  double t_now_s, const char * gid_prefix, uint32_t bytes_per_pkt, uint64_t pkt_per_s,
  double mbps_window, double mbps_avg, uint64_t lost_tick, uint64_t lost_total,
  uint64_t pkts_total);

void report_sub_oneway_tick(
  double t_now_s, const char * gid_prefix, const LatencyStat::Summary & w);

void report_sub_summary_throughput(
  const char * gid_prefix, uint64_t pkts_total, uint32_t bytes_per_pkt, double duration_s,
  double mbps, uint64_t lost_total);

void report_sub_oneway_summary(const char * gid_prefix, const LatencyStat::Summary & s);

void report_ping_count_tick(
  double t_now_s, uint64_t pkt_per_s, uint64_t sent_per_s, uint64_t lost_tick, uint64_t lost_total,
  uint64_t pongs_total);

void report_ping_no_pong_tick(
  double t_now_s, uint64_t sent_per_s, uint64_t lost_tick, uint64_t lost_total,
  uint64_t sent_total);

void report_ping_rtt_tick(double t_now_s, const char * gid_prefix, const LatencyStat::Summary & w);

void report_ping_summary_count(
  uint64_t sent_total, uint64_t pongs_total, double duration_s, uint64_t lost_total);

void report_ping_rtt_summary(const char * gid_prefix, const LatencyStat::Summary & s);

void report_pub_tick(
  double t_now_s, const char * gid_prefix, uint64_t pkt_per_s, uint32_t bytes_per_pkt, double bps);

void report_pub_summary(
  const char * gid_prefix, uint64_t pkts_total, uint32_t bytes_per_pkt, double duration_s,
  double bps);

void report_pong_tick(double t_now_s, const char * gid_prefix, uint64_t pkt_per_s);

void report_pong_summary(const char * gid_prefix, uint64_t pkts_total, double duration_s);

void report_peer_matched(const char * role, const char * topic, const char * gid_prefix);
void report_peer_gone(const char * role, const char * topic, const char * gid_prefix);

}  // namespace ros2_perf

#endif  // ROS2_PERF__REPORT_HPP_
