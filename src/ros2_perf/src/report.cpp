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

#include "ros2_perf/report.hpp"

#include <cinttypes>
#include <cstdio>

namespace ros2_perf
{

double loss_rate_pct(uint64_t lost, uint64_t received)
{
  const uint64_t expected = received + lost;
  if (expected == 0) {
    return 0.0;
  }
  return 100.0 * static_cast<double>(lost) / static_cast<double>(expected);
}

void fmt_us_buf(int64_t ns, char * buf, size_t cap)
{
  const double us = static_cast<double>(ns) / 1000.0;
  if (us < 1000.0) {
    std::snprintf(buf, cap, "%.1f", us);
  } else if (us < 1e6) {
    std::snprintf(buf, cap, "%.0f", us);
  } else {
    std::snprintf(buf, cap, "%.3e", us);
  }
}

void fmt_mbps_buf(double bytes_per_s, char * buf, size_t cap)
{
  const double mbps = bytes_per_s * 8.0 / 1e6;
  if (mbps < 1000.0) {
    std::snprintf(buf, cap, "%.1f", mbps);
  } else {
    std::snprintf(buf, cap, "%.0f", mbps);
  }
}

void fill_latency_fmt_bufs(const LatencyStat::Summary & s, LatencyFmtBufs & out)
{
  fmt_us_buf(s.min_ns, out.min, sizeof(out.min));
  fmt_us_buf(static_cast<int64_t>(s.mean_ns), out.mean, sizeof(out.mean));
  fmt_us_buf(s.p50_ns, out.p50, sizeof(out.p50));
  fmt_us_buf(s.p90_ns, out.p90, sizeof(out.p90));
  fmt_us_buf(s.p95_ns, out.p95, sizeof(out.p95));
  fmt_us_buf(s.p99_ns, out.p99, sizeof(out.p99));
  fmt_us_buf(s.p999_ns, out.p999, sizeof(out.p999));
  fmt_us_buf(s.max_ns, out.max, sizeof(out.max));
  fmt_us_buf(static_cast<int64_t>(s.stddev_ns), out.stddev, sizeof(out.stddev));
}

void report_sub_tick_throughput(
  double t_now_s, const char * gid_prefix, uint32_t bytes_per_pkt, uint64_t pkt_per_s,
  double mbps_window, double mbps_avg, uint64_t lost_tick, uint64_t lost_total, uint64_t pkts_total)
{
  char mbps_win[32];
  char mbps_run[32];
  fmt_mbps_buf(mbps_window, mbps_win, sizeof(mbps_win));
  fmt_mbps_buf(mbps_avg, mbps_run, sizeof(mbps_run));
  const uint64_t recv_tick = pkt_per_s;
  const double loss_tick_pct = loss_rate_pct(lost_tick, recv_tick);
  const double loss_total_pct = loss_rate_pct(lost_total, pkts_total);
  std::printf(
    "[t=%0.3f] sub: %s  %u B/pkt  %" PRIu64
    " pkt/s  %s Mb/s  %s Mb/s avg  "
    "lost/s %" PRIu64 " (%.2f%%)  lost total %" PRIu64
    " (%.2f%%)  "
    "pkts total %" PRIu64 "\n",
    t_now_s, gid_prefix, bytes_per_pkt, pkt_per_s, mbps_win, mbps_run, lost_tick, loss_tick_pct,
    lost_total, loss_total_pct, pkts_total);
}

void report_sub_oneway_tick(double t_now_s, const char * gid_prefix, const LatencyStat::Summary & w)
{
  LatencyFmtBufs b{};
  fill_latency_fmt_bufs(w, b);
  std::printf(
    "[t=%0.3f] sub: %s  oneway(us) cnt %" PRIu64
    "  min %s  mean %s  "
    "50%% %s  90%% %s  95%% %s  99%% %s  99.9%% %s  max %s  stddev %s\n",
    t_now_s, gid_prefix, w.count, b.min, b.mean, b.p50, b.p90, b.p95, b.p99, b.p999, b.max,
    b.stddev);
}

void report_sub_summary_throughput(
  const char * gid_prefix, uint64_t pkts_total, uint32_t bytes_per_pkt, double duration_s,
  double mbps, uint64_t lost_total)
{
  char mbps_buf[32];
  fmt_mbps_buf(mbps, mbps_buf, sizeof(mbps_buf));
  std::printf(
    "sub: %s  pkts total %" PRIu64
    "  %u B/pkt  duration %.3f s  %s Mb/s  "
    "lost total %" PRIu64 " (%.2f%%)\n",
    gid_prefix, pkts_total, bytes_per_pkt, duration_s, mbps_buf, lost_total,
    loss_rate_pct(lost_total, pkts_total));
}

void report_sub_oneway_summary(const char * gid_prefix, const LatencyStat::Summary & s)
{
  LatencyFmtBufs b{};
  fill_latency_fmt_bufs(s, b);
  std::printf(
    "sub: %s  oneway (us)  cnt %" PRIu64
    "  min %s  mean %s  "
    "50%% %s  90%% %s  95%% %s  99%% %s  99.9%% %s  max %s  stddev %s\n",
    gid_prefix, s.count, b.min, b.mean, b.p50, b.p90, b.p95, b.p99, b.p999, b.max, b.stddev);
}

void report_ping_count_tick(
  double t_now_s, uint64_t pkt_per_s, uint64_t sent_per_s, uint64_t lost_tick, uint64_t lost_total,
  uint64_t pongs_total)
{
  const uint64_t recv_tick = pkt_per_s;
  std::printf(
    "[t=%0.3f] ping: %" PRIu64 " pkt/s  %" PRIu64
    " sent/s  "
    "lost/s %" PRIu64 " (%.2f%%)  lost total %" PRIu64
    " (%.2f%%)  "
    "pongs total %" PRIu64 "\n",
    t_now_s, pkt_per_s, sent_per_s, lost_tick, loss_rate_pct(lost_tick, recv_tick), lost_total,
    loss_rate_pct(lost_total, pongs_total + lost_total), pongs_total);
}

void report_ping_no_pong_tick(
  double t_now_s, uint64_t sent_per_s, uint64_t lost_tick, uint64_t lost_total, uint64_t sent_total)
{
  std::printf(
    "[t=%0.3f] ping: (no pong)  sent/s %" PRIu64 "  lost/s %" PRIu64
    " (%.2f%%)  lost total %" PRIu64 " (%.2f%%)  sent total %" PRIu64 "\n",
    t_now_s, sent_per_s, lost_tick, loss_rate_pct(lost_tick, sent_per_s), lost_total,
    loss_rate_pct(lost_total, sent_total), sent_total);
}

void report_ping_rtt_tick(double t_now_s, const char * gid_prefix, const LatencyStat::Summary & w)
{
  LatencyFmtBufs b{};
  fill_latency_fmt_bufs(w, b);
  std::printf(
    "[t=%0.3f] ping: %s rtt(us) cnt %" PRIu64
    "  min %s  mean %s  "
    "50%% %s  90%% %s  95%% %s  99%% %s  99.9%% %s  max %s  stddev %s\n",
    t_now_s, gid_prefix, w.count, b.min, b.mean, b.p50, b.p90, b.p95, b.p99, b.p999, b.max,
    b.stddev);
}

void report_ping_summary_count(
  uint64_t sent_total, uint64_t pongs_total, double duration_s, uint64_t lost_total)
{
  std::printf(
    "ping: sent %" PRIu64 "  pongs %" PRIu64
    "  duration %.3f s  "
    "lost %" PRIu64 " (%.2f%%)\n",
    sent_total, pongs_total, duration_s, lost_total,
    loss_rate_pct(lost_total, pongs_total + lost_total));
}

void report_ping_rtt_summary(const char * gid_prefix, const LatencyStat::Summary & s)
{
  LatencyFmtBufs b{};
  fill_latency_fmt_bufs(s, b);
  std::printf(
    "ping: %s rtt (us)  cnt %" PRIu64
    "  min %s  mean %s  "
    "50%% %s  90%% %s  95%% %s  99%% %s  99.9%% %s  max %s  stddev %s\n",
    gid_prefix, s.count, b.min, b.mean, b.p50, b.p90, b.p95, b.p99, b.p999, b.max, b.stddev);
}

void report_pub_tick(
  double t_now_s, const char * gid_prefix, uint64_t pkt_per_s, uint32_t bytes_per_pkt, double bps)
{
  char mbps_buf[32];
  fmt_mbps_buf(bps, mbps_buf, sizeof(mbps_buf));
  std::printf(
    "[t=%0.3f] pub: %s  %" PRIu64 " pkt/s  %u B/pkt  %s Mb/s\n", t_now_s, gid_prefix, pkt_per_s,
    bytes_per_pkt, mbps_buf);
}

void report_pub_summary(
  const char * gid_prefix, uint64_t pkts_total, uint32_t bytes_per_pkt, double duration_s,
  double bps)
{
  char mbps_buf[32];
  fmt_mbps_buf(bps, mbps_buf, sizeof(mbps_buf));
  const double pkt_per_s = duration_s > 0.0 ? static_cast<double>(pkts_total) / duration_s : 0.0;
  std::printf(
    "pub: %s  pkts total %" PRIu64
    "  %u B/pkt  duration %.3f s  "
    "%.0f pkt/s  %s Mb/s\n",
    gid_prefix, pkts_total, bytes_per_pkt, duration_s, pkt_per_s, mbps_buf);
}

void report_pong_tick(double t_now_s, const char * gid_prefix, uint64_t pkt_per_s)
{
  std::printf("[t=%0.3f] pong: %s  %" PRIu64 " pkt/s\n", t_now_s, gid_prefix, pkt_per_s);
}

void report_pong_summary(const char * gid_prefix, uint64_t pkts_total, double duration_s)
{
  const double pkt_per_s = duration_s > 0.0 ? static_cast<double>(pkts_total) / duration_s : 0.0;
  std::printf(
    "pong: %s  pkts total %" PRIu64 "  duration %.3f s  %.0f pkt/s\n", gid_prefix, pkts_total,
    duration_s, pkt_per_s);
}

void report_peer_matched(const char * role, const char * topic, const char * gid_prefix)
{
  std::printf("%s [%s]: %s matched\n", role, topic, gid_prefix);
  std::fflush(stdout);
}

void report_peer_gone(const char * role, const char * topic, const char * gid_prefix)
{
  std::printf("%s [%s]: %s gone\n", role, topic, gid_prefix);
  std::fflush(stdout);
}

}  // namespace ros2_perf
