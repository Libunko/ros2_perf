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

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ros2_perf/gid.hpp"
#include "ros2_perf/mode.hpp"
#include "ros2_perf/mode_utils.hpp"
#include "ros2_perf/msg/uks.hpp"
#include "ros2_perf/report.hpp"
#include "ros2_perf/stats.hpp"

namespace ros2_perf
{

namespace
{

struct PubTrack
{
  uint32_t id = 0;
  char gid_prefix[RMW_GID_STORAGE_SIZE * 2 + 1] = {};
  uint32_t last_seq = 0;
  uint64_t recv = 0;
  uint64_t lost = 0;
  uint64_t recv_tick = 0;
  uint64_t lost_tick = 0;
  uint64_t total_bytes = 0;
  uint32_t last_size = kFixedPartBytes;
  LatencyStat lat;
};

struct PubTickSnapshot
{
  uint32_t id = 0;
  char gid_prefix[RMW_GID_STORAGE_SIZE * 2 + 1] = {};
  uint64_t recv_tick = 0;
  uint64_t lost_tick = 0;
  uint64_t recv_total = 0;
  uint64_t lost_total = 0;
  uint64_t total_bytes = 0;
  uint32_t last_size = kFixedPartBytes;
  LatencyStat::Summary lat_window{};
  bool has_lat_window = false;
  LatencyStat::Summary lat_total{};
  bool has_lat_total = false;
};

enum class PubSnapshotKind
{
  kTickReset,
  kTotals,
};

class PubRegistry
{
public:
  explicit PubRegistry(bool report_oneway_latency) : report_latency_(report_oneway_latency) {}

  void on_message(const GidKey & key, uint64_t seq, size_t sample_size, int64_t oneway_ns);

  void snapshot(std::vector<PubTickSnapshot> & out, PubSnapshotKind kind);

private:
  void apply_seq_gap(PubTrack & track, uint64_t seq);

  const bool report_latency_;
  std::mutex mu_;
  uint32_t next_id_ = 1;
  std::unordered_map<GidKey, PubTrack, GidKeyHash> pubs_;
};

void PubRegistry::apply_seq_gap(PubTrack & track, uint64_t seq)
{
  if (seq <= track.last_seq) {
    return;
  }
  const uint64_t diff = seq - track.last_seq - 1;
  track.lost += static_cast<uint64_t>(diff);
  track.lost_tick += static_cast<uint64_t>(diff);
  track.last_seq = seq;
}

void PubRegistry::on_message(
  const GidKey & key, uint64_t seq, size_t sample_size, int64_t oneway_ns)
{
  std::lock_guard<std::mutex> lock(mu_);
  auto [it, inserted] = pubs_.try_emplace(key);
  PubTrack & track = it->second;
  if (inserted) {
    track.id = next_id_++;
    format_gid_prefix(key, track.gid_prefix, sizeof(track.gid_prefix));
    track.last_seq = seq;
    track.recv = 1;
    track.recv_tick = 1;
    track.last_size = sample_size;
    track.total_bytes = sample_size;
  } else {
    track.recv++;
    track.recv_tick++;
    track.last_size = sample_size;
    track.total_bytes += sample_size;
    apply_seq_gap(track, seq);
  }
  if (report_latency_ && oneway_ns >= 0) {
    track.lat.add(oneway_ns);
  }
}

void PubRegistry::snapshot(std::vector<PubTickSnapshot> & out, PubSnapshotKind kind)
{
  std::lock_guard<std::mutex> lock(mu_);
  out.clear();
  out.reserve(pubs_.size());
  for (auto & entry : pubs_) {
    PubTrack & p = entry.second;
    if (kind == PubSnapshotKind::kTickReset) {
      if (p.recv_tick == 0 && p.lost_tick == 0) {
        continue;
      }
    }
    PubTickSnapshot snap{};
    snap.id = p.id;
    std::memcpy(snap.gid_prefix, p.gid_prefix, sizeof(snap.gid_prefix));
    snap.last_size = p.last_size;
    snap.total_bytes = p.total_bytes;
    if (kind == PubSnapshotKind::kTickReset) {
      snap.recv_tick = p.recv_tick;
      snap.lost_tick = p.lost_tick;
      snap.recv_total = p.recv;
      snap.lost_total = p.lost;
      p.recv_tick = 0;
      p.lost_tick = 0;
      if (report_latency_) {
        snap.lat_window = p.lat.summary_window_and_reset();
        snap.has_lat_window = snap.lat_window.count > 0;
      }
    } else {
      snap.recv_total = p.recv;
      snap.lost_total = p.lost;
      if (report_latency_) {
        snap.lat_total = p.lat.summary_total();
        snap.has_lat_total = snap.lat_total.count > 0;
      }
    }
    out.push_back(snap);
  }
  std::sort(out.begin(), out.end(), [](const PubTickSnapshot & a, const PubTickSnapshot & b) {
    return a.id < b.id;
  });
}

}  // namespace

namespace
{

class SubMode : public ModeBase
{
public:
  explicit SubMode(const SubArgs & args) : args_(args) {}

  void setup(rclcpp::Node::SharedPtr node, RuntimeCtx & ctx) override
  {
    ctx_ = &ctx;
    node_ = node;
    pubs_.emplace(ctx.cfg.report_oneway_latency);
    rclcpp::QoS qos = make_qos(ctx.cfg);
    sub_peers_ = PeerTracker::create("sub", ctx.cfg.topic, PeerQueryKind::kPublishers);
    sub_ = make_subscription(
      node, ctx.cfg.topic, qos, args_.trigger,
      [this](ros2_perf::msg::Uks::SharedPtr msg, const rclcpp::MessageInfo & info) {
        handle(*msg, info);
      },
      sub_peers_);
  }

  void start() override
  {
    if (args_.trigger == Trigger::kWaitSet) {
      worker_ = std::thread([this] {
        run_waitset_loop<ros2_perf::msg::Uks>(
          sub_, *ctx_->stop,
          [this](const ros2_perf::msg::Uks & m, const rclcpp::MessageInfo & info) {
            handle(m, info);
          });
      });
    }
  }

  void stop() override
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    if (sub_peers_) {
      sub_peers_->flush_gone();
    }
  }

  void tick(double t_now_s, double dt_s) override
  {
    if (sub_peers_) {
      sub_peers_->tick(node_, args_.trigger == Trigger::kWaitSet);
    }

    std::vector<PubTickSnapshot> pub_snap;
    pubs_->snapshot(pub_snap, PubSnapshotKind::kTickReset);

    if (pub_snap.empty() || dt_s <= 0.0) {
      return;
    }

    for (const PubTickSnapshot & p : pub_snap) {
      const double pkt_per_s = static_cast<double>(p.recv_tick) / dt_s;
      const double bps_window = static_cast<double>(p.recv_tick * p.last_size) / dt_s;
      const double bps_avg = (t_now_s > 0.0) ? static_cast<double>(p.total_bytes) / t_now_s : 0.0;
      report_sub_tick_throughput(
        t_now_s, p.gid_prefix, p.last_size, static_cast<uint64_t>(pkt_per_s + 0.5), bps_window,
        bps_avg, p.lost_tick, p.lost_total, p.recv_total);
      if (p.has_lat_window) {
        report_sub_oneway_tick(t_now_s, p.gid_prefix, p.lat_window);
      }
    }
    std::fflush(stdout);
  }

  void summary(double t_total_s) override
  {
    std::vector<PubTickSnapshot> pub_totals;
    pubs_->snapshot(pub_totals, PubSnapshotKind::kTotals);

    for (const PubTickSnapshot & p : pub_totals) {
      const double bps =
        (t_total_s > 0.0) ? static_cast<double>(p.recv_total * p.last_size) / t_total_s : 0.0;
      report_sub_summary_throughput(
        p.gid_prefix, p.recv_total, p.last_size, t_total_s, bps, p.lost_total);
      if (p.has_lat_total) {
        report_sub_oneway_summary(p.gid_prefix, p.lat_total);
      }
    }
  }

  const char * label() const override { return "sub"; }
  bool needs_executor() const override { return args_.trigger == Trigger::kListener; }
  rclcpp::Node::SharedPtr node() const override { return node_; }

private:
  void handle(const ros2_perf::msg::Uks & msg, const rclcpp::MessageInfo & info)
  {
    const size_t sample_size = static_cast<size_t>(kFixedPartBytes + msg.baggage.size());
    const GidKey key = to_key(info.get_rmw_message_info().publisher_gid);

    int64_t oneway_ns = -1;
    if (ctx_->cfg.report_oneway_latency && msg.stamp_ns != 0) {
      const uint64_t recv_ns =
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count());
      oneway_ns = static_cast<int64_t>(recv_ns) - static_cast<int64_t>(msg.stamp_ns);
    }
    pubs_->on_message(key, msg.seq, sample_size, oneway_ns);
  }

  SubArgs args_;
  RuntimeCtx * ctx_ = nullptr;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<ros2_perf::msg::Uks>::SharedPtr sub_;
  std::shared_ptr<PeerTracker> sub_peers_;
  std::thread worker_;
  std::optional<PubRegistry> pubs_;
};

}  // namespace

std::shared_ptr<ModeBase> make_sub_mode(const ModeSpec & spec)
{
  return std::make_shared<SubMode>(spec.sub);
}

}  // namespace ros2_perf
