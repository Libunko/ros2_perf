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
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
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

struct PongTrack
{
  uint32_t id = 0;
  char gid_prefix[RMW_GID_STORAGE_SIZE * 2 + 1] = {};
  uint64_t last_seq = 0;
  LatencyStat lat;
};

struct PongTickSnapshot
{
  uint32_t id = 0;
  char gid_prefix[RMW_GID_STORAGE_SIZE * 2 + 1] = {};
  LatencyStat::Summary lat_window{};
  bool has_lat_window = false;
  LatencyStat::Summary lat_total{};
  bool has_lat_total = false;
};

enum class PongSnapshotKind
{
  kTickReset,
  kTotals,
};

class PongRegistry
{
public:
  bool on_pong(const GidKey & key, uint64_t seq, int64_t rtt_ns);

  void snapshot(std::vector<PongTickSnapshot> & out, PongSnapshotKind kind);

private:
  std::mutex mu_;
  uint32_t next_id_ = 1;
  std::unordered_map<GidKey, PongTrack, GidKeyHash> pongs_;
};

bool PongRegistry::on_pong(const GidKey & key, uint64_t seq, int64_t rtt_ns)
{
  std::lock_guard<std::mutex> lock(mu_);
  auto [it, inserted] = pongs_.try_emplace(key);
  if (inserted) {
    PongTrack & track = it->second;
    track.id = next_id_++;
    format_gid_prefix(key, track.gid_prefix, sizeof(track.gid_prefix));
    track.last_seq = seq;
    track.lat.add(rtt_ns);
    return true;
  }
  PongTrack & track = it->second;
  const int64_t diff = static_cast<int64_t>(seq) - static_cast<int64_t>(track.last_seq);
  if (diff <= 0) {
    return false;
  }
  track.last_seq = seq;
  track.lat.add(rtt_ns);
  return true;
}

void PongRegistry::snapshot(std::vector<PongTickSnapshot> & out, PongSnapshotKind kind)
{
  std::lock_guard<std::mutex> lock(mu_);
  out.clear();
  out.reserve(pongs_.size());
  for (auto & entry : pongs_) {
    PongTrack & p = entry.second;
    PongTickSnapshot snap{};
    snap.id = p.id;
    std::memcpy(snap.gid_prefix, p.gid_prefix, sizeof(snap.gid_prefix));
    if (kind == PongSnapshotKind::kTickReset) {
      snap.lat_window = p.lat.summary_window_and_reset();
      snap.has_lat_window = snap.lat_window.count > 0;
    } else {
      snap.lat_total = p.lat.summary_total();
      snap.has_lat_total = snap.lat_total.count > 0;
    }
    out.push_back(snap);
  }
  std::sort(out.begin(), out.end(), [](const PongTickSnapshot & a, const PongTickSnapshot & b) {
    return a.id < b.id;
  });
}

}  // namespace

namespace
{

class PingMode : public ModeBase
{
public:
  explicit PingMode(const PingArgs & args) : args_(args) {}

  void setup(rclcpp::Node::SharedPtr node, RuntimeCtx & ctx) override
  {
    ctx_ = &ctx;
    node_ = node;
    rclcpp::QoS qos = make_qos(ctx.cfg);
    ping_topic_ = ctx.cfg.topic + "_ping";
    pong_topic_ = ctx.cfg.topic + "_pong";

    ping_pub_peers_ = PeerTracker::create("ping", ping_topic_, PeerQueryKind::kSubscriptions);
    pong_sub_peers_ = PeerTracker::create("ping", pong_topic_, PeerQueryKind::kPublishers);

    pub_ = make_publisher(node, ping_topic_, qos, ping_pub_peers_);
    sub_ = make_subscription(
      node, pong_topic_, qos, args_.trigger,
      [this](ros2_perf::msg::Uks::SharedPtr m, const rclcpp::MessageInfo & info) {
        handle(*m, info);
      },
      pong_sub_peers_);

    const uint32_t baggage =
      (args_.size_bytes > kFixedPartBytes) ? (args_.size_bytes - kFixedPartBytes) : 0u;
    baggage_template_.assign(baggage, 0xaa);
  }

  void start() override
  {
    if (args_.trigger == Trigger::kWaitSet) {
      ws_worker_ = std::thread([this] {
        run_waitset_loop<ros2_perf::msg::Uks>(
          sub_, *ctx_->stop,
          [this](const ros2_perf::msg::Uks & m, const rclcpp::MessageInfo & info) {
            handle(m, info);
          });
      });
    }

    if (args_.rate_hz > 0.0) {
      sender_ = std::thread([this] {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (ctx_->stop->load()) {
          return;
        }
        send_ping();
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / args_.rate_hz));
        auto next = clock::now() + period;
        while (!ctx_->stop->load()) {
          std::this_thread::sleep_until(next);
          if (ctx_->stop->load()) {
            break;
          }
          send_ping();
          next += period;
        }
      });
      return;
    }

    // rate_hz == 0: response-driven — send the next ping only after a pong is
    // received (see handle). One bootstrap ping starts the ping↔pong loop.
    sender_ = std::thread([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      if (ctx_->stop->load()) {
        return;
      }
      send_ping();
    });
  }

  void stop() override
  {
    if (sender_.joinable()) {
      sender_.join();
    }
    if (ws_worker_.joinable()) {
      ws_worker_.join();
    }
    if (ping_pub_peers_) {
      ping_pub_peers_->flush_gone();
    }
    if (pong_sub_peers_) {
      pong_sub_peers_->flush_gone();
    }
  }

  void tick(double t_now_s, double dt_s) override
  {
    const bool poll = args_.trigger == Trigger::kWaitSet;
    if (ping_pub_peers_) {
      ping_pub_peers_->tick(node_, poll);
    }
    if (pong_sub_peers_) {
      pong_sub_peers_->tick(node_, poll);
    }

    if (dt_s <= 0.0) {
      return;
    }

    const uint64_t pongs_now = pongs_total_.load();
    const uint64_t delta_pongs = pongs_now - last_pongs_;
    const uint64_t pings_now = pings_.load();
    const uint64_t delta_pings = pings_now - last_pings_;
    const uint64_t delta_lost = (delta_pings > delta_pongs) ? (delta_pings - delta_pongs) : 0u;
    last_pongs_ = pongs_now;
    last_pings_ = pings_now;

    const uint64_t pongs_per_s =
      static_cast<uint64_t>(static_cast<double>(delta_pongs) / dt_s + 0.5);
    const uint64_t pings_per_s =
      static_cast<uint64_t>(static_cast<double>(delta_pings) / dt_s + 0.5);
    const uint64_t lost_per_s = static_cast<uint64_t>(static_cast<double>(delta_lost) / dt_s + 0.5);

    std::vector<PongTickSnapshot> pong_snap;
    pong_registry_.snapshot(pong_snap, PongSnapshotKind::kTickReset);

    if (pongs_per_s == 0 && pings_per_s == 0 && pong_snap.empty()) {
      return;
    }

    lost_total_ += delta_lost;

    if (pongs_per_s == 0 && pings_per_s > 0) {
      report_ping_no_pong_tick(t_now_s, pings_per_s, lost_per_s, lost_total_, pings_now);
    } else {
      report_ping_count_tick(t_now_s, pongs_per_s, pings_per_s, lost_per_s, lost_total_, pongs_now);
      for (const PongTickSnapshot & p : pong_snap) {
        if (p.has_lat_window) {
          report_ping_rtt_tick(t_now_s, p.gid_prefix, p.lat_window);
        }
      }
    }
    std::fflush(stdout);
  }

  void summary(double t_total_s) override
  {
    const uint64_t sent = pings_.load();
    const uint64_t recv = pongs_total_.load();
    const uint64_t inflight = (sent > recv) ? (sent - recv) : 0u;
    report_ping_summary_count(sent, recv, t_total_s, lost_total_ + inflight);

    std::vector<PongTickSnapshot> pong_totals;
    pong_registry_.snapshot(pong_totals, PongSnapshotKind::kTotals);
    for (const PongTickSnapshot & p : pong_totals) {
      if (p.has_lat_total) {
        report_ping_rtt_summary(p.gid_prefix, p.lat_total);
      }
    }
  }

  const char * label() const override { return "ping"; }
  bool needs_executor() const override { return args_.trigger == Trigger::kListener; }
  rclcpp::Node::SharedPtr node() const override { return node_; }

private:
  void send_ping()
  {
    std::lock_guard<std::mutex> lock(send_mu_);
    auto msg = std::make_unique<ros2_perf::msg::Uks>();
    msg->seq = (next_seq_.fetch_add(1, std::memory_order_relaxed) << 1) | 1u;
    msg->stamp_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                            std::chrono::steady_clock::now().time_since_epoch())
                                            .count());
    msg->baggage = baggage_template_;
    pub_->publish(std::move(msg));
    pings_.fetch_add(1, std::memory_order_relaxed);
  }

  void handle(const ros2_perf::msg::Uks & msg, const rclcpp::MessageInfo & info)
  {
    if ((msg.seq & 1u) != 0u) {
      return;
    }

    const GidKey key = to_key(info.get_rmw_message_info().publisher_gid);
    const uint64_t now_ns =
      static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count());
    const int64_t rtt = (static_cast<int64_t>(now_ns) - static_cast<int64_t>(msg.stamp_ns)) / 2;
    if (!pong_registry_.on_pong(key, msg.seq, rtt)) {
      return;
    }
    pongs_total_.fetch_add(1, std::memory_order_relaxed);

    if (args_.rate_hz > 0.0 || ctx_->stop->load()) {
      return;
    }

    // Steady state: each accepted pong triggers the next ping.
    send_ping();
  }

  PingArgs args_;
  RuntimeCtx * ctx_ = nullptr;
  rclcpp::Node::SharedPtr node_;
  std::string ping_topic_;
  std::string pong_topic_;
  std::shared_ptr<PeerTracker> ping_pub_peers_;
  std::shared_ptr<PeerTracker> pong_sub_peers_;
  rclcpp::Publisher<ros2_perf::msg::Uks>::SharedPtr pub_;
  rclcpp::Subscription<ros2_perf::msg::Uks>::SharedPtr sub_;
  std::vector<uint8_t> baggage_template_;
  std::thread sender_;
  std::thread ws_worker_;
  std::mutex send_mu_;
  std::atomic<uint64_t> pings_{0};
  std::atomic<uint64_t> pongs_total_{0};
  std::atomic<uint32_t> next_seq_{0};
  uint64_t last_pongs_ = 0;
  uint64_t last_pings_ = 0;
  uint64_t lost_total_ = 0;
  PongRegistry pong_registry_;
};

}  // namespace

std::shared_ptr<ModeBase> make_ping_mode(const ModeSpec & spec)
{
  return std::make_shared<PingMode>(spec.ping);
}

}  // namespace ros2_perf
