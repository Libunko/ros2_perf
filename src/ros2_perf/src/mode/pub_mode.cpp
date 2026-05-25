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
#include <cmath>
#include <cstdint>
#include <memory>
#include <thread>

#include "ros2_perf/gid.hpp"
#include "ros2_perf/mode.hpp"
#include "ros2_perf/mode_utils.hpp"
#include "ros2_perf/msg/uks.hpp"
#include "ros2_perf/report.hpp"

namespace ros2_perf
{

namespace
{

class PubMode : public ModeBase
{
public:
  explicit PubMode(const PubArgs & args) : args_(args) {}

  void setup(rclcpp::Node::SharedPtr node, RuntimeCtx & ctx) override
  {
    ctx_ = &ctx;
    node_ = node;
    pub_peers_ = PeerTracker::create("pub", ctx.cfg.topic, PeerQueryKind::kSubscriptions);
    pub_ = make_publisher(node, ctx.cfg.topic, make_qos(ctx.cfg), pub_peers_);
    const uint32_t baggage =
      (args_.size_bytes > kFixedPartBytes) ? (args_.size_bytes - kFixedPartBytes) : 0u;
    msg_.baggage.assign(baggage, 0xaa);
    msg_.seq = 0;
    format_rmw_gid_prefix(pub_->get_gid(), gid, sizeof(gid));

    if (args_.rate_hz > 0.0) {
      const uint32_t burst = std::max(1u, args_.burst);
      double msg_rate_hz_ = args_.rate_hz * static_cast<double>(burst);
      interval_ns_ = static_cast<int64_t>(std::llround(1e9 / msg_rate_hz_));
    }
  }

  void start() override
  {
    worker_ = std::thread([this] { run(); });
  }

  void stop() override
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    if (pub_peers_) {
      pub_peers_->flush_gone();
    }
  }

  void tick(double t_now_s, double dt_s) override
  {
    if (pub_peers_) {
      pub_peers_->tick(node_, false);
    }

    if (dt_s <= 0.0) {
      return;
    }

    const uint64_t sent = sent_.load();
    const uint64_t delta_msgs = sent - last_sent_;
    last_sent_ = sent;
    const uint64_t pkt_per_s = static_cast<uint64_t>(static_cast<double>(delta_msgs) / dt_s + 0.5);

    const uint64_t size = (args_.size_bytes > kFixedPartBytes) ? args_.size_bytes : kFixedPartBytes;
    const double bps = static_cast<double>(delta_msgs * size) / dt_s;
    report_pub_tick(t_now_s, gid, pkt_per_s, static_cast<uint32_t>(size), bps);
    std::fflush(stdout);
  }

  void summary(double t_total_s) override
  {
    const uint64_t sent = sent_.load();
    const uint32_t size = static_cast<uint32_t>(
      (args_.size_bytes > kFixedPartBytes) ? args_.size_bytes : kFixedPartBytes);
    const double bps = t_total_s > 0.0 ? static_cast<double>(sent * size) / t_total_s : 0.0;
    report_pub_summary(gid, sent, size, t_total_s, bps);
  }

  const char * label() const override { return "pub"; }
  bool needs_executor() const override { return false; }
  rclcpp::Node::SharedPtr node() const override { return node_; }

private:
  void publish_one()
  {
    using clock = std::chrono::steady_clock;
    msg_.seq = sent_seq_++;
    msg_.stamp_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now().time_since_epoch())
        .count());
    pub_->publish(msg_);
    sent_.fetch_add(1, std::memory_order_relaxed);
  }

  void run()
  {
    using clock = std::chrono::steady_clock;

    if (args_.rate_hz <= 0.0) {
      while (!ctx_->stop->load()) {
        for (uint32_t b = 0; b < args_.burst && !ctx_->stop->load(); ++b) {
          publish_one();
        }
        std::this_thread::yield();
      }
      return;
    }

    // Sample rate = rate_hz × burst (msg/s). Message m at anchor + (m+1)*interval_ns.
    const auto anchor = ctx_->t0;
    const auto now0 = clock::now();
    const int64_t elapsed0_ns = std::max<int64_t>(
      0, std::chrono::duration_cast<std::chrono::nanoseconds>(now0 - anchor).count());
    uint64_t msg_index = static_cast<uint64_t>(elapsed0_ns / interval_ns_);

    while (!ctx_->stop->load()) {
      const auto deadline =
        anchor + std::chrono::nanoseconds(static_cast<int64_t>(msg_index + 1) * interval_ns_);

      const auto now = clock::now();
      if (now < deadline) {
        std::this_thread::sleep_until(deadline);
        if (ctx_->stop->load()) {
          break;
        }
      }

      publish_one();
      msg_index++;
    }
  }

  PubArgs args_;
  RuntimeCtx * ctx_ = nullptr;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<PeerTracker> pub_peers_;
  rclcpp::Publisher<ros2_perf::msg::Uks>::SharedPtr pub_;
  ros2_perf::msg::Uks msg_;
  std::thread worker_;
  std::atomic<uint64_t> sent_{0};
  uint64_t last_sent_ = 0;
  uint64_t sent_seq_ = 0;
  char gid[RMW_GID_STORAGE_SIZE * 2 + 1] = {};
  int64_t interval_ns_ = 0;
};

}  // namespace

std::shared_ptr<ModeBase> make_pub_mode(const ModeSpec & spec)
{
  return std::make_shared<PubMode>(spec.pub);
}

}  // namespace ros2_perf
