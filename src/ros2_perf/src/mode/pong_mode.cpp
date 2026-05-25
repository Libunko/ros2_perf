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

#include <atomic>
#include <chrono>
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

class PongMode : public ModeBase
{
public:
  explicit PongMode(const PongArgs & args) : args_(args) {}

  void setup(rclcpp::Node::SharedPtr node, RuntimeCtx & ctx) override
  {
    ctx_ = &ctx;
    node_ = node;
    rclcpp::QoS qos = make_qos(ctx.cfg);
    ping_topic_ = ctx.cfg.topic + "_ping";
    pong_topic_ = ctx.cfg.topic + "_pong";

    pong_pub_peers_ = PeerTracker::create("pong", pong_topic_, PeerQueryKind::kSubscriptions);
    ping_sub_peers_ = PeerTracker::create("pong", ping_topic_, PeerQueryKind::kPublishers);

    pub_ = make_publisher(node, pong_topic_, qos, pong_pub_peers_);
    sub_ = make_subscription(
      node, ping_topic_, qos, args_.trigger,
      [this](ros2_perf::msg::Uks::SharedPtr m, const rclcpp::MessageInfo & info) {
        handle(*m, info);
      },
      ping_sub_peers_);
    format_rmw_gid_prefix(pub_->get_gid(), gid, sizeof(gid));
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
    if (pong_pub_peers_) {
      pong_pub_peers_->flush_gone();
    }
    if (ping_sub_peers_) {
      ping_sub_peers_->flush_gone();
    }
  }

  void tick(double t_now_s, double dt_s) override
  {
    const bool poll = args_.trigger == Trigger::kWaitSet;
    if (pong_pub_peers_) {
      pong_pub_peers_->tick(node_, poll);
    }
    if (ping_sub_peers_) {
      ping_sub_peers_->tick(node_, poll);
    }

    if (dt_s <= 0.0) {
      return;
    }

    const uint64_t cnt = roundtrips_.load();
    const uint64_t delta = cnt - last_cnt_;
    last_cnt_ = cnt;
    const uint64_t pkt_per_s = static_cast<uint64_t>(static_cast<double>(delta) / dt_s + 0.5);
    if (pkt_per_s == 0) {
      return;
    }
    report_pong_tick(t_now_s, gid, pkt_per_s);
    std::fflush(stdout);
  }

  void summary(double t_total_s) override
  {
    report_pong_summary(gid, roundtrips_.load(), t_total_s);
  }

  const char * label() const override { return "pong"; }
  bool needs_executor() const override { return args_.trigger == Trigger::kListener; }
  rclcpp::Node::SharedPtr node() const override { return node_; }

private:
  void handle(const ros2_perf::msg::Uks & msg, const rclcpp::MessageInfo & info)
  {
    (void)info;
    if ((msg.seq & 1u) == 0u) {
      return;
    }

    auto out = std::make_unique<ros2_perf::msg::Uks>();
    out->seq = msg.seq & ~1u;
    out->stamp_ns = msg.stamp_ns;
    out->baggage = msg.baggage;
    pub_->publish(std::move(out));
    roundtrips_.fetch_add(1, std::memory_order_relaxed);
  }

  PongArgs args_;
  RuntimeCtx * ctx_ = nullptr;
  rclcpp::Node::SharedPtr node_;
  std::string ping_topic_;
  std::string pong_topic_;
  std::shared_ptr<PeerTracker> pong_pub_peers_;
  std::shared_ptr<PeerTracker> ping_sub_peers_;
  rclcpp::Publisher<ros2_perf::msg::Uks>::SharedPtr pub_;
  rclcpp::Subscription<ros2_perf::msg::Uks>::SharedPtr sub_;
  std::thread worker_;
  std::atomic<uint64_t> roundtrips_{0};
  uint64_t last_cnt_ = 0;
  char gid[RMW_GID_STORAGE_SIZE * 2 + 1] = {};
};

}  // namespace

std::shared_ptr<ModeBase> make_pong_mode(const ModeSpec & spec)
{
  return std::make_shared<PongMode>(spec.pong);
}

}  // namespace ros2_perf
