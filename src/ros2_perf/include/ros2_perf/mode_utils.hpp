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

#ifndef ROS2_PERF__MODE_UTILS_HPP_
#define ROS2_PERF__MODE_UTILS_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "ros2_perf/config.hpp"
#include "ros2_perf/gid.hpp"
#include "ros2_perf/msg/uks.hpp"

namespace ros2_perf
{

class PeerTracker;

// Size of the fixed (non-baggage) part of the Uks message: 8 bytes for
// `seq`, 8 bytes for `stamp_ns`. The numbers reported as "B/sample"
// intentionally exclude CDR encapsulation / sequence-length headers;
// they are pre-serialization payload sizes.
constexpr uint32_t kFixedPartBytes = 16;

enum class PeerQueryKind
{
  kPublishers,
  kSubscriptions,
};

// Tracks endpoint GIDs on a topic via graph queries; reports matched/gone.
class PeerTracker : public std::enable_shared_from_this<PeerTracker>
{
public:
  static std::shared_ptr<PeerTracker> create(
    const char * role, std::string topic, PeerQueryKind query);

  void sync_from_graph(const rclcpp::Node::SharedPtr & node);
  void flush_gone();
  void tick(const rclcpp::Node::SharedPtr & node, bool poll_graph_in_tick);

  rclcpp::SubscriptionOptions make_subscription_options(const rclcpp::Node::SharedPtr & node);
  rclcpp::PublisherOptions make_publisher_options(const rclcpp::Node::SharedPtr & node);

private:
  PeerTracker(const char * role, std::string topic, PeerQueryKind query);

  const char * role_;
  std::string topic_;
  PeerQueryKind query_;
  mutable std::mutex mu_;
  std::unordered_map<GidKey, std::array<char, RMW_GID_STORAGE_SIZE * 2 + 1>, GidKeyHash> known_;
};

rclcpp::QoS make_qos(const Config & cfg);

rclcpp::Publisher<ros2_perf::msg::Uks>::SharedPtr make_publisher(
  const rclcpp::Node::SharedPtr & node, const std::string & topic, const rclcpp::QoS & qos,
  const std::shared_ptr<PeerTracker> & peers = nullptr);

rclcpp::Subscription<ros2_perf::msg::Uks>::SharedPtr make_subscription(
  const rclcpp::Node::SharedPtr & node, const std::string & topic, const rclcpp::QoS & qos,
  Trigger trigger,
  std::function<void(ros2_perf::msg::Uks::SharedPtr, const rclcpp::MessageInfo &)> on_listener,
  const std::shared_ptr<PeerTracker> & peers = nullptr);

// Run the standard "create a WaitSet, take everything available, dispatch
// to a callable" loop until `stop` is set. The handler is invoked with
// `(const Msg & msg, const rclcpp::MessageInfo & info)`.
//
// Identical control flow used to live in three places (sub_mode,
// ping_mode, pong_mode); deduplicating it here keeps termination
// behavior (the 100 ms timeout, the inner drain loop) consistent.
template <typename Msg, typename Handler>
void run_waitset_loop(
  std::shared_ptr<rclcpp::Subscription<Msg>> sub, std::atomic<bool> & stop, Handler && handler,
  std::chrono::milliseconds wait_timeout = std::chrono::milliseconds(100))
{
  rclcpp::WaitSet ws;
  ws.add_subscription(sub);
  while (!stop.load()) {
    auto r = ws.wait(wait_timeout);
    if (r.kind() != rclcpp::WaitResultKind::Ready) {
      continue;
    }
    Msg m;
    rclcpp::MessageInfo info;
    while (sub->take(m, info)) {
      handler(m, info);
    }
  }
}

}  // namespace ros2_perf

#endif  // ROS2_PERF__MODE_UTILS_HPP_
