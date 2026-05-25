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

#include "ros2_perf/mode_utils.hpp"

#include <vector>

#include "ros2_perf/report.hpp"

namespace ros2_perf
{

std::shared_ptr<PeerTracker> PeerTracker::create(
  const char * role, std::string topic, PeerQueryKind query)
{
  return std::shared_ptr<PeerTracker>(new PeerTracker(role, std::move(topic), query));
}

PeerTracker::PeerTracker(const char * role, std::string topic, PeerQueryKind query)
: role_(role), topic_(std::move(topic)), query_(query)
{
}

void PeerTracker::sync_from_graph(const rclcpp::Node::SharedPtr & node)
{
  std::vector<rclcpp::TopicEndpointInfo> endpoints;
  if (query_ == PeerQueryKind::kPublishers) {
    endpoints = node->get_publishers_info_by_topic(topic_);
  } else {
    endpoints = node->get_subscriptions_info_by_topic(topic_);
  }

  std::unordered_map<GidKey, std::array<char, RMW_GID_STORAGE_SIZE * 2 + 1>, GidKeyHash> current;
  current.reserve(endpoints.size());
  for (const auto & ep : endpoints) {
    const GidKey key = to_key(ep.endpoint_gid());
    auto & prefix = current[key];
    format_gid_prefix(key, prefix.data(), prefix.size());
  }

  std::lock_guard<std::mutex> lock(mu_);
  for (const auto & entry : current) {
    if (known_.find(entry.first) == known_.end()) {
      known_.emplace(entry.first, entry.second);
      report_peer_matched(role_, topic_.c_str(), entry.second.data());
    }
  }
  for (auto it = known_.begin(); it != known_.end();) {
    if (current.find(it->first) == current.end()) {
      report_peer_gone(role_, topic_.c_str(), it->second.data());
      it = known_.erase(it);
    } else {
      ++it;
    }
  }
}

void PeerTracker::flush_gone()
{
  std::lock_guard<std::mutex> lock(mu_);
  for (const auto & entry : known_) {
    report_peer_gone(role_, topic_.c_str(), entry.second.data());
  }
  known_.clear();
}

void PeerTracker::tick(const rclcpp::Node::SharedPtr & node, bool poll_graph_in_tick)
{
  if (poll_graph_in_tick) {
    sync_from_graph(node);
  }
}

rclcpp::SubscriptionOptions PeerTracker::make_subscription_options(
  const rclcpp::Node::SharedPtr & node)
{
  rclcpp::SubscriptionOptions opts;
  std::shared_ptr<PeerTracker> self = shared_from_this();
  opts.event_callbacks.matched_callback = [self, node](rclcpp::MatchedInfo &) {
    self->sync_from_graph(node);
  };
  return opts;
}

rclcpp::PublisherOptions PeerTracker::make_publisher_options(const rclcpp::Node::SharedPtr & node)
{
  rclcpp::PublisherOptions opts;
  std::shared_ptr<PeerTracker> self = shared_from_this();
  opts.event_callbacks.matched_callback = [self, node](rclcpp::MatchedInfo &) {
    self->sync_from_graph(node);
  };
  return opts;
}

rclcpp::QoS make_qos(const Config & cfg)
{
  rclcpp::QoS qos = (cfg.keep_mode == KeepMode::kKeepAll)
                      ? rclcpp::QoS(rclcpp::KeepAll())
                      : rclcpp::QoS(rclcpp::KeepLast(cfg.keep_depth));
  if (cfg.best_effort) {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  return qos;
}

rclcpp::Publisher<ros2_perf::msg::Uks>::SharedPtr make_publisher(
  const rclcpp::Node::SharedPtr & node, const std::string & topic, const rclcpp::QoS & qos,
  const std::shared_ptr<PeerTracker> & peers)
{
  if (peers) {
    return node->create_publisher<ros2_perf::msg::Uks>(
      topic, qos, peers->make_publisher_options(node));
  }
  return node->create_publisher<ros2_perf::msg::Uks>(topic, qos);
}

rclcpp::Subscription<ros2_perf::msg::Uks>::SharedPtr make_subscription(
  const rclcpp::Node::SharedPtr & node, const std::string & topic, const rclcpp::QoS & qos,
  Trigger trigger,
  std::function<void(ros2_perf::msg::Uks::SharedPtr, const rclcpp::MessageInfo &)> on_listener,
  const std::shared_ptr<PeerTracker> & peers)
{
  rclcpp::SubscriptionOptions sub_opts;
  if (peers) {
    sub_opts = peers->make_subscription_options(node);
  }
  if (trigger == Trigger::kListener) {
    return node->create_subscription<ros2_perf::msg::Uks>(
      topic, qos, std::move(on_listener), sub_opts);
  }
  return node->create_subscription<ros2_perf::msg::Uks>(
    topic, qos, [](ros2_perf::msg::Uks::SharedPtr) {}, sub_opts);
}

}  // namespace ros2_perf
