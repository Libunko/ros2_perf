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

#ifndef ROS2_PERF__MODE_HPP_
#define ROS2_PERF__MODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_perf/config.hpp"

namespace ros2_perf
{

// Common runtime context shared by all modes.
struct RuntimeCtx
{
  std::atomic<bool> * stop = nullptr;
  std::chrono::steady_clock::time_point t0;
  Config cfg;
};

class ModeBase
{
public:
  virtual ~ModeBase() = default;

  // Attach this mode to the node. Called once before spin.
  virtual void setup(rclcpp::Node::SharedPtr node, RuntimeCtx & ctx) = 0;

  // Start any background threads. Called after setup, before spin.
  virtual void start() {}

  // Stop background threads (idempotent). Called when shutting down.
  virtual void stop() {}

  // Print a one-line periodic stats update (every ~1s from the
  // central loop). dt_s is the nominal reporting interval (1.0).
  virtual void tick(double t_now_s, double dt_s) = 0;

  // Print final summary line(s) after spin returns.
  virtual void summary(double t_total_s) = 0;

  // Short label used in the summary header ("pub", "sub", ...).
  virtual const char * label() const = 0;

  // If true, this mode's node must be added to an executor (because it
  // relies on subscription callbacks). If false, the mode either does
  // not need an executor at all (pub) or runs its own WaitSet thread.
  virtual bool needs_executor() const = 0;

  // The mode's owning node, populated during setup().
  virtual rclcpp::Node::SharedPtr node() const = 0;
};

std::shared_ptr<ModeBase> make_pub_mode(const ModeSpec & spec);
std::shared_ptr<ModeBase> make_sub_mode(const ModeSpec & spec);
std::shared_ptr<ModeBase> make_ping_mode(const ModeSpec & spec);
std::shared_ptr<ModeBase> make_pong_mode(const ModeSpec & spec);

}  // namespace ros2_perf

#endif  // ROS2_PERF__MODE_HPP_
