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

#include <unistd.h>  // getpid

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rmw/rmw.h"
#include "ros2_perf/args.hpp"
#include "ros2_perf/mode.hpp"
#include "ros2_perf/version.hpp"

namespace
{

std::atomic<bool> g_stop{false};

extern "C" void sigint_handler(int signum)  // NOLINT(readability/casting)
{
  // Only async-signal-safe operation here: atomic store.
  (void)signum;
  g_stop.store(true);
}

const char * mode_keyword(ros2_perf::ModeKind k)
{
  switch (k) {
    case ros2_perf::ModeKind::kPub:
      return "pub";
    case ros2_perf::ModeKind::kSub:
      return "sub";
    case ros2_perf::ModeKind::kPing:
      return "ping";
    case ros2_perf::ModeKind::kPong:
      return "pong";
  }
  return "?";
}

std::string compose_mode_label(const ros2_perf::Config & cfg)
{
  std::string s;
  for (size_t i = 0; i < cfg.modes.size(); ++i) {
    if (i) {
      s += "+";
    }
    s += mode_keyword(cfg.modes[i].kind);
  }
  return s;
}

const char * executor_label(ros2_perf::ExecutorKind e)
{
  switch (e) {
    case ros2_perf::ExecutorKind::kSingle:
      return "single";
    case ros2_perf::ExecutorKind::kMulti:
      return "multi";
    case ros2_perf::ExecutorKind::kStatic:
      return "static";
  }
  return "?";
}

const char * rmw_label()
{
  const char * id = rmw_get_implementation_identifier();
  return id ? id : "unknown";
}

// Delay stats tick until after the last sample of each 1 s window (half of the
// shortest pub inter-sample interval) so arbitrary rate_hz/burst do not bleed
// into the next second's count.
std::chrono::nanoseconds pub_tick_slack(const ros2_perf::Config & cfg)
{
  std::chrono::nanoseconds slack{1};
  for (const ros2_perf::ModeSpec & spec : cfg.modes) {
    if (spec.kind != ros2_perf::ModeKind::kPub || spec.pub.rate_hz <= 0.0) {
      continue;
    }
    const uint32_t burst = std::max(1u, spec.pub.burst);
    const double msg_rate = spec.pub.rate_hz * static_cast<double>(burst);
    const int64_t interval_ns = static_cast<int64_t>(std::llround(1e9 / msg_rate));
    const auto half = std::chrono::nanoseconds(std::max<int64_t>(1, interval_ns / 2));
    slack = std::max(slack, half);
  }
  constexpr auto kMaxSlack = std::chrono::milliseconds(100);
  return std::min(slack, std::chrono::duration_cast<std::chrono::nanoseconds>(kMaxSlack));
}

bool uses_waitset(const ros2_perf::ModeSpec & s)
{
  switch (s.kind) {
    case ros2_perf::ModeKind::kSub:
      return s.sub.trigger == ros2_perf::Trigger::kWaitSet;
    case ros2_perf::ModeKind::kPing:
      return s.ping.trigger == ros2_perf::Trigger::kWaitSet;
    case ros2_perf::ModeKind::kPong:
      return s.pong.trigger == ros2_perf::Trigger::kWaitSet;
    case ros2_perf::ModeKind::kPub:
      return false;
  }
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  int consumed = argc;
  auto pr = ros2_perf::parse_args(argc, argv, &consumed);
  if (pr.status != ros2_perf::ParseResult::Status::kOk) {
    std::fprintf(stderr, "ros2_perf: %s\n\n", pr.error.c_str());
    std::fputs(ros2_perf::usage_text().c_str(), stderr);
    return EXIT_FAILURE;
  }

  const ros2_perf::Config & cfg = pr.cfg;

  if (cfg.domain_id >= 0) {
    const std::string v = std::to_string(cfg.domain_id);
    ::setenv("ROS_DOMAIN_ID", v.c_str(), 1);
  }

  // Hand only the remaining argv (--ros-args ...) to rclcpp::init.
  std::vector<char *> ros_argv;
  ros_argv.push_back(argv[0]);
  for (int i = consumed; i < argc; ++i) {
    ros_argv.push_back(argv[i]);
  }
  int ros_argc = static_cast<int>(ros_argv.size());

  try {
    rclcpp::init(ros_argc, ros_argv.data());
  } catch (const std::exception & e) {
    std::fprintf(stderr, "ros2_perf: rclcpp::init failed: %s\n", e.what());
    return EXIT_FAILURE;
  }

  std::signal(SIGINT, sigint_handler);
  std::signal(SIGTERM, sigint_handler);

  ros2_perf::RuntimeCtx ctx;
  ctx.stop = &g_stop;
  ctx.t0 = std::chrono::steady_clock::now();
  ctx.cfg = cfg;

  std::vector<std::shared_ptr<ros2_perf::ModeBase>> modes;
  std::vector<rclcpp::Node::SharedPtr> nodes;

  const int pid = ::getpid();
  bool warned_about_ipc_downgrade = false;

  try {
    for (size_t i = 0; i < cfg.modes.size(); ++i) {
      std::shared_ptr<ros2_perf::ModeBase> m;
      switch (cfg.modes[i].kind) {
        case ros2_perf::ModeKind::kPub:
          m = ros2_perf::make_pub_mode(cfg.modes[i]);
          break;
        case ros2_perf::ModeKind::kSub:
          m = ros2_perf::make_sub_mode(cfg.modes[i]);
          break;
        case ros2_perf::ModeKind::kPing:
          m = ros2_perf::make_ping_mode(cfg.modes[i]);
          break;
        case ros2_perf::ModeKind::kPong:
          m = ros2_perf::make_pong_mode(cfg.modes[i]);
          break;
      }
      // IPC and WaitSet are mutually exclusive in rclcpp (intra-process
      // delivery bypasses the middleware queue, which is what take()
      // reads from). Force IPC off for waitset-driven modes and emit a
      // one-time note so the discrepancy isn't a silent surprise.
      const bool ws = uses_waitset(cfg.modes[i]);
      rclcpp::NodeOptions node_opts;
      node_opts.use_intra_process_comms(cfg.intra_process && !ws);
      if (cfg.intra_process && ws && !warned_about_ipc_downgrade) {
        std::fprintf(
          stderr,
          "ros2_perf: note: -L (intra-process) disabled for waitset "
          "trigger; using inter-process delivery for those modes.\n");
        warned_about_ipc_downgrade = true;
      }
      // Include PID + index so concurrent ros2_perf processes don't
      // collide on node names.
      const std::string name = std::string("ros2_perf_") + m->label() + "_" + std::to_string(i) +
                               "_" + std::to_string(pid);
      auto node = std::make_shared<rclcpp::Node>(name, node_opts);
      m->setup(node, ctx);
      modes.push_back(m);
      nodes.push_back(node);
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "ros2_perf: setup failed: %s\n", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Build executor only if at least one mode needs it. Pub-only or
  // entirely-waitset configurations skip it.
  std::shared_ptr<rclcpp::Executor> executor;
  bool any_needs_executor = false;
  for (auto & m : modes) {
    if (m->needs_executor()) {
      any_needs_executor = true;
      break;
    }
  }
  if (any_needs_executor) {
    switch (cfg.executor) {
      case ros2_perf::ExecutorKind::kSingle:
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        break;
      case ros2_perf::ExecutorKind::kStatic:
        executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
        break;
      case ros2_perf::ExecutorKind::kMulti:
      default:
        executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        break;
    }
    for (auto & m : modes) {
      if (m->needs_executor()) {
        executor->add_node(m->node());
      }
    }
  }

  for (auto & m : modes) {
    m->start();
  }

  std::thread spin_thread;
  if (executor) {
    spin_thread = std::thread([&] { executor->spin(); });
  }

  std::printf(
    "==== ros2_perf %s ====\n"
    "modes: %s  topic: %s  qos: %s/%s/%u  ipc: %s  executor: %s  "
    "rmw: %s  domain: %d  pid: %d\n",
    ros2_perf::kVersion, compose_mode_label(cfg).c_str(), cfg.topic.c_str(),
    cfg.best_effort ? "best_effort" : "reliable",
    cfg.keep_mode == ros2_perf::KeepMode::kKeepAll ? "keep_all" : "keep_last", cfg.keep_depth,
    cfg.intra_process ? "on" : "off", executor_label(cfg.executor), rmw_label(), cfg.domain_id,
    pid);
  std::fflush(stdout);

  // Periodic tick + duration check. Poll 50 ms when far from the next tick;
  // sleep_until(next_tick) near the boundary. Slack scales with pub rate.
  const auto t0 = ctx.t0;
  using clock = std::chrono::steady_clock;
  constexpr auto kPollSlice = std::chrono::milliseconds(50);
  const auto tick_slack = pub_tick_slack(cfg);
  uint64_t tick_count = 0;
  auto next_tick = t0 + std::chrono::seconds(1) + tick_slack;
  while (!g_stop.load()) {
    const auto now = clock::now();
    const double t_wall_s = std::chrono::duration<double>(now - t0).count();

    if (now >= next_tick) {
      tick_count++;
      constexpr double kTickIntervalS = 1.0;
      for (auto & m : modes) {
        m->tick(t_wall_s, kTickIntervalS);
      }
      next_tick = t0 + std::chrono::seconds(tick_count + 1) + tick_slack;
    } else {
      const auto poll_until = next_tick - kPollSlice;
      if (now < poll_until) {
        std::this_thread::sleep_for(kPollSlice);
      } else {
        std::this_thread::sleep_until(next_tick);
      }
    }

    if (cfg.duration_s > 0.0 && t_wall_s >= cfg.duration_s) {
      g_stop.store(true);
      break;
    }
  }

  if (executor) {
    executor->cancel();
  }
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  for (auto & m : modes) {
    m->stop();
  }

  const double t_total = std::chrono::duration<double>(clock::now() - t0).count();
  std::printf("\n==== ros2_perf summary ====\n");
  std::printf(
    "modes: %s  topic: %s  qos: %s/%s/%u  ipc: %s  executor: %s  rmw: %s\n",
    compose_mode_label(cfg).c_str(), cfg.topic.c_str(),
    cfg.best_effort ? "best_effort" : "reliable",
    cfg.keep_mode == ros2_perf::KeepMode::kKeepAll ? "keep_all" : "keep_last", cfg.keep_depth,
    cfg.intra_process ? "on" : "off", executor_label(cfg.executor), rmw_label());
  for (auto & m : modes) {
    m->summary(t_total);
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
