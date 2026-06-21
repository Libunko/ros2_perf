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

#ifndef ROS2_PERF__CONFIG_HPP_
#define ROS2_PERF__CONFIG_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace ros2_perf
{

enum class ModeKind
{
  kPub,
  kSub,
  kPing,
  kPong,
};

enum class Trigger
{
  kListener,
  kWaitSet,
};

enum class ExecutorKind
{
  kSingle,
  kMulti,
  kStatic,
};

enum class KeepMode
{
  kKeepLast,
  kKeepAll,
};

struct PubArgs
{
  // 0 means "as fast as possible" (ddsperf "inf").
  double rate_hz = 0.0;
  uint32_t size_bytes = 12;  // Minimum payload (ddsperf Uks default-ish).
  uint32_t burst = 1;
};

struct SubArgs
{
  Trigger trigger = Trigger::kListener;
};

struct PingArgs
{
  // 0 means "send next ping as soon as a pong arrives".
  double rate_hz = 0.0;
  uint32_t size_bytes = 12;
  Trigger trigger = Trigger::kListener;
};

struct PongArgs
{
  Trigger trigger = Trigger::kListener;
};

struct ModeSpec
{
  ModeKind kind;
  PubArgs pub;
  SubArgs sub;
  PingArgs ping;
  PongArgs pong;
};

struct Config
{
  std::vector<ModeSpec> modes;

  std::string topic = "/ros2_perf";
  bool best_effort = false;
  KeepMode keep_mode = KeepMode::kKeepAll;
  uint32_t keep_depth = 0;

  // 0 means "run until SIGINT".
  double duration_s = 0.0;

  bool intra_process = false;          // -L
  bool report_oneway_latency = false;  // -l

  // Negative means "leave domain id alone".
  int32_t domain_id = -1;

  ExecutorKind executor = ExecutorKind::kMulti;
};

}  // namespace ros2_perf

#endif  // ROS2_PERF__CONFIG_HPP_
