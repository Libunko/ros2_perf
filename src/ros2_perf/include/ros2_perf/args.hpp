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

#ifndef ROS2_PERF__ARGS_HPP_
#define ROS2_PERF__ARGS_HPP_

#include <string>

#include "ros2_perf/config.hpp"

namespace ros2_perf
{

// Outcome of parsing argv.
struct ParseResult
{
  enum class Status
  {
    kOk,       // Continue, run modes from cfg.
    kHelp,     // Print help, exit 0.
    kVersion,  // Print version, exit 0.
    kError,    // Print error + usage, exit 3.
  };
  Status status = Status::kOk;
  Config cfg;
  std::string error;
};

// Parses ros2_perf-specific argv. Anything from "--ros-args" onward is
// left untouched for rclcpp::init. The function reports back, via
// *consumed_argc, how many leading arguments it consumed.
ParseResult parse_args(int argc, char ** argv, int * consumed_argc);

// Returns the ddsperf-style help text.
std::string usage_text();

}  // namespace ros2_perf

#endif  // ROS2_PERF__ARGS_HPP_
