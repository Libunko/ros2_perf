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

#ifndef ROS2_PERF__GID_HPP_
#define ROS2_PERF__GID_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "rmw/types.h"

namespace ros2_perf
{

// Stable, hashable, std-container-friendly key derived from rmw_gid_t.
// We do not interpret the bytes: from our point of view it is just
// RMW_GID_STORAGE_SIZE bytes that uniquely identify a publisher within
// a domain.
using GidKey = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;

struct GidKeyHash
{
  size_t operator()(const GidKey & k) const noexcept
  {
    // FNV-1a 64-bit; cheap and good enough for a handful of publishers.
    uint64_t h = 1469598103934665603ull;
    for (auto b : k) {
      h ^= b;
      h *= 1099511628211ull;
    }
    return static_cast<size_t>(h);
  }
};

inline GidKey to_key(const rmw_gid_t & g) noexcept
{
  GidKey k{};
  std::memcpy(k.data(), g.data, RMW_GID_STORAGE_SIZE);
  return k;
}

inline GidKey to_key(const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & g) noexcept
{
  GidKey k{};
  std::memcpy(k.data(), g.data(), RMW_GID_STORAGE_SIZE);
  return k;
}

// Writes the first 8 bytes of a GID as hex into buf (no "0x" prefix).
inline void format_gid_prefix(const GidKey & k, char * buf, size_t cap)
{
  if (cap == 0) {
    return;
  }
  size_t pos = 0;
  const size_t n = (k.size() < 8) ? k.size() : RMW_GID_STORAGE_SIZE;
  for (size_t i = 0; i < n && pos + 3 <= cap; ++i) {
    pos += static_cast<size_t>(std::snprintf(buf + pos, cap - pos, "%02x", k[i]));
  }
  if (pos < cap) {
    buf[pos] = '\0';
  } else {
    buf[cap - 1] = '\0';
  }
}

inline void format_rmw_gid_prefix(const rmw_gid_t & g, char * buf, size_t cap)
{
  format_gid_prefix(to_key(g), buf, cap);
}

}  // namespace ros2_perf

#endif  // ROS2_PERF__GID_HPP_
