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

#include "ros2_perf/args.hpp"

#include <cctype>
#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

namespace ros2_perf
{

namespace
{

// Parse a base-10 int64 from a string. Stricter than std::strtol:
// rejects empty input, trailing garbage, and leading whitespace.
bool parse_strict_int64(const std::string & s, int64_t * out)
{
  if (s.empty()) {
    return false;
  }
  const char * first = s.data();
  const char * last = s.data() + s.size();
  int64_t v = 0;
  auto r = std::from_chars(first, last, v, 10);
  if (r.ec != std::errc{} || r.ptr != last) {
    return false;
  }
  *out = v;
  return true;
}

bool is_mode_keyword(const std::string & s)
{
  return s == "pub" || s == "sub" || s == "ping" || s == "pong" || s == "sanity" || s == "help";
}

// Boundary that stops a MODE's sub-argument run: either the start of the
// next MODE keyword, or the start of an OPTION (a token beginning with
// '-'). Keeping this consistent across all sub-arg parsers lets users
// freely mix options and modes in the command line.
bool is_subargs_boundary(const std::string & s)
{
  return is_mode_keyword(s) || (!s.empty() && s[0] == '-');
}

// Try to parse a string of the form "<number><suffix>" where the suffix is
// one of the entries in `units`. On success, multiplies the number by the
// unit multiplier and writes the result to *out. Returns false if the
// suffix is unknown or the number is malformed.
//
// Number parsing is delegated entirely to strtod, which already rejects
// garbage like "+-1" and "1.2.3"; we only split numeric prefix from
// alphabetic suffix.
bool parse_with_units(
  const std::string & s, const std::vector<std::pair<std::string, double>> & units,
  double default_mult, double * out)
{
  if (s.empty()) {
    return false;
  }
  char * end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str()) {
    return false;
  }
  const std::string suffix(end);
  if (suffix.empty()) {
    *out = v * default_mult;
    return true;
  }
  for (const auto & u : units) {
    if (u.first == suffix) {
      *out = v * u.second;
      return true;
    }
  }
  return false;
}

bool parse_frequency(const std::string & s, double * hz)
{
  if (s == "inf") {
    *hz = 0.0;  // 0 means "as fast as possible".
    return true;
  }
  static const std::vector<std::pair<std::string, double>> units = {
    {"Hz", 1.0},
    {"kHz", 1000.0},
  };
  return parse_with_units(s, units, 1.0, hz);
}

bool parse_size(const std::string & s, uint32_t * bytes)
{
  // Note: "k" / "M" and "kB" / "MB" are 1024-based, matching ddsperf.
  // SI consumers should use "KiB" / "MiB" for clarity (same multiplier).
  static const std::vector<std::pair<std::string, double>> units = {
    {"B", 1.0},      {"k", 1024.0},     {"M", 1048576.0},   {"kB", 1024.0},
    {"KiB", 1024.0}, {"MB", 1048576.0}, {"MiB", 1048576.0},
  };
  double v = 0.0;
  if (!parse_with_units(s, units, 1.0, &v)) {
    return false;
  }
  if (v < 0.0 || v > static_cast<double>(UINT32_MAX)) {
    return false;
  }
  *bytes = static_cast<uint32_t>(v);
  return true;
}

bool parse_trigger(const std::string & s, Trigger * t)
{
  if (s == "listener") {
    *t = Trigger::kListener;
    return true;
  }
  if (s == "waitset") {
    *t = Trigger::kWaitSet;
    return true;
  }
  return false;
}

bool parse_pub_subargs(
  const std::vector<std::string> & toks, size_t & i, PubArgs & pa, std::string & err)
{
  while (i < toks.size() && !is_subargs_boundary(toks[i])) {
    const std::string & t = toks[i];
    if (t == "size") {
      if (i + 1 >= toks.size()) {
        err = "'size' requires a value";
        return false;
      }
      if (!parse_size(toks[i + 1], &pa.size_bytes)) {
        err = "invalid size '" + toks[i + 1] + "'";
        return false;
      }
      i += 2;
    } else if (t == "burst") {
      if (i + 1 >= toks.size()) {
        err = "'burst' requires a value";
        return false;
      }
      int64_t n = 0;
      if (!parse_strict_int64(toks[i + 1], &n) || n < 1) {
        err = "invalid burst '" + toks[i + 1] + "'";
        return false;
      }
      pa.burst = static_cast<uint32_t>(n);
      i += 2;
    } else {
      double hz = 0.0;
      if (parse_frequency(t, &hz)) {
        pa.rate_hz = hz;
        ++i;
      } else {
        err = "unknown pub argument '" + t + "'";
        return false;
      }
    }
  }
  return true;
}

bool parse_sub_subargs(
  const std::vector<std::string> & toks, size_t & i, SubArgs & sa, std::string & err)
{
  while (i < toks.size() && !is_subargs_boundary(toks[i])) {
    Trigger tr;
    if (parse_trigger(toks[i], &tr)) {
      sa.trigger = tr;
      ++i;
    } else {
      err = "unknown sub argument '" + toks[i] + "'";
      return false;
    }
  }
  return true;
}

bool parse_ping_subargs(
  const std::vector<std::string> & toks, size_t & i, PingArgs & pa, std::string & err)
{
  while (i < toks.size() && !is_subargs_boundary(toks[i])) {
    const std::string & t = toks[i];
    if (t == "size") {
      if (i + 1 >= toks.size()) {
        err = "'size' requires a value";
        return false;
      }
      if (!parse_size(toks[i + 1], &pa.size_bytes)) {
        err = "invalid size '" + toks[i + 1] + "'";
        return false;
      }
      i += 2;
      continue;
    }
    Trigger tr;
    if (parse_trigger(t, &tr)) {
      pa.trigger = tr;
      ++i;
      continue;
    }
    double hz = 0.0;
    if (parse_frequency(t, &hz)) {
      pa.rate_hz = hz;
      ++i;
      continue;
    }
    err = "unknown ping argument '" + t + "'";
    return false;
  }
  return true;
}

bool parse_pong_subargs(
  const std::vector<std::string> & toks, size_t & i, PongArgs & pa, std::string & err)
{
  while (i < toks.size() && !is_subargs_boundary(toks[i])) {
    Trigger tr;
    if (parse_trigger(toks[i], &tr)) {
      pa.trigger = tr;
      ++i;
    } else {
      err = "unknown pong argument '" + toks[i] + "'";
      return false;
    }
  }
  return true;
}

bool parse_executor(const std::string & s, ExecutorKind * e)
{
  if (s == "single") {
    *e = ExecutorKind::kSingle;
    return true;
  }
  if (s == "multi") {
    *e = ExecutorKind::kMulti;
    return true;
  }
  if (s == "static") {
    *e = ExecutorKind::kStatic;
    return true;
  }
  return false;
}

bool parse_keep(const std::string & s, KeepMode * km, uint32_t * depth)
{
  if (s == "all") {
    *km = KeepMode::kKeepAll;
    *depth = 0;
    return true;
  }
  int64_t n = 0;
  if (!parse_strict_int64(s, &n) || n < 1) {
    return false;
  }
  *km = KeepMode::kKeepLast;
  *depth = static_cast<uint32_t>(n);
  return true;
}

}  // namespace

ParseResult parse_args(int argc, char ** argv, int * consumed_argc)
{
  ParseResult result;
  Config & cfg = result.cfg;

  // Collect tokens up to (but not including) "--ros-args".
  std::vector<std::string> toks;
  int stop = argc;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--ros-args") == 0) {
      stop = i;
      break;
    }
    toks.emplace_back(argv[i]);
  }
  *consumed_argc = stop;

  size_t i = 0;

  // Unified loop: at each step, either consume an OPTION (token starting
  // with '-') or a MODE keyword (with its sub-arguments). Options and
  // modes may interleave freely; e.g. `sub -l -D 5` works just like
  // `-l -D 5 sub`.
  auto parse_option = [&](std::string & opt_err) -> bool {
    const std::string & t = toks[i];
    auto require_value = [&](std::string & out) -> bool {
      if (i + 1 >= toks.size()) {
        opt_err = "option '" + t + "' requires a value";
        return false;
      }
      out = toks[i + 1];
      return true;
    };
    if (t == "-h" || t == "--help") {
      result.status = ParseResult::Status::kHelp;
      ++i;
      return true;
    } else if (t == "--version") {
      result.status = ParseResult::Status::kVersion;
      ++i;
      return true;
    } else if (t == "-u") {
      cfg.best_effort = true;
      ++i;
    } else if (t == "-L") {
      cfg.intra_process = true;
      ++i;
    } else if (t == "-l") {
      cfg.report_oneway_latency = true;
      ++i;
    } else if (t == "-k") {
      std::string v;
      if (!require_value(v)) {
        return false;
      }
      if (!parse_keep(v, &cfg.keep_mode, &cfg.keep_depth)) {
        opt_err = "invalid -k value '" + v + "'";
        return false;
      }
      i += 2;
    } else if (t == "-D") {
      std::string v;
      if (!require_value(v)) {
        return false;
      }
      char * end = nullptr;
      const double d = std::strtod(v.c_str(), &end);
      if (end == v.c_str() || *end != '\0' || d < 0.0) {
        opt_err = "invalid -D value '" + v + "'";
        return false;
      }
      cfg.duration_s = d;
      i += 2;
    } else if (t == "-i") {
      std::string v;
      if (!require_value(v)) {
        return false;
      }
      int64_t n = 0;
      if (!parse_strict_int64(v, &n) || n < 0 || n > 232) {
        opt_err = "invalid -i value '" + v + "' (valid range 0-232)";
        return false;
      }
      cfg.domain_id = static_cast<int32_t>(n);
      i += 2;
    } else if (t == "-t") {
      std::string v;
      if (!require_value(v)) {
        return false;
      }
      cfg.topic = v;
      i += 2;
    } else if (t == "-E") {
      std::string v;
      if (!require_value(v)) {
        return false;
      }
      ExecutorKind e;
      if (!parse_executor(v, &e)) {
        opt_err = "invalid -E value '" + v + "'";
        return false;
      }
      cfg.executor = e;
      i += 2;
    } else {
      opt_err = "unknown option '" + t + "'";
      return false;
    }
    return true;
  };

  while (i < toks.size()) {
    const std::string & head = toks[i];
    if (!head.empty() && head[0] == '-') {
      std::string opt_err;
      if (!parse_option(opt_err)) {
        result.status = ParseResult::Status::kError;
        result.error = opt_err;
        return result;
      }
      // An option may set kHelp / kVersion; finish parsing the rest of
      // argv anyway (cheap) but return the status either way.
      if (
        result.status == ParseResult::Status::kHelp ||
        result.status == ParseResult::Status::kVersion) {
        return result;
      }
      continue;
    }
    if (!is_mode_keyword(head)) {
      result.status = ParseResult::Status::kError;
      result.error = "unexpected token '" + head + "' (expected option or mode)";
      return result;
    }
    const std::string keyword = toks[i++];
    if (keyword == "help") {
      result.status = ParseResult::Status::kHelp;
      return result;
    }
    if (keyword == "sanity") {
      // Equivalent to "ping 1Hz".
      ModeSpec m{};
      m.kind = ModeKind::kPing;
      m.ping.rate_hz = 1.0;
      cfg.modes.push_back(m);
      continue;
    }
    ModeSpec m{};
    std::string err;
    if (keyword == "pub") {
      m.kind = ModeKind::kPub;
      if (!parse_pub_subargs(toks, i, m.pub, err)) {
        result.status = ParseResult::Status::kError;
        result.error = err;
        return result;
      }
    } else if (keyword == "sub") {
      m.kind = ModeKind::kSub;
      if (!parse_sub_subargs(toks, i, m.sub, err)) {
        result.status = ParseResult::Status::kError;
        result.error = err;
        return result;
      }
    } else if (keyword == "ping") {
      m.kind = ModeKind::kPing;
      if (!parse_ping_subargs(toks, i, m.ping, err)) {
        result.status = ParseResult::Status::kError;
        result.error = err;
        return result;
      }
    } else if (keyword == "pong") {
      m.kind = ModeKind::kPong;
      if (!parse_pong_subargs(toks, i, m.pong, err)) {
        result.status = ParseResult::Status::kError;
        result.error = err;
        return result;
      }
    } else {
      result.status = ParseResult::Status::kError;
      result.error = "unknown mode '" + keyword + "'";
      return result;
    }
    cfg.modes.push_back(m);
  }

  if (cfg.modes.empty()) {
    result.status = ParseResult::Status::kError;
    result.error = "no mode specified (try 'help' or 'sanity')";
    return result;
  }

  return result;
}

std::string usage_text()
{
  std::ostringstream o;
  o << "ros2_perf help                       (this text; also -h / --help)\n"
       "ros2_perf --version                  (print version and exit)\n"
       "ros2_perf sanity                     (ping 1Hz)\n"
       "ros2_perf [OPTIONS] MODE...\n"
       "\n"
       "OPTIONS:\n"
       "  -t NAME       data topic name (default: /ros2_perf;\n"
       "                ping/pong derive /ros2_perf_ping, /ros2_perf_pong)\n"
       "  -u            best-effort instead of reliable\n"
       "  -k all|N      keep-all or keep-last-N history (default: 10)\n"
       "  -D DUR        run for at most DUR seconds (default: forever)\n"
       "  -L            allow same-process matching (intra-process comms)\n"
       "                (silently disabled for waitset-triggered modes)\n"
       "  -l            report one-way latency in subscriber mode\n"
       "  -i ID         use ROS_DOMAIN_ID = ID (0-232; set before init)\n"
       "  -E E          executor: single|multi|static (default: multi)\n"
       "\n"
       "MODE... is zero or more of:\n"
       "  pub  [R[Hz|kHz]|inf] [size S] [burst N]\n"
       "       Publish at rate R (default inf = as fast as possible). Sample\n"
       "       size S may use suffix B/k/M/kB/KiB/MB/MiB (k=1024, kB=1024,\n"
       "       KiB=1024); burst N samples per tick.\n"
       "  sub  [listener|waitset]\n"
       "       Subscribe. Trigger via Subscription callback (default) or via\n"
       "       a dedicated WaitSet thread.\n"
       "  ping [R[Hz]] [size S] [listener|waitset]\n"
       "       Round-trip latency initiator. If R is unset, sends next ping\n"
       "       as soon as the corresponding pong arrives.\n"
       "  pong [listener|waitset]\n"
       "       Echoes pings back as pongs.\n"
       "\n"
       "Anything after '--ros-args' is forwarded to rclcpp::init unchanged.\n"
       "\n"
       "EXAMPLES:\n"
       "  ros2_perf pub size 1k & ros2_perf -D 10 sub\n"
       "  ros2_perf ping 100Hz & ros2_perf pong\n"
       "  ros2_perf -L -D 10 pub sub\n";
  return o.str();
}

}  // namespace ros2_perf
