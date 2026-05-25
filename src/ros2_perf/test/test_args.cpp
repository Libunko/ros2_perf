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

#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "ros2_perf/args.hpp"

namespace
{

// Helper: build argv from a vector of literals, run parse_args, return
// the ParseResult along with the consumed-argc value the parser sets.
struct ParseOut
{
  ros2_perf::ParseResult pr;
  int consumed = 0;
  int argc = 0;
};

ParseOut run_parse(const std::vector<std::string> & args)
{
  std::vector<std::vector<char>> bufs;
  bufs.reserve(args.size() + 1);
  std::vector<char *> argv;
  argv.reserve(args.size() + 1);

  auto push = [&](const std::string & s) {
    bufs.emplace_back(s.begin(), s.end());
    bufs.back().push_back('\0');
    argv.push_back(bufs.back().data());
  };

  push("ros2_perf");
  for (const auto & a : args) {
    push(a);
  }

  ParseOut out;
  out.argc = static_cast<int>(argv.size());
  out.pr = ros2_perf::parse_args(out.argc, argv.data(), &out.consumed);
  return out;
}

}  // namespace

TEST(Args, Help)
{
  auto o = run_parse({"help"});
  EXPECT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kHelp);
}

TEST(Args, HelpShortFlag)
{
  auto o = run_parse({"-h"});
  EXPECT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kHelp);
}

TEST(Args, HelpLongFlag)
{
  auto o = run_parse({"--help"});
  EXPECT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kHelp);
}

TEST(Args, Version)
{
  auto o = run_parse({"--version"});
  EXPECT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kVersion);
}

TEST(Args, NoMode)
{
  auto o = run_parse({});
  EXPECT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kError);
}

TEST(Args, UnknownOption)
{
  auto o = run_parse({"-Z"});
  EXPECT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kError);
}

TEST(Args, Sanity)
{
  auto o = run_parse({"sanity"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  ASSERT_EQ(o.pr.cfg.modes.size(), 1u);
  EXPECT_EQ(o.pr.cfg.modes[0].kind, ros2_perf::ModeKind::kPing);
  EXPECT_DOUBLE_EQ(o.pr.cfg.modes[0].ping.rate_hz, 1.0);
}

TEST(Args, PubBasic)
{
  auto o = run_parse({"pub"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  ASSERT_EQ(o.pr.cfg.modes.size(), 1u);
  EXPECT_EQ(o.pr.cfg.modes[0].kind, ros2_perf::ModeKind::kPub);
  EXPECT_DOUBLE_EQ(o.pr.cfg.modes[0].pub.rate_hz, 0.0);  // inf default
  EXPECT_EQ(o.pr.cfg.modes[0].pub.burst, 1u);
}

TEST(Args, PubWithRate)
{
  auto o = run_parse({"pub", "100Hz"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_DOUBLE_EQ(o.pr.cfg.modes[0].pub.rate_hz, 100.0);
}

TEST(Args, PubKHz)
{
  auto o = run_parse({"pub", "2kHz"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_DOUBLE_EQ(o.pr.cfg.modes[0].pub.rate_hz, 2000.0);
}

TEST(Args, PubInf)
{
  auto o = run_parse({"pub", "inf"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_DOUBLE_EQ(o.pr.cfg.modes[0].pub.rate_hz, 0.0);
}

TEST(Args, PubSizeUnits)
{
  auto o = run_parse({"pub", "size", "1k"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.modes[0].pub.size_bytes, 1024u);

  o = run_parse({"pub", "size", "2KiB"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.modes[0].pub.size_bytes, 2048u);

  o = run_parse({"pub", "size", "1M"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.modes[0].pub.size_bytes, 1048576u);

  o = run_parse({"pub", "size", "256B"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.modes[0].pub.size_bytes, 256u);
}

TEST(Args, PubBurst)
{
  auto o = run_parse({"pub", "burst", "8"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.modes[0].pub.burst, 8u);
}

TEST(Args, PubInvalidSize)
{
  auto o = run_parse({"pub", "size", "abc"});
  EXPECT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kError);
}

TEST(Args, PubGarbageInNumberIsRejected)
{
  // strtod cleanly rejects "+-1" (the new parser delegates entirely to
  // strtod so we don't accept that any more).
  auto o = run_parse({"pub", "+-1Hz"});
  EXPECT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kError);
}

TEST(Args, SubWaitSet)
{
  auto o = run_parse({"sub", "waitset"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.modes[0].kind, ros2_perf::ModeKind::kSub);
  EXPECT_EQ(o.pr.cfg.modes[0].sub.trigger, ros2_perf::Trigger::kWaitSet);
}

TEST(Args, PingAllArgs)
{
  auto o = run_parse({"ping", "100Hz", "size", "64", "listener"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_DOUBLE_EQ(o.pr.cfg.modes[0].ping.rate_hz, 100.0);
  EXPECT_EQ(o.pr.cfg.modes[0].ping.size_bytes, 64u);
  EXPECT_EQ(o.pr.cfg.modes[0].ping.trigger, ros2_perf::Trigger::kListener);
}

TEST(Args, OptionsAndModesInterleave)
{
  auto o = run_parse({"-L", "pub", "-D", "5", "sub"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_TRUE(o.pr.cfg.intra_process);
  EXPECT_DOUBLE_EQ(o.pr.cfg.duration_s, 5.0);
  ASSERT_EQ(o.pr.cfg.modes.size(), 2u);
  EXPECT_EQ(o.pr.cfg.modes[0].kind, ros2_perf::ModeKind::kPub);
  EXPECT_EQ(o.pr.cfg.modes[1].kind, ros2_perf::ModeKind::kSub);
}

TEST(Args, BestEffortAndKeepAll)
{
  auto o = run_parse({"-u", "-k", "all", "sub"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_TRUE(o.pr.cfg.best_effort);
  EXPECT_EQ(o.pr.cfg.keep_mode, ros2_perf::KeepMode::kKeepAll);
}

TEST(Args, KeepLastN)
{
  auto o = run_parse({"-k", "100", "sub"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.keep_mode, ros2_perf::KeepMode::kKeepLast);
  EXPECT_EQ(o.pr.cfg.keep_depth, 100u);
}

TEST(Args, DomainId)
{
  auto o = run_parse({"-i", "42", "sub"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.domain_id, 42);
}

TEST(Args, DomainIdOutOfRange)
{
  EXPECT_EQ(run_parse({"-i", "-1", "sub"}).pr.status, ros2_perf::ParseResult::Status::kError);
  EXPECT_EQ(run_parse({"-i", "233", "sub"}).pr.status, ros2_perf::ParseResult::Status::kError);
}

TEST(Args, ExecutorSelection)
{
  auto o = run_parse({"-E", "static", "sub"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.executor, ros2_perf::ExecutorKind::kStatic);
}

TEST(Args, TopicOverride)
{
  auto o = run_parse({"-t", "/bench", "sub"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  EXPECT_EQ(o.pr.cfg.topic, "/bench");
}

TEST(Args, RosArgsCutoff)
{
  // parse_args must stop at --ros-args and report it via consumed.
  auto o = run_parse({"pub", "--ros-args", "-p", "foo:=bar"});
  ASSERT_EQ(o.pr.status, ros2_perf::ParseResult::Status::kOk);
  // consumed == index of "--ros-args" in argv. argv[0]=ros2_perf,
  // argv[1]=pub, argv[2]=--ros-args -> consumed == 2.
  EXPECT_EQ(o.consumed, 2);
}

TEST(Args, UsageTextNotEmpty) { EXPECT_FALSE(ros2_perf::usage_text().empty()); }
