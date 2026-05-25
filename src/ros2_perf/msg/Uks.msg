# ros2_perf unkeyed sequence topic (Uks), modeled after ddsperf's KS topic.
#
# seq        -- monotonic sequence number; the LSB distinguishes
#               ping(1) from pong(0) when reused by the ping/pong modes.
# stamp_ns   -- publisher steady_clock timestamp in nanoseconds, used
#               by the one-way latency measurement (-l).
# baggage    -- variable-length payload used to control message size.
uint64 seq
uint64 stamp_ns
uint8[] baggage
