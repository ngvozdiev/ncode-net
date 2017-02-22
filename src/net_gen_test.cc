#include "net_gen.h"

#include <google/protobuf/repeated_field.h>
#include <ncode/ncode_common/strutil.h>
#include <stddef.h>
#include <algorithm>
#include <set>
#include <string>

#include "gtest/gtest.h"
#include "net.pb.h"

namespace nc {
namespace net {

using namespace std::chrono;
static constexpr Bandwidth kBandwidth = Bandwidth::FromBitsPerSecond(10000);

TEST(HE, Generate) {
  PBNet net_pb = GenerateHE(kBandwidth, Delay(0), 1);
  // All links should be bidirectional -- 2x64 in total.
  ASSERT_EQ(112, net_pb.links_size());

  std::set<std::string> endpoints;
  for (const auto& link : net_pb.links()) {
    endpoints.insert(link.src());
    endpoints.insert(link.dst());
    ASSERT_EQ(kBandwidth.bps(), link.bandwidth_bps());

    // All links should have src and dst ports set to positive numbers.
    ASSERT_LT(0ul, link.src_port());
    ASSERT_LT(0ul, link.dst_port());
  }

  ASSERT_EQ(31ul, endpoints.size());
  ASSERT_EQ(3, net_pb.regions_size());

  size_t total = 0;
  for (const auto& region : net_pb.regions()) {
    total += region.nodes_size();
  }
  ASSERT_EQ(31ul, total);
}

TEST(NTT, Generate) {
  PBNet net_pb = GenerateNTT();
  ASSERT_EQ(176, net_pb.links_size());

  std::set<std::string> endpoints;
  for (const auto& link : net_pb.links()) {
    endpoints.insert(link.src());
    endpoints.insert(link.dst());

    // All links should have src and dst ports set to positive numbers.
    ASSERT_LT(0ul, link.src_port());
    ASSERT_LT(0ul, link.dst_port());
  }

  ASSERT_EQ(43ul, endpoints.size());
  size_t total = 0;
  for (const auto& region : net_pb.regions()) {
    total += region.nodes_size();
  }
  ASSERT_EQ(43ul, total);
}

TEST(HE, GenerateDelayAdd) {
  PBNet net_pb = GenerateHE(kBandwidth, Delay(0), 1);
  PBNet net_pb_plus = GenerateHE(kBandwidth, Delay(10000), 1);
  for (int i = 0; i < net_pb.links_size(); ++i) {
    const PBGraphLink& link = net_pb.links(i);
    const PBGraphLink& link_plus = net_pb_plus.links(i);
    ASSERT_NEAR(link.delay_sec(),
                std::max(0.000001, link_plus.delay_sec() - 0.010), 0.000001);
  }
}

TEST(HE, GenerateDelayMultiply) {
  PBNet net_pb = GenerateHE(kBandwidth, Delay(0), 1);
  PBNet net_pb_times_1000 = GenerateHE(kBandwidth, Delay(0), 1000);
  for (int i = 0; i < net_pb.links_size(); ++i) {
    const PBGraphLink& link = net_pb.links(i);
    const PBGraphLink& link_times_1000 = net_pb_times_1000.links(i);
    ASSERT_NEAR(link.delay_sec(),
                std::max(0.000001, link_times_1000.delay_sec() / 1000),
                0.000001);
  }
}

TEST(Ladder, NoLevels) {
  ASSERT_DEATH(GenerateLadder(0, kBandwidth, Delay(10000)), ".*");
}

TEST(Ladder, SingleLevel) {
  PBNet net_pb_ladder = GenerateLadder(1, kBandwidth, Delay(10000));
  PBNet net_pb = GenerateFullGraph(2, kBandwidth, Delay(10000));
  ASSERT_EQ(net_pb.DebugString(), net_pb_ladder.DebugString());
}

TEST(Ladder, MultiLevel) {
  PBNet net_pb = GenerateLadder(10, kBandwidth, Delay(10000));
  ASSERT_EQ((1 + 9 * 5) * 2, net_pb.links_size());

  std::set<std::string> nodes;
  for (const auto& link : net_pb.links()) {
    ASSERT_FALSE(link.src().empty());
    ASSERT_FALSE(link.dst().empty());

    nodes.emplace(link.src());
    nodes.emplace(link.dst());
  }

  ASSERT_EQ(20ul + 18ul, nodes.size());
}

TEST(Ladder, MultiLevelCentralBandwidth) {
  PBNet net_pb = GenerateLadder(10, kBandwidth, Delay(10000), 0.5);

  for (uint32_t i = 0; i < 10; ++i) {
    std::string src = StrCat("N", i * 4);
    std::string dst = StrCat("N", i * 4 + 1);
    PBGraphLink* link_pb = FindEdgeOrDie(src, dst, &net_pb);
    ASSERT_EQ(link_pb->bandwidth_bps(), kBandwidth.bps() * 0.5);
  }
}

TEST(Random, ZeroSize) {
  std::mt19937 random(1.0);

  PBNet net_pb = GenerateRandom(0, 1.0, Delay(10000), Delay(10000), kBandwidth,
                                kBandwidth, &random);
  ASSERT_TRUE(net_pb.links().empty());

  // Even with 0 probability the graph should be connected.
  net_pb = GenerateRandom(10, 0.0, Delay(10000), Delay(10000), kBandwidth,
                          kBandwidth, &random);
  ASSERT_EQ(18, net_pb.links_size());
}

TEST(Random, BadArgs) {
  std::mt19937 random(1.0);
  ASSERT_DEATH(GenerateRandom(0, 1.0, Delay(20000), Delay(10000),
                              Bandwidth::FromBitsPerSecond(1000),
                              Bandwidth::FromBitsPerSecond(1000), &random),
               ".*");
  ASSERT_DEATH(GenerateRandom(0, 1.0, Delay(10000), Delay(10000),
                              Bandwidth::FromBitsPerSecond(2000),
                              Bandwidth::FromBitsPerSecond(1000), &random),
               ".*");
}

TEST(Random, Random) {
  std::mt19937 random(1.0);
  PBNet net_pb = GenerateRandom(1000, 0.1, Delay(1000), Delay(3000),
                                Bandwidth::FromBitsPerSecond(1000),
                                Bandwidth::FromBitsPerSecond(3000), &random);

  // There should be more than 100000 links.
  ASSERT_LT(100000, net_pb.links_size());

  double delay_sum = 0;
  uint64_t bw_sum = 0;
  for (const auto& link : net_pb.links()) {
    delay_sum += link.delay_sec();
    bw_sum += link.bandwidth_bps();
  }

  double mean_delay = delay_sum / net_pb.links_size();
  ASSERT_NEAR(0.002, mean_delay, 0.001);

  double mean_bw = bw_sum / net_pb.links_size();
  ASSERT_NEAR(2000, mean_bw, 1000);
}

}  // namespace net
}  // namespace ncode
