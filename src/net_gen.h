#ifndef NCODE_NET_GEN_H
#define NCODE_NET_GEN_H

#include <chrono>
#include <cstdint>
#include <random>
#include <vector>

#include "net_common.h"

namespace nc {
namespace net {

// Generates a topology similar to HE's backbone. The topology will have 31
// devices and 56 bidirectional links. All links in the network will have a
// delay of max(1, d * delay_multiply + delay_add) where d is the speed of light
// in fiber delay.
GraphBuilder GenerateHE(Bandwidth bw, Delay delay_add = Delay(0),
                        double delay_multiply = 1.0);

// Generates a topology similar to NTT's backbone. Unlike HE above this function
// will also populate real-world bandiwdth values.
GraphBuilder GenerateNTT(Delay delay_add = Delay(0),
                         double delay_multiply = 1.0,
                         Bandwidth bw_add = Bandwidth::FromBitsPerSecond(0),
                         double bw_multiply = 1.0);

// Generates a full graph of a given size. Each node will be named Ni for i in
// [0, size).
GraphBuilder GenerateFullGraph(uint32_t size, Bandwidth bw, Delay delay);

// Generates a ladder-like topology. All links have the same rate and delay.
// Nodes will be named Ni for i in [0, ...]. Odd nodes will be on one side
// of the ladder, even nodes will be on the other. The middle links will have
// their capacity multiplied by the central_rate_multiplier argument. If the
// central_delays argument is not empty it should contain as many elements as
// levels. Each element will be the delay of the middle connecting link for the
// respective level. With 1 level this is a line, 2 levels a hexagon, 3 levels
// two hexagons attached etc.
GraphBuilder GenerateLadder(size_t levels, Bandwidth rate_bps, Delay delay,
                            double central_rate_multiplier = 1.0,
                            const std::vector<Delay>& central_delays = {});

// Generates a random graph with N nodes. Each of the n * (n - 1) edges has a
// uniform probability of edge_prob of being part of the graph. If this is set
// to 1.0 the graph will be a full graph. Each edge's delay and bandwidth will
// be picked at uniform from the given ranges. Nodes will be named Ni for i in
// [0, N).
GraphBuilder GenerateRandom(size_t n, double edge_prob, Delay delay_min,
                            Delay delay_max, Bandwidth bw_bps_min,
                            Bandwidth bw_bps_max, std::mt19937* generator);

// A simple Braess paradox-like topology used for testing.
GraphBuilder GenerateBraess(Bandwidth bw);

}  // namespace net
}  // namespace ncode
#endif
