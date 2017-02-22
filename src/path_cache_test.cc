#include "path_cache.h"

#include <string>

#include "gtest/gtest.h"
#include "net.pb.h"
#include "net_gen.h"

namespace nc {
namespace net {
namespace {

using namespace std::chrono;

static constexpr net::Bandwidth kDefaultBw =
    net::Bandwidth::FromBitsPerSecond(10000);

class PathUtilsTest : public ::testing::Test {
 protected:
  PathUtilsTest() : graph_storage_(GenerateBraess(kDefaultBw)) {
    a_ = graph_storage_.NodeFromStringOrDie("A");
    b_ = graph_storage_.NodeFromStringOrDie("B");
    c_ = graph_storage_.NodeFromStringOrDie("C");
    d_ = graph_storage_.NodeFromStringOrDie("D");
  }

  LinkSequence GetPath(const std::string& path_string) {
    return graph_storage_.PathFromStringOrDie(path_string, 0)->link_sequence();
  }

  GraphNodeIndex a_;
  GraphNodeIndex b_;
  GraphNodeIndex c_;
  GraphNodeIndex d_;

  GraphStorage graph_storage_;
};

TEST_F(PathUtilsTest, ShortestPath) {
  PathCache cache(&graph_storage_);

  ASSERT_EQ(
      GetPath("[A->C, C->D]"),
      cache.NodePairCache(std::make_tuple(a_, d_, 0ul))->KthShortestPath(0));
  ASSERT_EQ(GetPath("[B->C]"), cache.NodePairCache(std::make_tuple(b_, c_, 0ul))
                                   ->KthShortestPath(0));
}

TEST_F(PathUtilsTest, KShortestPaths) {
  PathCache cache(&graph_storage_);

  std::vector<LinkSequence> model;
  std::vector<LinkSequence> output;

  NodePairPathCache* pair_cache =
      cache.NodePairCache(std::make_tuple(a_, d_, 0ul));

  model.emplace_back(GetPath("[A->C, C->D]"));
  output.emplace_back(pair_cache->KthShortestPath(0));
  ASSERT_EQ(model, output);

  model.emplace_back(GetPath("[A->B, B->D]"));
  output.emplace_back(pair_cache->KthShortestPath(1));
  ASSERT_EQ(model, output);

  model.emplace_back(GetPath("[A->B, B->C, C->D]"));
  output.emplace_back(pair_cache->KthShortestPath(2));
  ASSERT_EQ(model, output);

  ASSERT_TRUE(pair_cache->KthShortestPath(3).empty());
}

TEST_F(PathUtilsTest, Paths) {
  PathCache cache(&graph_storage_);
  NodePairPathCache* pair_cache =
      cache.NodePairCache(std::make_tuple(a_, d_, 0ul));

  size_t i = 0;
  std::vector<LinkSequence> model;
  std::vector<LinkSequence> output;

  model.emplace_back(GetPath("[A->C, C->D]"));
  for (const LinkSequence* link_sequence : pair_cache->Paths(i, &i, nullptr)) {
    output.emplace_back(*link_sequence);
  }

  ASSERT_EQ(model, output);
  ASSERT_EQ(1ul, i);

  model.emplace_back(GetPath("[A->B, B->D]"));
  for (const LinkSequence* link_sequence : pair_cache->Paths(i, &i, nullptr)) {
    output.emplace_back(*link_sequence);
  }

  ASSERT_EQ(model, output);
  ASSERT_EQ(2ul, i);
}

TEST_F(PathUtilsTest, PathsSkip) {
  PathCache cache(&graph_storage_);
  NodePairPathCache* pair_cache =
      cache.NodePairCache(std::make_tuple(a_, d_, 0ul));

  size_t i = 1;
  std::vector<LinkSequence> model;
  std::vector<LinkSequence> output;

  model.emplace_back(GetPath("[A->B, B->D]"));
  model.emplace_back(GetPath("[A->B, B->C, C->D]"));
  GraphLinkSet to_avoid = {graph_storage_.LinkOrDie("A", "C"),
                           graph_storage_.LinkOrDie("B", "D")};

  for (const LinkSequence* link_sequence :
       pair_cache->Paths(i, &i, &to_avoid)) {
    output.emplace_back(*link_sequence);
  }

  ASSERT_EQ(model, output);
  ASSERT_EQ(3ul, i);
  ASSERT_TRUE(pair_cache->Paths(i, &i, &to_avoid).empty());
  ASSERT_EQ(3ul, i);
}

}  // namespace
}  // namespace net
}  // namespace nc
