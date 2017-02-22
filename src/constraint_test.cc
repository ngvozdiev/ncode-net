#include "constraint.h"
#include "net_gen.h"
#include "gtest/gtest.h"

namespace nc {
namespace net {
namespace {

static constexpr Bandwidth kBw = Bandwidth::FromBitsPerSecond(100);

using namespace std::chrono;

class ConstraintTest : public ::testing::Test {
 protected:
  ConstraintTest()
      : path_storage_(GenerateBraess(kBw)), graph_(&path_storage_) {
    a_ = path_storage_.NodeFromStringOrDie("A");
    b_ = path_storage_.NodeFromStringOrDie("B");
    c_ = path_storage_.NodeFromStringOrDie("C");
    d_ = path_storage_.NodeFromStringOrDie("D");
  }

  net::LinkSequence GetPath(const std::string& path) {
    return path_storage_.PathFromStringOrDie(path, 0ul)->link_sequence();
  }

  GraphNodeIndex a_;
  GraphNodeIndex b_;
  GraphNodeIndex c_;
  GraphNodeIndex d_;

  GraphStorage path_storage_;
  DirectedGraph graph_;
};

TEST_F(ConstraintTest, Dummy) {
  auto dummy = DummyConstraint();
  ASSERT_TRUE(dummy->PathComplies({}));

  ASSERT_EQ(GetPath("[A->C, C->D]"),
            dummy->PathGenerator(graph_, a_, d_, nullptr)->NextPath());
  ASSERT_EQ(GetPath("[D->B, B->C]"),
            dummy->PathGenerator(graph_, d_, c_, nullptr)->NextPath());
}

TEST_F(ConstraintTest, ConjunctionAvoidOne) {
  GraphLinkIndex bc = path_storage_.LinkOrDie("B", "C");
  Conjunction conjunction({bc}, {});

  ASSERT_EQ(GetPath("[D->C]"),
            conjunction.PathGenerator(graph_, d_, c_, nullptr)->NextPath());
  ASSERT_EQ(GetPath("[B->A, A->C]"),
            conjunction.PathGenerator(graph_, b_, c_, nullptr)->NextPath());
}

TEST_F(ConstraintTest, ConjunctionVisitOne) {
  GraphLinkIndex dc = path_storage_.LinkOrDie("D", "C");
  Conjunction conjunction({}, {dc});

  ASSERT_EQ(GetPath("[D->C]"),
            conjunction.PathGenerator(graph_, d_, c_, nullptr)->NextPath());
  ASSERT_EQ(GetPath("[B->D, D->C, C->A]"),
            conjunction.PathGenerator(graph_, b_, a_, nullptr)->NextPath());
  ASSERT_EQ(LinkSequence(),
            conjunction.PathGenerator(graph_, a_, b_, nullptr)->NextPath());
}

}  // namespace
}  // namespace dfs
}  // namespace ncode
