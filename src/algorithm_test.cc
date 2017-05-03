#include "algorithm.h"

#include <ncode/ncode_common/perfect_hash.h>
#include <chrono>
#include <initializer_list>
#include <vector>

#include "gtest/gtest.h"
#include "net_gen.h"

namespace nc {
namespace net {
namespace {

static constexpr Bandwidth kBw = Bandwidth::FromBitsPerSecond(100);

TEST(SimpleGraph, DoubleEdge) {
  GraphBuilder builder;
  builder.AddLink({"A", "B", kBw, Delay(100)});
  builder.AddLink({"A", "B", kBw, Delay(10)});

  GraphStorage graph_storage(builder);
  DirectedGraph graph(&graph_storage);
  ASSERT_FALSE(graph.IsSimple());
}

class Base {
 protected:
  Base(const GraphBuilder& builder) : storage_(builder) {}

  // Returns the path described by a string.
  Links P(const std::string& path_string) {
    return storage_.WalkFromStringOrDie(path_string)->links();
  }

  // Returns a node.
  GraphNodeIndex N(const std::string& node) {
    return storage_.NodeFromStringOrDie(node);
  }

  // Returns a link.
  GraphLinkIndex L(const std::string& src, const std::string& dst) {
    return storage_.LinkOrDie(src, dst);
  }

  GraphStorage storage_;
};

class SingleLink : public ::testing::Test, public Base {
 protected:
  static GraphBuilder GetBuilder() {
    GraphBuilder out;
    out.AddLink({"A", "B", kBw, Delay(100)});
    return out;
  }

  SingleLink() : Base(GetBuilder()) {}
};

TEST_F(SingleLink, SubGraphNoExclusion) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_FALSE(sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[A->B]"), sub_graph.ShortestPath(N("A"), N("B"))->links());
}

TEST_F(SingleLink, SubGraph) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "B")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_FALSE(sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("A"), N("B")));
}

class ThreeEdges : public ::testing::Test, public Base {
 protected:
  static GraphBuilder GetBuilder() {
    GraphBuilder out;
    out.AddLink({"A", "B", kBw, Delay(100)});
    out.AddLink({"B", "C", kBw, Delay(100)});
    out.AddLink({"C", "B", kBw, Delay(100)});
    return out;
  }

  ThreeEdges() : Base(GetBuilder()) {}
};

TEST_F(ThreeEdges, ShortestPath) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_FALSE(sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C"))->links());
  ASSERT_EQ(P("[C->B]"), sub_graph.ShortestPath(N("C"), N("B"))->links());
}

TEST_F(ThreeEdges, SubGraphOne) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "B")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_FALSE(sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("A"), N("B")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C"))->links());
  ASSERT_EQ(P("[C->B]"), sub_graph.ShortestPath(N("C"), N("B"))->links());
}

TEST_F(ThreeEdges, SubGraphTwo) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("B", "C")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_FALSE(sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[A->B]"), sub_graph.ShortestPath(N("A"), N("B"))->links());
  ASSERT_EQ(P("[C->B]"), sub_graph.ShortestPath(N("C"), N("B"))->links());
}

TEST_F(ThreeEdges, SubGraphThree) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Nodes({N("B")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_FALSE(sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("C"), N("B")));
}

class FourEdges : public ::testing::Test, public Base {
 protected:
  static GraphBuilder GetBuilder() {
    GraphBuilder out;
    out.AddLink({"A", "B", kBw, Delay(100)});
    out.AddLink({"B", "C", kBw, Delay(100)});
    out.AddLink({"C", "D", kBw, Delay(100)});
    out.AddLink({"A", "D", kBw, Delay(100)});
    return out;
  }

  FourEdges() : Base(GetBuilder()) {}
};

TEST_F(FourEdges, VisitConstraintsMet) {
  ConstraintSet constraints;
  ASSERT_EQ(0ul, constraints.MinVisit(P("[A->D]"), storage_));

  constraints.AddToVisitSet({N("B")});
  ASSERT_EQ(0ul, constraints.MinVisit(P("[A->D]"), storage_));
  ASSERT_EQ(1ul, constraints.MinVisit(P("[A->B]"), storage_));
  ASSERT_EQ(1ul, constraints.MinVisit(P("[A->B, B->C]"), storage_));

  constraints.AddToVisitSet({N("C")});
  ASSERT_EQ(0ul, constraints.MinVisit(P("[A->D]"), storage_));
  ASSERT_EQ(1ul, constraints.MinVisit(P("[A->B]"), storage_));
  ASSERT_EQ(2ul, constraints.MinVisit(P("[A->B, B->C]"), storage_));

  constraints.AddToVisitSet({N("D")});
  ASSERT_EQ(0ul, constraints.MinVisit({}, storage_));
  ASSERT_EQ(1ul, constraints.MinVisit(P("[A->B]"), storage_));
  ASSERT_EQ(2ul, constraints.MinVisit(P("[A->B, B->C]"), storage_));
  ASSERT_EQ(3ul, constraints.MinVisit(P("[A->B, B->C, C->D]"), storage_));
}

TEST_F(FourEdges, VisitConstraintsMetTwo) {
  ConstraintSet constraints;
  constraints.AddToVisitSet({N("B"), N("C")});

  ASSERT_EQ(0ul, constraints.MinVisit(P("[A->D]"), storage_));
  ASSERT_EQ(1ul, constraints.MinVisit(P("[A->B]"), storage_));
  ASSERT_EQ(1ul, constraints.MinVisit(P("[A->B, B->C]"), storage_));
  ASSERT_EQ(1ul, constraints.MinVisit(P("[B->C]"), storage_));
  ASSERT_EQ(1ul, constraints.MinVisit(P("[A->B, B->C, C->D]"), storage_));
}

TEST_F(FourEdges, VisitConstraintsMetThree) {
  ConstraintSet constraints;
  constraints.AddToVisitSet({N("A")});
  constraints.AddToVisitSet({N("C")});

  ASSERT_EQ(1ul, constraints.MinVisit(P("[A->D]"), storage_));
  ASSERT_EQ(2ul, constraints.MinVisit(P("[A->B]"), storage_));
  ASSERT_EQ(2ul, constraints.MinVisit(P("[A->B, B->C]"), storage_));
  ASSERT_EQ(0ul, constraints.MinVisit(P("[B->C]"), storage_));
  ASSERT_EQ(0ul, constraints.MinVisit(P("[A->B, B->C, C->D]"), storage_));
}

TEST_F(FourEdges, ShortestPath) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[A->D]"), sub_graph.ShortestPath(N("A"), N("D"))->links());
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C"))->links());
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C"))->links());
  ASSERT_FALSE(sub_graph.ShortestPath(N("C"), N("A")));
}

TEST_F(FourEdges, ShortestPathVisitConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("B")});

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->B, B->C, C->D]"),
            sub_graph.ShortestPath(N("A"), N("D"))->links());
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C"))->links());
  ASSERT_FALSE(sub_graph.ShortestPath(N("C"), N("A")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C"))->links());
}

TEST_F(FourEdges, ShortestPathVisitConstraintTwoNodes) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("B"), N("C")});

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->B, B->C, C->D]"),
            sub_graph.ShortestPath(N("A"), N("D"))->links());
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C"))->links());
  ASSERT_FALSE(sub_graph.ShortestPath(N("C"), N("A")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C"))->links());
}

TEST_F(FourEdges, ShortestPathVisitConstraintFull) {
  DirectedGraph graph(&storage_);

  // Any path through the graph is valid.
  ConstraintSet constraints;
  constraints.AddToVisitSet({N("A"), N("B"), N("C"), N("D")});

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->D]"), sub_graph.ShortestPath(N("A"), N("D"))->links());
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C"))->links());
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C"))->links());
  ASSERT_FALSE(sub_graph.ShortestPath(N("C"), N("A")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintSpecific) {
  DirectedGraph graph(&storage_);

  // Only this path is valid.
  ConstraintSet constraints;
  constraints.AddToVisitSet({N("A")});
  constraints.AddToVisitSet({N("B")});
  constraints.AddToVisitSet({N("C")});
  constraints.AddToVisitSet({N("D")});

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->B, B->C, C->D]"),
            sub_graph.ShortestPath(N("A"), N("D"))->links());
  ASSERT_FALSE(sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("B"), N("D")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("C"), N("A")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintSpecificTwo) {
  DirectedGraph graph(&storage_);

  // Only this path is valid.
  ConstraintSet constraints;
  constraints.AddToVisitSet({N("B")});
  constraints.AddToVisitSet({N("C")});
  constraints.AddToVisitSet({N("D")});

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->B, B->C, C->D]"),
            sub_graph.ShortestPath(N("A"), N("D"))->links());
  ASSERT_EQ(P("[B->C, C->D]"), sub_graph.ShortestPath(N("B"), N("D"))->links());
  ASSERT_FALSE(sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_FALSE(sub_graph.ShortestPath(N("C"), N("A")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintTwoSets) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("A"), N("D")});
  constraints.AddToVisitSet({N("B"), N("C")});

  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[A->B, B->C, C->D]"),
            sub_graph.ShortestPath(N("A"), N("D"))->links());
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C"))->links());
  ASSERT_EQ(P("[A->B]"), sub_graph.ShortestPath(N("A"), N("B"))->links());
}

TEST_F(FourEdges, SubGraph) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "D")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[A->B, B->C, C->D]"),
            sub_graph.ShortestPath(N("A"), N("D"))->links());
}

TEST_F(FourEdges, SubGraphTwo) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "B")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_FALSE(sub_graph.ShortestPath(N("A"), N("B")));
}

TEST_F(FourEdges, KSPImpossible) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("D"), N("A"), sub_graph);
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(0));
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(1));
}

TEST_F(FourEdges, KSPConstraintImpossible) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("A")});
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("C"), N("D"), sub_graph);
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(0));
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(1));
}

class Ring : public ::testing::Test, public Base {
 protected:
  static GraphBuilder GetBuilder() {
    GraphBuilder out;
    out.AddLink({"A", "B", kBw, Delay(100)});
    out.AddLink({"B", "C", kBw, Delay(100)});
    out.AddLink({"C", "D", kBw, Delay(100)});
    out.AddLink({"D", "A", kBw, Delay(100)});
    out.AddLink({"B", "E", kBw, Delay(100)});
    out.AddLink({"C", "E", kBw, Delay(100000)});
    return out;
  }

  Ring() : Base(GetBuilder()) {}
};

TEST_F(Ring, DuplicateLink) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("C")});

  SubGraph sub_graph(&graph, &constraints);

  // The shortest way to get to E from A via C would repeat A->B.
  ASSERT_EQ(P("[A->B, B->C, C->D, D->A, A->B, B->E]"),
            sub_graph.ShortestPath(N("A"), N("E"))->links());
}

TEST_F(Ring, DuplicateLinkNoAvoid) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("C", "D")});
  constraints.AddToVisitSet({N("C")});

  SubGraph sub_graph(&graph, &constraints);

  // The second best path.
  ASSERT_EQ(P("[A->B, B->C, C->E]"),
            sub_graph.ShortestPath(N("A"), N("E"))->links());
}

class Braess : public ::testing::Test, public Base {
 protected:
  Braess() : Base(GenerateBraess(kBw)) {}
};

TEST_F(Braess, ShortestPathVisitConstraintDstExclude) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Nodes({N("B")});
  constraints.AddToVisitSet({N("D")});

  SubGraph sub_graph(&graph, &constraints);

  // The shortest path from A to D is ACD which goes via the destination.
  ASSERT_FALSE(sub_graph.ShortestPath(N("A"), N("C")));
}

TEST_F(Braess, DFS) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  std::vector<std::unique_ptr<Walk>> paths;
  sub_graph.Paths(N("A"), N("D"), [&paths](std::unique_ptr<Walk> path) {
    paths.emplace_back(std::move(path));
  }, {});

  std::sort(paths.begin(), paths.end(), [](const std::unique_ptr<Walk>& lhs,
                                           const std::unique_ptr<Walk>& rhs) {
    return lhs->delay() < rhs->delay();
  });
  ASSERT_EQ(3ul, paths.size());
  ASSERT_EQ(P("[A->C, C->D]"), paths[0]->links());
  ASSERT_EQ(P("[A->B, B->D]"), paths[1]->links());
  ASSERT_EQ(P("[A->B, B->C, C->D]"), paths[2]->links());
}

TEST_F(Braess, DFSNotSimple) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  DFSConfig dfs_config;
  dfs_config.simple = false;

  std::vector<std::unique_ptr<Walk>> paths;
  sub_graph.Paths(N("A"), N("D"), [&paths](std::unique_ptr<Walk> path) {
    paths.emplace_back(std::move(path));
  }, dfs_config);

  std::sort(paths.begin(), paths.end(), [](const std::unique_ptr<Walk>& lhs,
                                           const std::unique_ptr<Walk>& rhs) {
    return lhs->delay() < rhs->delay();
  });
  ASSERT_EQ(7ul, paths.size());
  ASSERT_EQ(P("[A->C, C->D]"), paths[0]->links());
  ASSERT_EQ(P("[A->B, B->D]"), paths[1]->links());
  ASSERT_EQ(P("[A->B, B->C, C->D]"), paths[2]->links());
  ASSERT_EQ(P("[A->C, C->A, A->B, B->D]"), paths[3]->links());
  ASSERT_EQ(P("[A->B, B->C, C->A, A->C, C->D]"), paths[4]->links());
  ASSERT_EQ(P("[A->C, C->A, A->B, B->C, C->D]"), paths[5]->links());
  ASSERT_EQ(P("[A->B, B->A, A->C, C->D]"), paths[6]->links());
}

TEST_F(Braess, DFSConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "B")});
  SubGraph sub_graph(&graph, &constraints);

  std::vector<std::unique_ptr<Walk>> paths;
  sub_graph.Paths(N("A"), N("D"), [&paths](std::unique_ptr<Walk> path) {
    paths.emplace_back(std::move(path));
  }, {});

  ASSERT_EQ(1ul, paths.size());
  ASSERT_EQ(P("[A->C, C->D]"), paths[0]->links());
}

TEST_F(Braess, DFSVisitConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("C")});
  SubGraph sub_graph(&graph, &constraints);

  std::vector<std::unique_ptr<Walk>> paths;
  sub_graph.Paths(N("A"), N("D"), [&paths](std::unique_ptr<Walk> path) {
    paths.emplace_back(std::move(path));
  }, {});

  std::sort(paths.begin(), paths.end(), [](const std::unique_ptr<Walk>& lhs,
                                           const std::unique_ptr<Walk>& rhs) {
    return lhs->delay() < rhs->delay();
  });
  ASSERT_EQ(2ul, paths.size());
  ASSERT_EQ(P("[A->C, C->D]"), paths[0]->links());
  ASSERT_EQ(P("[A->B, B->C, C->D]"), paths[1]->links());
}

TEST_F(Braess, KSP) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("A"), N("D"), sub_graph);
  ASSERT_EQ(P("[A->C, C->D]"), ksp.KthShortestPathOrNull(0)->links());
  ASSERT_EQ(P("[A->B, B->D]"), ksp.KthShortestPathOrNull(1)->links());
  ASSERT_EQ(P("[A->B, B->C, C->D]"), ksp.KthShortestPathOrNull(2)->links());
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(3));
}

TEST_F(Braess, KSPConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "C")});
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("A"), N("D"), sub_graph);
  ASSERT_EQ(P("[A->B, B->D]"), ksp.KthShortestPathOrNull(0)->links());
  ASSERT_EQ(P("[A->B, B->C, C->D]"), ksp.KthShortestPathOrNull(1)->links());
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(2));
}

TEST_F(Braess, KSPVistConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("C")});
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("A"), N("D"), sub_graph);
  ASSERT_EQ(P("[A->C, C->D]"), ksp.KthShortestPathOrNull(0)->links());
  ASSERT_EQ(P("[A->B, B->C, C->D]"), ksp.KthShortestPathOrNull(1)->links());
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(2));
}

TEST_F(Braess, KSPVisitConstraintNonSimple) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("C")});
  constraints.AddToVisitSet({N("B")});
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("A"), N("D"), sub_graph);
  ASSERT_EQ(P("[A->C, C->A, A->B, B->D]"),
            ksp.KthShortestPathOrNull(0)->links());
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(1));
}

TEST_F(Braess, KSPVistConstraintDisjunct) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints_one;
  constraints_one.AddToVisitSet({N("C")});
  SubGraph sub_graph_one(&graph, &constraints_one);

  ConstraintSet constraints_two;
  constraints_two.AddToVisitSet({N("C")});
  SubGraph sub_graph_two(&graph, &constraints_two);

  // We are OR-ing two identical constraints---the result should be the same as
  // with a single constraint.
  DisjunctKShortestPathsGenerator ksp(N("A"), N("D"),
                                      {&sub_graph_one, &sub_graph_two});
  ASSERT_EQ(P("[A->C, C->D]"), ksp.KthShortestPathOrNull(0)->links());
  ASSERT_EQ(P("[A->B, B->C, C->D]"), ksp.KthShortestPathOrNull(1)->links());
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(2));
}

TEST_F(Braess, KSPVistConstraintDisjunctTwo) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints_one;
  constraints_one.AddToVisitSet({N("C")});
  SubGraph sub_graph_one(&graph, &constraints_one);

  ConstraintSet constraints_two;
  constraints_two.AddToVisitSet({N("B")});
  SubGraph sub_graph_two(&graph, &constraints_two);

  // Going through either C or B should yield all possible paths.
  DisjunctKShortestPathsGenerator ksp(N("A"), N("D"),
                                      {&sub_graph_one, &sub_graph_two});
  ASSERT_EQ(P("[A->C, C->D]"), ksp.KthShortestPathOrNull(0)->links());
  ASSERT_EQ(P("[A->B, B->D]"), ksp.KthShortestPathOrNull(1)->links());
  ASSERT_EQ(P("[A->B, B->C, C->D]"), ksp.KthShortestPathOrNull(2)->links());
  ASSERT_EQ(nullptr, ksp.KthShortestPathOrNull(3));
}

}  // namespace
}  // namespace net
}  // namespace ncode
