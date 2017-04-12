#include "algorithm.h"

#include <ncode/ncode_common/perfect_hash.h>
#include <chrono>
#include <initializer_list>
#include <vector>

#include "gtest/gtest.h"
#include "net.pb.h"
#include "net_gen.h"

namespace nc {
namespace net {
namespace {

static constexpr Bandwidth kBw = Bandwidth::FromBitsPerSecond(100);

TEST(SimpleGraph, DoubleEdge) {
  PBNet net;
  AddEdgeToGraph("A", "B", Delay(100), kBw, &net);
  AddEdgeToGraph("A", "B", Delay(10), kBw, &net);

  GraphStorage graph_storage(net);
  DirectedGraph graph(&graph_storage);
  ASSERT_FALSE(graph.IsSimple());
}

class Base {
 protected:
  Base(const PBNet& net) : storage_(net) {}

  // Returns the path described by a string.
  LinkSequence P(const std::string& path_string) {
    const GraphPath* path = storage_.PathFromStringOrDie(path_string, 1);
    return path->link_sequence();
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
  static PBNet GetPb() {
    PBNet out;
    AddEdgeToGraph("A", "B", Delay(100), kBw, &out);
    return out;
  }

  SingleLink() : Base(GetPb()) {}
};

TEST_F(SingleLink, SubGraphNoExclusion) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[A->B]"), sub_graph.ShortestPath(N("A"), N("B")));
}

TEST_F(SingleLink, SubGraph) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "B")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("B")));
}

class ThreeEdges : public ::testing::Test, public Base {
 protected:
  static PBNet GetPb() {
    PBNet net;
    AddEdgeToGraph("A", "B", Delay(100), kBw, &net);
    AddEdgeToGraph("B", "C", Delay(100), kBw, &net);
    AddEdgeToGraph("C", "B", Delay(100), kBw, &net);
    return net;
  }

  ThreeEdges() : Base(GetPb()) {}
};

TEST_F(ThreeEdges, ShortestPath) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[C->B]"), sub_graph.ShortestPath(N("C"), N("B")));
}

TEST_F(ThreeEdges, SubGraphOne) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "B")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("B")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[C->B]"), sub_graph.ShortestPath(N("C"), N("B")));
}

TEST_F(ThreeEdges, SubGraphTwo) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("B", "C")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[A->B]"), sub_graph.ShortestPath(N("A"), N("B")));
  ASSERT_EQ(P("[C->B]"), sub_graph.ShortestPath(N("C"), N("B")));
}

TEST_F(ThreeEdges, SubGraphThree) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Nodes({N("B")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("B")));
}

class FourEdges : public ::testing::Test, public Base {
 protected:
  static PBNet GetPb() {
    PBNet net;
    AddEdgeToGraph("A", "B", Delay(100), kBw, &net);
    AddEdgeToGraph("B", "C", Delay(100), kBw, &net);
    AddEdgeToGraph("C", "D", Delay(100), kBw, &net);
    AddEdgeToGraph("A", "D", Delay(100), kBw, &net);
    return net;
  }

  FourEdges() : Base(GetPb()) {}
};

TEST_F(FourEdges, VisitConstraintsMet) {
  ConstraintSet constraints;
  ASSERT_TRUE(constraints.OrderOk(P("[A->D]").links(), &storage_));

  constraints.AddToVisitSet({N("B")});
  ASSERT_FALSE(constraints.OrderOk(P("[A->D]").links(), &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[A->B]").links(), &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[A->B, B->C]").links(), &storage_));

  constraints.AddToVisitSet({N("C")});
  ASSERT_FALSE(constraints.OrderOk(P("[A->D]").links(), &storage_));
  ASSERT_FALSE(constraints.OrderOk(P("[A->B]").links(), &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[A->B, B->C]").links(), &storage_));

  constraints.AddToVisitSet({N("D")});
  ASSERT_FALSE(constraints.OrderOk({}, &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[A->B, B->C, C->D]").links(), &storage_));
}

TEST_F(FourEdges, VisitConstraintsMetTwo) {
  ConstraintSet constraints;
  constraints.AddToVisitSet({N("B"), N("C")});

  ASSERT_FALSE(constraints.OrderOk(P("[A->D]").links(), &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[A->B]").links(), &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[A->B, B->C]").links(), &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[B->C]").links(), &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[A->B, B->C, C->D]").links(), &storage_));
}

TEST_F(FourEdges, VisitConstraintsMetThree) {
  ConstraintSet constraints;
  constraints.AddToVisitSet({N("A"), N("D")});
  constraints.AddToVisitSet({N("B"), N("C")});

  ASSERT_FALSE(constraints.OrderOk(P("[A->D]").links(), &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[A->B]").links(), &storage_));
  ASSERT_TRUE(constraints.OrderOk(P("[A->B, B->C]").links(), &storage_));
  ASSERT_FALSE(constraints.OrderOk(P("[B->C]").links(), &storage_));
  ASSERT_FALSE(constraints.OrderOk(P("[A->B, B->C, C->D]").links(), &storage_));
}

TEST_F(FourEdges, ShortestPath) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[A->D]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("A")));
}

TEST_F(FourEdges, ShortestPathVisitConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("B")});

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->B, B->C, C->D]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("A")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintTwoNodes) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("B"), N("C")});

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->B, B->C, C->D]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("A")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintFull) {
  DirectedGraph graph(&storage_);

  // Any path through the graph is valid.
  ConstraintSet constraints;
  constraints.AddToVisitSet({N("A"), N("B"), N("C"), N("D")});

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->D]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("A")));
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
  ASSERT_EQ(P("[A->B, B->C, C->D]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("D")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("A")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintSpecificTwo) {
  DirectedGraph graph(&storage_);

  // Only this path is valid.
  ConstraintSet constraints;
  constraints.AddToVisitSet({N("B")});
  constraints.AddToVisitSet({N("C")});
  constraints.AddToVisitSet({N("D")});

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->B, B->C, C->D]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[B->C, C->D]"), sub_graph.ShortestPath(N("B"), N("D")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("A")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintTwoSets) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("A"), N("D")});
  constraints.AddToVisitSet({N("B"), N("C")});

  SubGraph sub_graph(&graph, &constraints);

  // Empty because 'D' is in the source set.
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[A->B]"), sub_graph.ShortestPath(N("A"), N("B")));
}

TEST_F(FourEdges, SubGraph) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "D")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[A->B, B->C, C->D]"), sub_graph.ShortestPath(N("A"), N("D")));
}

TEST_F(FourEdges, SubGraphTwo) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "B")});
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("B")));
}

class Ring : public ::testing::Test, public Base {
 protected:
  static PBNet GetPb() {
    PBNet net;
    AddEdgeToGraph("A", "B", Delay(100), kBw, &net);
    AddEdgeToGraph("B", "C", Delay(100), kBw, &net);
    AddEdgeToGraph("C", "D", Delay(100), kBw, &net);
    AddEdgeToGraph("D", "A", Delay(100), kBw, &net);
    AddEdgeToGraph("B", "E", Delay(100), kBw, &net);
    AddEdgeToGraph("C", "E", Delay(100000), kBw, &net);
    return net;
  }

  Ring() : Base(GetPb()) {}
};

TEST_F(Ring, DuplicateLink) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("C")});

  SubGraph sub_graph(&graph, &constraints);

  // The shortest way to get to E from A via C would repeat A->B.
  ASSERT_EQ(P("[A->B, B->C, C->E]"), sub_graph.ShortestPath(N("A"), N("E")));
}

TEST_F(Ring, DuplicateLinkNoAvoid) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("C", "E")});
  constraints.AddToVisitSet({N("C")});

  SubGraph sub_graph(&graph, &constraints);

  // The shortest way to get to E from A via C would repeat A->B.
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("E")));
}

class Braess : public ::testing::Test, public Base {
 protected:
  static PBNet GetPb() { return GenerateBraess(kBw); }

  Braess() : Base(GetPb()) {}
};

TEST_F(Braess, ShortestPathVisitConstraintDstExclude) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Nodes({N("B")});
  constraints.AddToVisitSet({N("D")});

  SubGraph sub_graph(&graph, &constraints);

  // The shortest path from A to D is ACD which goes via the destination.
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("C")));
}

TEST_F(Braess, DFS) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  std::vector<LinkSequence> paths;
  sub_graph.Paths(N("A"), N("D"), [&paths](const LinkSequence& path) {
    paths.emplace_back(path);
  });

  std::sort(paths.begin(), paths.end());
  ASSERT_EQ(3ul, paths.size());
  ASSERT_EQ(P("[A->C, C->D]"), paths[0]);
  ASSERT_EQ(P("[A->B, B->D]"), paths[1]);
  ASSERT_EQ(P("[A->B, B->C, C->D]"), paths[2]);
}

TEST_F(Braess, DFSConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "B")});
  SubGraph sub_graph(&graph, &constraints);

  std::vector<LinkSequence> paths;
  sub_graph.Paths(N("A"), N("D"), [&paths](const LinkSequence& path) {
    paths.emplace_back(path);
  });

  ASSERT_EQ(1ul, paths.size());
  ASSERT_EQ(P("[A->C, C->D]"), paths[0]);
}

TEST_F(Braess, DFSVisitConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("C")});
  SubGraph sub_graph(&graph, &constraints);

  std::vector<LinkSequence> paths;
  sub_graph.Paths(N("A"), N("D"), [&paths](const LinkSequence& path) {
    paths.emplace_back(path);
  });

  std::sort(paths.begin(), paths.end());
  ASSERT_EQ(2ul, paths.size());
  ASSERT_EQ(P("[A->C, C->D]"), paths[0]);
  ASSERT_EQ(P("[A->B, B->C, C->D]"), paths[1]);
}

TEST_F(Braess, KSP) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("A"), N("D"), &sub_graph);
  ASSERT_EQ(P("[A->C, C->D]"), ksp.KthShortestPath(0));
  ASSERT_EQ(P("[A->B, B->D]"), ksp.KthShortestPath(1));
  ASSERT_EQ(P("[A->B, B->C, C->D]"), ksp.KthShortestPath(2));
  ASSERT_EQ(P("[]"), ksp.KthShortestPath(3));
}

TEST_F(Braess, KSPConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.Exclude().Links({L("A", "C")});
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("A"), N("D"), &sub_graph);
  ASSERT_EQ(P("[A->B, B->D]"), ksp.KthShortestPath(0));
  ASSERT_EQ(P("[A->B, B->C, C->D]"), ksp.KthShortestPath(1));
  ASSERT_EQ(P("[]"), ksp.KthShortestPath(2));
}

TEST_F(Braess, KSPVistConstraint) {
  DirectedGraph graph(&storage_);

  ConstraintSet constraints;
  constraints.AddToVisitSet({N("C")});
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("A"), N("D"), &sub_graph);
  ASSERT_EQ(P("[A->C, C->D]"), ksp.KthShortestPath(0));
  ASSERT_EQ(P("[A->B, B->C, C->D]"), ksp.KthShortestPath(1));
  ASSERT_EQ(P("[]"), ksp.KthShortestPath(2));
}

}  // namespace
}  // namespace net
}  // namespace ncode
