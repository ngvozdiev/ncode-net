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

  GraphLinkSet links_to_exlude = {L("A", "B")};
  ConstraintSet constraints;
  constraints.AddToExcludeLinks(&links_to_exlude);
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

  //  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
  //  ASSERT_EQ(P("[C->B]"), sub_graph.ShortestPath(N("C"), N("B")));
}

TEST_F(ThreeEdges, SubGraphOne) {
  DirectedGraph graph(&storage_);

  GraphLinkSet links_to_exlude = {L("A", "B")};
  ConstraintSet constraints;
  constraints.AddToExcludeLinks(&links_to_exlude);
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("B")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[C->B]"), sub_graph.ShortestPath(N("C"), N("B")));
}

TEST_F(ThreeEdges, SubGraphTwo) {
  DirectedGraph graph(&storage_);

  GraphLinkSet links_to_exlude = {L("B", "C")};
  ConstraintSet constraints;
  constraints.AddToExcludeLinks(&links_to_exlude);
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[A->B]"), sub_graph.ShortestPath(N("A"), N("B")));
  ASSERT_EQ(P("[C->B]"), sub_graph.ShortestPath(N("C"), N("B")));
}

TEST_F(ThreeEdges, SubGraphThree) {
  DirectedGraph graph(&storage_);

  GraphNodeSet nodes_to_exlude = {N("B")};
  ConstraintSet constraints;
  constraints.AddToExcludeNodes(&nodes_to_exlude);
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

  GraphNodeSet to_visit = {N("B")};
  ConstraintSet constraints;
  constraints.AddToVisitSet(&to_visit);

  SubGraph sub_graph(&graph, &constraints);
//  ASSERT_EQ(P("[A->B, B->C, C->D]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C")));
//  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("A")));
//  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintTwoNodes) {
  DirectedGraph graph(&storage_);

  GraphNodeSet to_visit = {N("B"), N("C")};
  ConstraintSet constraints;
  constraints.AddToVisitSet(&to_visit);

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->B, B->C, C->D]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("A")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintFull) {
  DirectedGraph graph(&storage_);

  // Any path through the graph is valid.
  GraphNodeSet to_visit = {N("A"), N("B"), N("C"), N("D")};
  ConstraintSet constraints;
  constraints.AddToVisitSet(&to_visit);

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[A->D]"), sub_graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[B->C]"), sub_graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[A->B, B->C]"), sub_graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("C"), N("A")));
}

TEST_F(FourEdges, ShortestPathVisitConstraintTwoSets) {
  DirectedGraph graph(&storage_);

  GraphNodeSet to_visit_one = {N("A"), N("D")};
  GraphNodeSet to_visit_two = {N("B"), N("C")};

  ConstraintSet constraints;
  constraints.AddToVisitSet(&to_visit_one);
  constraints.AddToVisitSet(&to_visit_two);

  SubGraph sub_graph(&graph, &constraints);
  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("D")));
}

TEST_F(FourEdges, SubGraph) {
  DirectedGraph graph(&storage_);

  GraphLinkSet links_to_exlude = {L("A", "D")};
  ConstraintSet constraints;
  constraints.AddToExcludeLinks(&links_to_exlude);
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[A->B, B->C, C->D]"), sub_graph.ShortestPath(N("A"), N("D")));
}

TEST_F(FourEdges, SubGraphTwo) {
  DirectedGraph graph(&storage_);

  GraphLinkSet links_to_exlude = {L("A", "B")};
  ConstraintSet constraints;
  constraints.AddToExcludeLinks(&links_to_exlude);
  SubGraph sub_graph(&graph, &constraints);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("B")));
}

class Braess : public ::testing::Test, public Base {
 protected:
  static PBNet GetPb() { return GenerateBraess(kBw); }

  Braess() : Base(GetPb()) {}
};

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
  GraphLinkSet to_avoid = {L("A", "B")};
  constraints.AddToExcludeLinks(&to_avoid);
  SubGraph sub_graph(&graph, &constraints);

  std::vector<LinkSequence> paths;
  sub_graph.Paths(N("A"), N("D"), [&paths](const LinkSequence& path) {
    paths.emplace_back(path);
  });

  ASSERT_EQ(1ul, paths.size());
  ASSERT_EQ(P("[A->C, C->D]"), paths[0]);
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
  GraphLinkSet to_avoid = {L("A", "C")};
  constraints.AddToExcludeLinks(&to_avoid);
  SubGraph sub_graph(&graph, &constraints);

  KShortestPathsGenerator ksp(N("A"), N("D"), &sub_graph);
  ASSERT_EQ(P("[A->B, B->D]"), ksp.KthShortestPath(0));
  ASSERT_EQ(P("[A->B, B->C, C->D]"), ksp.KthShortestPath(1));
  ASSERT_EQ(P("[]"), ksp.KthShortestPath(2));
}

}  // namespace
}  // namespace net
}  // namespace ncode
