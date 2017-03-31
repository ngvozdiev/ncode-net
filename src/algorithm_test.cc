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
    return storage_.NodeFromStringOrDie("A");
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

TEST_F(SingleLink, ShortestPath) {
  DirectedGraph graph(&storage_);
  ASSERT_EQ(P("[]"), graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[A->B]"), graph.ShortestPath(N("A"), N("B")));
}

TEST_F(SingleLink, SubGraphNoExclusion) {
  DirectedGraph graph(&storage_);

  ExclusionSet to_exclude;
  SubGraph sub_graph(&graph, &to_exclude);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[A->B]"), sub_graph.ShortestPath(N("A"), N("B")));
}

TEST_F(SingleLink, SubGraph) {
  DirectedGraph graph(&storage_);

  GraphLinkSet links_to_exlude = {L("A", "B")};
  ExclusionSet to_exclude;
  to_exclude.AddToExcludeLinks(&links_to_exlude);
  SubGraph sub_graph(&graph, &to_exclude);

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
  ASSERT_EQ(P("[]"), graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[A->B, B->C]"), graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[C->B]"), graph.ShortestPath(N("C"), N("B")));
}

TEST_F(ThreeEdges, SubGraphOne) {
  DirectedGraph graph(&storage_);

  GraphLinkSet links_to_exlude = {L("A", "B")};
  ExclusionSet to_exclude;
  to_exclude.AddToExcludeLinks(&links_to_exlude);
  SubGraph sub_graph(&graph, &to_exclude);

  ASSERT_EQ(P("[]"), graph.ShortestPath(N("B"), N("A")));
  ASSERT_EQ(P("[]"), graph.ShortestPath(N("A"), N("B")));
  ASSERT_EQ(P("[B->C]"), graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[C->B]"), graph.ShortestPath(N("C"), N("B")));
}

TEST_F(ThreeEdges, SubGraphTwo) {
  DirectedGraph graph(&storage_);

  GraphLinkSet links_to_exlude = {L("B", "C")};
  ExclusionSet to_exclude;
  to_exclude.AddToExcludeLinks(&links_to_exlude);
  SubGraph sub_graph(&graph, &to_exclude);

  ASSERT_EQ(P("[]"), graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[A->B]"), graph.ShortestPath(N("A"), N("B")));
  ASSERT_EQ(P("[C->B]"), graph.ShortestPath(N("C"), N("B")));
}

TEST_F(ThreeEdges, SubGraphThree) {
  DirectedGraph graph(&storage_);

  GraphNodeSet nodes_to_exlude = {N("B")};
  ExclusionSet to_exclude;
  to_exclude.AddToExcludeNodes(&nodes_to_exlude);
  SubGraph sub_graph(&graph, &to_exclude);

  ASSERT_EQ(P("[]"), graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[]"), graph.ShortestPath(N("C"), N("B")));
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

  ASSERT_EQ(P("[A->D]"), graph.ShortestPath(N("A"), N("D")));
  ASSERT_EQ(P("[B->C]"), graph.ShortestPath(N("B"), N("C")));
  ASSERT_EQ(P("[A->B, B->C]"), graph.ShortestPath(N("A"), N("C")));
  ASSERT_EQ(P("[]"), graph.ShortestPath(N("C"), N("A")));
}

TEST_F(FourEdges, SubGraph) {
  DirectedGraph graph(&storage_);

  GraphLinkSet links_to_exlude = {L("A", "D")};
  ExclusionSet to_exclude;
  to_exclude.AddToExcludeLinks(&links_to_exlude);
  SubGraph sub_graph(&graph, &to_exclude);

  ASSERT_EQ(P("[A->B, B->C, C->D]"), sub_graph.ShortestPath(N("A"), N("D")));
}

TEST_F(FourEdges, SubGraphTwo) {
  DirectedGraph graph(&storage_);

  GraphLinkSet links_to_exlude = {L("A", "B")};
  ExclusionSet to_exclude;
  to_exclude.AddToExcludeLinks(&links_to_exlude);
  SubGraph sub_graph(&graph, &to_exclude);

  ASSERT_EQ(P("[]"), sub_graph.ShortestPath(N("A"), N("B")));
}

// TEST(DFS, SingleLink) {
//  PBNet net;
//  AddEdgeToGraph("A", "B", Delay(100), kBw, &net);
//
//  GraphStorage graph_storage(net);
//  GraphNodeIndex node_a = graph_storage.NodeFromStringOrDie("A");
//  GraphNodeIndex node_b = graph_storage.NodeFromStringOrDie("B");
//  GraphLinkIndex link_ab = graph_storage.LinkOrDie("A", "B");
//
//  DirectedGraph graph(&graph_storage);
//  DFS dfs({}, &graph);
//
//  std::vector<Links> paths;
//  dfs.Paths(node_a, node_b, Delay(100), 10, [&paths](const LinkSequence& path)
//  {
//    paths.emplace_back(path.links());
//  });
//
//  std::vector<Links> model_paths = {{link_ab}};
//  ASSERT_EQ(model_paths, paths);
//
//  paths.clear();
//  dfs.Paths(node_a, node_b, Delay(99), 10, [&paths](const LinkSequence& path)
//  {
//    paths.emplace_back(path.links());
//  });
//  ASSERT_TRUE(paths.empty());
//}
//
// TEST(DFS, MultiPath) {
//  PBNet net;
//  AddEdgeToGraph("A", "B", Delay(100), kBw, &net);
//  AddEdgeToGraph("B", "C", Delay(100), kBw, &net);
//  AddEdgeToGraph("C", "D", Delay(100), kBw, &net);
//  AddEdgeToGraph("A", "D", Delay(100), kBw, &net);
//
//  GraphStorage graph_storage(net);
//  GraphNodeIndex node_a = graph_storage.NodeFromStringOrDie("A");
//  GraphNodeIndex node_d = graph_storage.NodeFromStringOrDie("D");
//  GraphLinkIndex link_ab = graph_storage.LinkOrDie("A", "B");
//  GraphLinkIndex link_bc = graph_storage.LinkOrDie("B", "C");
//  GraphLinkIndex link_cd = graph_storage.LinkOrDie("C", "D");
//  GraphLinkIndex link_ad = graph_storage.LinkOrDie("A", "D");
//
//  DirectedGraph graph(&graph_storage);
//  DFS dfs({}, &graph);
//
//  std::vector<Links> paths;
//  dfs.Paths(
//      node_a, node_d, Delay(1000), 10,
//      [&paths](const LinkSequence& path) { paths.emplace_back(path.links());
//      });
//
//  std::vector<Links> model_paths = {{link_ab, link_bc, link_cd}, {link_ad}};
//  ASSERT_EQ(model_paths, paths);
//}
//
// TEST(DFS, Braess) {
//  using namespace std::chrono;
//  PBNet net = GenerateBraess(kBw);
//
//  GraphStorage graph_storage(net);
//  GraphNodeIndex node_a = graph_storage.NodeFromStringOrDie("A");
//  GraphNodeIndex node_d = graph_storage.NodeFromStringOrDie("D");
//
//  DirectedGraph graph(&graph_storage);
//  DFS dfs({}, &graph);
//
//  std::vector<const GraphPath*> paths;
//  dfs.Paths(node_a, node_d, duration_cast<Delay>(seconds(1)), 10,
//            [&graph_storage, &paths](const LinkSequence& path) {
//              paths.emplace_back(graph_storage.PathFromLinksOrDie(path, 0));
//            });
//
//  // The default metric depth limit should be enough to capture all three
//  paths
//  ASSERT_EQ(3ul, paths.size());
//  ASSERT_TRUE(IsInPaths("[A->C, C->D]", paths, 0, &graph_storage));
//  ASSERT_TRUE(IsInPaths("[A->B, B->D]", paths, 0, &graph_storage));
//  ASSERT_TRUE(IsInPaths("[A->B, B->C, C->D]", paths, 0, &graph_storage));
//}
//
// TEST(KShortest, Braess) {
//  PBNet net = GenerateBraess(kBw);
//
//  GraphStorage graph_storage(net);
//  GraphNodeIndex node_a = graph_storage.NodeFromStringOrDie("A");
//  GraphNodeIndex node_d = graph_storage.NodeFromStringOrDie("D");
//  GraphLinkIndex link_ab = graph_storage.LinkOrDie("A", "B");
//  GraphLinkIndex link_ac = graph_storage.LinkOrDie("A", "C");
//  GraphLinkIndex link_bc = graph_storage.LinkOrDie("B", "C");
//  GraphLinkIndex link_cd = graph_storage.LinkOrDie("C", "D");
//  GraphLinkIndex link_bd = graph_storage.LinkOrDie("B", "D");
//
//  DirectedGraph graph(&graph_storage);
//  KShortestPaths ksp({}, {}, node_a, node_d, &graph);
//
//  std::vector<Links> model_paths = {
//      {link_ac, link_cd}, {link_ab, link_bd}, {link_ab, link_bc, link_cd}};
//
//  std::vector<Links> paths = {ksp.NextPath().links(), ksp.NextPath().links(),
//                              ksp.NextPath().links()};
//  ASSERT_EQ(model_paths, paths);
//  ASSERT_TRUE(ksp.NextPath().empty());
//}
//
// net::PBNet GenerateWaypointGraph(Bandwidth bw) {
//  using namespace std::chrono;
//  PBNet net;
//  AddBiEdgeToGraph("A", "B", milliseconds(1), bw, &net);
//  AddBiEdgeToGraph("A", "E", milliseconds(2), bw, &net);
//  AddBiEdgeToGraph("E", "B", milliseconds(1), bw, &net);
//  AddBiEdgeToGraph("B", "C", milliseconds(100), bw, &net);
//  AddBiEdgeToGraph("E", "F", milliseconds(10), bw, &net);
//  AddBiEdgeToGraph("C", "D", milliseconds(1), bw, &net);
//  AddBiEdgeToGraph("C", "F", milliseconds(1), bw, &net);
//  AddBiEdgeToGraph("F", "D", milliseconds(1), bw, &net);
//
//  return net;
//}
//
// TEST(KShortest, Waypoints) {
//  PBNet net = GenerateWaypointGraph(kBw);
//  GraphStorage graph_storage(net);
//  GraphNodeIndex node_a = graph_storage.NodeFromStringOrDie("A");
//  GraphNodeIndex node_d = graph_storage.NodeFromStringOrDie("D");
//  GraphLinkIndex link_bc = graph_storage.LinkOrDie("B", "C");
//
//  DirectedGraph graph(&graph_storage);
//  KShortestPaths ksp({}, {link_bc}, node_a, node_d, &graph);
//
//  std::vector<const GraphPath*> paths;
//  paths.emplace_back(graph_storage.PathFromLinksOrDie(ksp.NextPath(), 0));
//  ASSERT_TRUE(IsInPaths("[A->B, B->C, C->D]", paths, 0, &graph_storage));
//
//  paths.emplace_back(graph_storage.PathFromLinksOrDie(ksp.NextPath(), 0));
//  ASSERT_TRUE(IsInPaths("[A->B, B->C, C->F, F->D]", paths, 0,
//  &graph_storage));
//
//  paths.emplace_back(graph_storage.PathFromLinksOrDie(ksp.NextPath(), 0));
//  ASSERT_TRUE(IsInPaths("[A->E, E->B, B->C, C->D]", paths, 0,
//  &graph_storage));
//
//  paths.emplace_back(graph_storage.PathFromLinksOrDie(ksp.NextPath(), 0));
//  ASSERT_TRUE(
//      IsInPaths("[A->E, E->B, B->C, C->F, F->D]", paths, 0, &graph_storage));
//
//  paths.emplace_back(graph_storage.PathFromLinksOrDie(ksp.NextPath(), 0));
//  ASSERT_TRUE(IsInPaths("[]", paths, 0, &graph_storage));
//}
//
// TEST(KShortest, WaypointsJoined) {
//  PBNet net = GenerateWaypointGraph(kBw);
//  GraphStorage graph_storage(net);
//  GraphNodeIndex node_a = graph_storage.NodeFromStringOrDie("A");
//  GraphNodeIndex node_d = graph_storage.NodeFromStringOrDie("D");
//  GraphLinkIndex link_ab = graph_storage.LinkOrDie("A", "B");
//  GraphLinkIndex link_bc = graph_storage.LinkOrDie("B", "C");
//  GraphLinkIndex link_cd = graph_storage.LinkOrDie("C", "D");
//
//  DirectedGraph graph(&graph_storage);
//  KShortestPaths ksp({}, {link_ab, link_bc, link_cd}, node_a, node_d, &graph);
//
//  std::vector<const GraphPath*> paths;
//  paths.emplace_back(graph_storage.PathFromLinksOrDie(ksp.NextPath(), 0));
//  ASSERT_TRUE(IsInPaths("[A->B, B->C, C->D]", paths, 0, &graph_storage));
//
//  paths.emplace_back(graph_storage.PathFromLinksOrDie(ksp.NextPath(), 0));
//  ASSERT_TRUE(IsInPaths("[]", paths, 0, &graph_storage));
//}
//
// TEST(Shortest, BadWaypoints) {
//  PBNet net = GenerateWaypointGraph(kBw);
//  GraphStorage graph_storage(net);
//  GraphNodeIndex node_a = graph_storage.NodeFromStringOrDie("A");
//  GraphNodeIndex node_d = graph_storage.NodeFromStringOrDie("D");
//  GraphLinkIndex link_bc = graph_storage.LinkOrDie("B", "C");
//  GraphLinkIndex link_be = graph_storage.LinkOrDie("B", "E");
//
//  Links waypoints = {link_bc, link_be};
//  DirectedGraph graph(&graph_storage);
//
//  auto sp = WaypointShortestPath({}, waypoints.begin(), waypoints.end(),
//  node_a,
//                                 node_d, &graph);
//  ASSERT_TRUE(sp.empty());
//}
//
// TEST(Shortest, BadWaypointsSrc) {
//  PBNet net = GenerateWaypointGraph(kBw);
//  GraphStorage graph_storage(net);
//  GraphNodeIndex node_b = graph_storage.NodeFromStringOrDie("B");
//  GraphNodeIndex node_d = graph_storage.NodeFromStringOrDie("D");
//  GraphLinkIndex link_bc = graph_storage.LinkOrDie("B", "C");
//  GraphLinkIndex link_be = graph_storage.LinkOrDie("B", "E");
//
//  Links waypoints = {link_bc, link_be};
//  DirectedGraph graph(&graph_storage);
//
//  auto sp = WaypointShortestPath({}, waypoints.begin(), waypoints.end(),
//  node_b,
//                                 node_d, &graph);
//  ASSERT_TRUE(sp.empty());
//}
//
// TEST(Shortest, BadWaypointsDst) {
//  PBNet net = GenerateWaypointGraph(kBw);
//  GraphStorage graph_storage(net);
//  GraphNodeIndex node_b = graph_storage.NodeFromStringOrDie("A");
//  GraphNodeIndex node_d = graph_storage.NodeFromStringOrDie("D");
//  GraphLinkIndex link_cf = graph_storage.LinkOrDie("C", "F");
//  GraphLinkIndex link_cd = graph_storage.LinkOrDie("C", "D");
//
//  Links waypoints = {link_cd, link_cf};
//  DirectedGraph graph(&graph_storage);
//
//  auto sp = WaypointShortestPath({}, waypoints.begin(), waypoints.end(),
//  node_b,
//                                 node_d, &graph);
//  ASSERT_TRUE(sp.empty());
//}

}  // namespace
}  // namespace net
}  // namespace ncode
