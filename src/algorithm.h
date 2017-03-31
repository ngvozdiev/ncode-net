#ifndef NCODE_NET_ALGO_H
#define NCODE_NET_ALGO_H

#include <stdint.h>
#include <limits>
#include <queue>

#include "net_common.h"

namespace nc {
namespace net {

// Single source shortest path.
class ShortestPath {
 public:
  ShortestPath(const GraphNodeMap<std::vector<GraphLinkIndex>>* adj_list,
               const GraphStorage* graph_storage, GraphNodeIndex src)
      : src_(src), adj_list_(adj_list), graph_storage_(graph_storage) {
    ComputePaths();
  }

  // Returns the shortest path to the destination.
  LinkSequence GetPath(GraphNodeIndex dst) const;

  // Returns the distance from the source to a destination.
  Delay GetPathDistance(GraphNodeIndex dst) const {
    return min_delays_[dst].distance;
  }

 private:
  struct DistanceFromSource {
    DistanceFromSource() : distance(Delay::max()) {}
    Delay distance;
  };

  void ComputePaths();

  // The source.
  GraphNodeIndex src_;

  // For each node, the link that leads to it in the SP tree.
  GraphNodeMap<GraphLinkIndex> previous_;

  // Delays from each node to the destination.
  GraphNodeMap<DistanceFromSource> min_delays_;

  // Adjacency list.
  const GraphNodeMap<std::vector<GraphLinkIndex>>* adj_list_;

  // Graph storage.
  const GraphStorage* graph_storage_;

  DISALLOW_COPY_AND_ASSIGN(ShortestPath);
};

class ExclusionSet {
 public:
  void AddToExcludeLinks(const GraphLinkSet* set) {
    link_sets_to_exclude_.emplace_back(set);
  }

  void AddToExcludeNodes(const GraphNodeSet* set) {
    node_sets_to_exclude_.emplace_back(set);
  }

  bool CanExcludeLink(const GraphLinkIndex link) const {
    for (const GraphLinkSet* set : link_sets_to_exclude_) {
      if (set->Contains(link)) {
        return true;
      }
    }

    return false;
  }

  bool CanExcludeNode(const GraphNodeIndex node) const {
    for (const GraphNodeSet* set : node_sets_to_exclude_) {
      if (set->Contains(node)) {
        return true;
      }
    }

    return false;
  }

  std::vector<const GraphLinkSet*>& link_sets_to_exclude() {
    return link_sets_to_exclude_;
  };

  std::vector<const GraphNodeSet*>& node_sets_to_exclude() {
    return node_sets_to_exclude_;
  };

 private:
  // Links/nodes that will be excluded from the graph.
  std::vector<const GraphLinkSet*> link_sets_to_exclude_;
  std::vector<const GraphNodeSet*> node_sets_to_exclude_;
};

// A directed graph.
class DirectedGraph {
 public:
  DirectedGraph(const GraphStorage* parent);

  // Returns the adjacency list.
  const GraphNodeMap<std::vector<GraphLinkIndex>>& AdjacencyList() const {
    return adjacency_list_;
  }

  // The parent graph. Not owned by this object.
  const GraphStorage* graph_storage() const { return graph_storage_; }

  // Returns true if there is at most one link between any two nodes.
  bool IsSimple() const { return simple_; }

  // Returns the shortest path from 'from 'to 'to'. This considers the entire
  // graph so it is a lower bound.
  LinkSequence ShortestPath(GraphNodeIndex from, GraphNodeIndex to) const;

  // Returns the delay of the shortest paths between 'from' and 'to'.
  Delay ShortestPathDelay(GraphNodeIndex from, GraphNodeIndex to) const;

 private:
  void ConstructAdjacencyList();

  // Runs N instances of single-source shortest path and populates
  // 'shortest_paths_'.
  void CacheSP();

  const GraphStorage* graph_storage_;

  // For each node the edges that leave the node. Some edges may lead to the
  // same neighbor if simple_ is false.
  GraphNodeMap<std::vector<GraphLinkIndex>> adjacency_list_;

  // True if there are no multiple edges between any two nodes.
  bool simple_;

  // Shortest path trees, per source.
  GraphNodeMap<std::unique_ptr<net::ShortestPath>> shortest_paths_;
};

// Subset of a directed graph.
class SubGraph {
 public:
  using PathCallback = std::function<void(const LinkSequence&)>;

  SubGraph(const DirectedGraph* parent, const ExclusionSet* exclusion_set)
      : parent_(parent), exclusion_set_(exclusion_set) {}

  // Shortest path between two nodes.
  LinkSequence ShortestPath(GraphNodeIndex from, GraphNodeIndex to) const;

  // Calls a callback on all paths between a source and a destination.
  void Paths(GraphNodeIndex src, GraphNodeIndex dst, PathCallback path_callback,
             Delay max_distance, size_t max_hops) const;

  // The set of nodes that are reachable from a given node.
  GraphNodeSet ReachableNodes(GraphNodeIndex src) const;

 private:
  void PathsRecursive(Delay max_distance, size_t max_hops, GraphNodeIndex at,
                      GraphNodeIndex dst, PathCallback path_callback,
                      GraphNodeSet* nodes_seen, Links* current,
                      Delay* total_distance) const;

  void ReachableNodesRecursive(GraphNodeIndex at,
                               GraphNodeSet* nodes_seen) const;

  // The complete graph.
  const DirectedGraph* parent_;

  // Nodes/links to exclude.
  const ExclusionSet* exclusion_set_;
};

//// Returns the single shortest path that goes through a series of links in the
//// given order or returns an empty path if no such path exists.
// LinkSequence WaypointShortestPath(const GraphSearchAlgorithmConfig& config,
//                                  Links::const_iterator waypoints_from,
//                                  Links::const_iterator waypoints_to,
//                                  GraphNodeIndex src, GraphNodeIndex dst,
//                                  const DirectedGraph* graph);
//
//// K shortest paths that optionally go through a set of waypoints.
// class KShortestPaths : public GraphSearchAlgorithm {
// public:
//  KShortestPaths(const GraphSearchAlgorithmConfig& config,
//                 const Links& waypoints, GraphNodeIndex src, GraphNodeIndex
//                 dst,
//                 const DirectedGraph* graph);
//
//  // Returns the next path.
//  LinkSequence NextPath();
//
// private:
//  using PathAndStartIndex = std::pair<LinkSequence, size_t>;
//
//  // Returns true if prefix_path[0:index] == path[0:index]
//  static bool HasPrefix(const Links& path, const Links& prefix);
//
//  // Returns a set of links that contains: for any path in k_paths_ that
//  starts
//  // with the same links as root_path pick the next link -- the one after.
//  void GetLinkExclusionSet(const Links& root_path, GraphLinkSet* out);
//
//  // Waypoints.
//  const std::vector<GraphLinkIndex> waypoints_;
//
//  // The source.
//  GraphNodeIndex src_;
//
//  // The destination.
//  GraphNodeIndex dst_;
//
//  // Stores the K shortest paths in order.
//  std::vector<PathAndStartIndex> k_paths_;
//
//  // Stores candidates for K shortest paths.
//  std::priority_queue<PathAndStartIndex, std::vector<PathAndStartIndex>,
//                      std::greater<PathAndStartIndex>> candidates_;
//};

}  // namespace nc
}  // namespace ncode

#endif
