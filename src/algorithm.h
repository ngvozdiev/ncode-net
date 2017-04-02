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
    // This is the tree that starts at 'src_'. It may be possible that there are
    // nodes in the graph that are not reachable from 'src_'. Those nodes will
    // not have a distance set in 'min_delays_'.
    if (!min_delays_.HasValue(dst)) {
      return Delay::max();
    }

    return min_delays_.UnsafeAccess(dst).distance;
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

// A set of constraints.
class ConstraintSet {
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

  const std::vector<const GraphLinkSet*>& link_sets_to_exclude() const {
    return link_sets_to_exclude_;
  };

  const std::vector<const GraphNodeSet*>& node_sets_to_exclude() const {
    return node_sets_to_exclude_;
  };

  const std::vector<const GraphNodeSet*>& visit_order() const {
    return visit_order_;
  }

 private:
  // Links/nodes that will be excluded from the graph.
  std::vector<const GraphLinkSet*> link_sets_to_exclude_;
  std::vector<const GraphNodeSet*> node_sets_to_exclude_;

  // All paths will have to visit at least one node from each set. The sets
  // should be visited in the order they appear here.
  std::vector<const GraphNodeSet*> visit_order_;
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

  SubGraph(const DirectedGraph* parent, const ConstraintSet* constraints)
      : parent_(parent), constraints_(constraints) {}

  // Shortest path between two nodes.
  LinkSequence ShortestPath(GraphNodeIndex from, GraphNodeIndex to) const;

  // Calls a callback on all paths between a source and a destination.
  void Paths(GraphNodeIndex src, GraphNodeIndex dst, PathCallback path_callback,
             Delay max_distance = Delay::max(),
             size_t max_hops = std::numeric_limits<size_t>::max()) const;

  // The set of nodes that are reachable from a given node.
  GraphNodeSet ReachableNodes(GraphNodeIndex src) const;

  const DirectedGraph* parent() const { return parent_; }

  const ConstraintSet* exclusion_set() const { return constraints_; }

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
  const ConstraintSet* constraints_;
};

// Generates shortest paths in increasing order.
class KShortestPathsGenerator {
 public:
  KShortestPathsGenerator(GraphNodeIndex src, GraphNodeIndex dst,
                          const SubGraph* sub_graph)
      : src_(src), dst_(dst), sub_graph_(sub_graph) {}

  // Returns the Kth shortest path.
  LinkSequence KthShortestPath(size_t k);

 private:
  using PathAndStartIndex = std::pair<LinkSequence, size_t>;

  // Adds the next shortest paths to the K shortest paths list. Returns true if
  // no more shortest paths exist.
  bool NextPath();

  // Returns a set of links that contains: for any path in k_paths_ that starts
  // with the same links as root_path pick the next link -- the one after.
  void GetLinkExclusionSet(const Links& root_path, GraphLinkSet* out);

  // Stores the K shortest paths in order.
  std::vector<PathAndStartIndex> k_paths_;

  // Stores candidates for K shortest paths.
  std::priority_queue<PathAndStartIndex, std::vector<PathAndStartIndex>,
                      std::greater<PathAndStartIndex>> candidates_;

  // The source.
  GraphNodeIndex src_;

  // The destination.
  GraphNodeIndex dst_;

  // The graph.
  const SubGraph* sub_graph_;
};

}  // namespace nc
}  // namespace ncode

#endif
