#ifndef NCODE_NET_ALGO_H
#define NCODE_NET_ALGO_H

#include <stdint.h>
#include <limits>
#include <queue>

#include "net_common.h"

namespace nc {
namespace net {

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

  const std::vector<const GraphNodeSet*>& to_visit() const { return to_visit_; }

 private:
  // Links/nodes that will be excluded from the graph.
  std::vector<const GraphLinkSet*> link_sets_to_exclude_;
  std::vector<const GraphNodeSet*> node_sets_to_exclude_;

  // Sets of nodes to visit, in the order given.
  std::vector<const GraphNodeSet*> to_visit_;
};

// Maintains connectivity information about a graph, and allows for quick
// retrieval of link information (without going to a GraphStorage etc.).
class AdjacencyList {
 public:
  struct LinkInfo {
    GraphLinkIndex link_index;
    GraphNodeIndex src_index;
    GraphNodeIndex dst_index;
    Delay delay;
  };

  void AddLink(GraphLinkIndex link_index, GraphNodeIndex src,
               GraphNodeIndex dst, Delay delay) {
    adj_[src].push_back({link_index, src, dst, delay});
    max_index_ = std::max(src, max_index_);
    max_index_ = std::max(dst, max_index_);
  }

  // The neighbors of a node. Empty if the node is a leaf.
  const std::vector<LinkInfo>& GetNeighbors(const GraphNodeIndex node) const {
    if (!adj_.HasValue(node)) {
      return empty_;
    }

    return adj_.GetValueOrDie(node);
  }

  // Max node index.
  GraphNodeIndex MaxNodeIndex() const { return max_index_; }

 private:
  // Max node index.
  GraphNodeIndex max_index_;

  // An empty vector that GetNeighbors can return a reference to;
  std::vector<LinkInfo> empty_;

  // For each node its neighbors.
  GraphNodeMap<std::vector<LinkInfo>> adj_;
};

// Single source shortest path tree from a source to a set of nodes.
class ShortestPath {
 public:
  ShortestPath(GraphNodeIndex src, const GraphNodeSet& dst_nodes,
               const ConstraintSet* constraints, const AdjacencyList* adj_list)
      : src_(src),
        destinations_(dst_nodes),
        adj_list_(adj_list),
        constraints_(constraints) {
    ComputePaths();
  }

  // Returns the shortest path to the destination.
  LinkSequence GetPath(GraphNodeIndex dst) const;

  // Returns the distance from the source to a destination.
  Delay GetPathDistance(GraphNodeIndex dst) const;

 private:
  struct DistanceFromSource {
    DistanceFromSource() : distance(Delay::max()) {}
    Delay distance;
  };

  void ComputePaths();

  // The source.
  GraphNodeIndex src_;

  // For each node, the link that leads to it in the SP tree.
  GraphNodeMap<const AdjacencyList::LinkInfo*> previous_;

  // Delays from the source to each destination node.
  GraphNodeMap<DistanceFromSource> min_delays_;

  // The destinations.
  GraphNodeSet destinations_;

  // Adjacency list.
  const AdjacencyList* adj_list_;

  // Constraints for the problem.
  const ConstraintSet* constraints_;

  DISALLOW_COPY_AND_ASSIGN(ShortestPath);
};

// Each node can belong to one of up to 64 groups. By assigning nodes to groups,
// constraints can be later specified for paths with respect to groups (e.g,
// paths should always cross a member of a group of nodes).
struct NodeGroupTag {};
using NodeGroup = TypesafeUintWrapper<NodeGroupTag, uint8_t>;

// A directed graph.
class DirectedGraph {
 public:
  DirectedGraph(const GraphStorage* parent);

  // Returns the adjacency list.
  const net::AdjacencyList& AdjacencyList() const { return adjacency_list_; }

  // Returns true if there is at most one link between any two nodes.
  bool IsSimple() const { return simple_; }

  const GraphStorage* graph_storage() const { return graph_storage_; }

 private:
  void PopulateAdjacencyList();

  // The graph's adjacency list.
  net::AdjacencyList adjacency_list_;

  // True if there are no multiple edges between any two nodes.
  bool simple_;

  const GraphStorage* graph_storage_;
};

// Subset of a directed graph.
class SubGraph {
 public:
  using PathCallback = std::function<void(const LinkSequence&)>;

  SubGraph(const DirectedGraph* parent, const ConstraintSet* constraints)
      : parent_(parent), constraints_(constraints) {}

  // Shortest path between two nodes.
  LinkSequence ShortestPath(GraphNodeIndex from, GraphNodeIndex to) const;

  // Calls a callback with all paths between a source and a destination.
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
