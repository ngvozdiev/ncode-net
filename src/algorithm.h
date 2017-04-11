#ifndef NCODE_NET_ALGO_H
#define NCODE_NET_ALGO_H

#include <stdint.h>
#include <limits>
#include <queue>

#include "net_common.h"
#include "trie.h"

namespace nc {
namespace net {

// A set of constraints.
class ConstraintSet {
 public:
  void AddToExcludeLinks(const GraphLinkSet* set) {
    link_sets_to_exclude_.emplace_back(set);
  }

  void PopExcludeLinks() { link_sets_to_exclude_.pop_back(); }

  void AddToExcludeNodes(const GraphNodeSet* set) {
    node_sets_to_exclude_.emplace_back(set);
  }

  void PopExcludeNodes() { node_sets_to_exclude_.pop_back(); }

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

  void AddToVisitSet(const GraphNodeSet* set) { to_visit_.emplace_back(set); }

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

// A disjunction constraint combines a number of ConstraintSets with ORs.
class DisjunctionConstraint {

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
    all_nodes_.Insert(src);
    all_nodes_.Insert(dst);
  }

  // The neighbors of a node. Empty if the node is a leaf.
  const std::vector<LinkInfo>& GetNeighbors(const GraphNodeIndex node) const {
    if (!adj_.HasValue(node)) {
      return empty_;
    }

    return adj_.GetValueOrDie(node);
  }

  const GraphNodeSet& AllNodes() const { return all_nodes_; }

  const GraphNodeMap<std::vector<LinkInfo>>& Adjacencies() const {
    return adj_;
  }

 private:
  // An empty vector that GetNeighbors can return a reference to;
  std::vector<LinkInfo> empty_;

  // For each node its neighbors.
  GraphNodeMap<std::vector<LinkInfo>> adj_;

  // All nodes/links.
  GraphNodeSet all_nodes_;
};

// Single source shortest path tree from a source to a set of nodes.
class ShortestPath {
 public:
  ShortestPath(GraphNodeIndex src, const GraphNodeSet& dst_nodes,
               const ConstraintSet& constraints, const AdjacencyList& adj_list,
               const GraphNodeSet* additional_nodes_to_avoid,
               const GraphLinkSet* additional_links_to_avoid)
      : src_(src), destinations_(dst_nodes) {
    ComputePaths(constraints, adj_list, additional_nodes_to_avoid,
                 additional_links_to_avoid);
  }

  // Returns the shortest path to the destination.
  LinkSequence GetPath(GraphNodeIndex dst) const;

  // Returns the distance from the source to a destination.
  Delay GetPathDistance(GraphNodeIndex dst) const;

  // Returns the nodes and  the links that are part of this tree.
  std::pair<GraphNodeSet, GraphLinkSet> ElementsInTree() const;

  friend bool operator<(const ShortestPath& a, const ShortestPath& b) {
    return std::tie(a.src_, a.previous_, a.min_delays_, a.destinations_) <
           std::tie(b.src_, b.previous_, b.min_delays_, b.destinations_);
  }

  friend bool operator==(const ShortestPath& a, const ShortestPath& b) {
    return std::tie(a.src_, a.previous_, a.min_delays_, a.destinations_) ==
           std::tie(b.src_, b.previous_, b.min_delays_, b.destinations_);
  }

 private:
  struct DistanceFromSource {
    DistanceFromSource() : distance(Delay::max()) {}
    Delay distance;

    friend bool operator<(const DistanceFromSource& a,
                          const DistanceFromSource& b) {
      return a.distance < b.distance;
    }

    friend bool operator==(const DistanceFromSource& a,
                           const DistanceFromSource& b) {
      return a.distance == b.distance;
    }
  };

  void ComputePaths(const ConstraintSet& constraints,
                    const AdjacencyList& adj_list,
                    const GraphNodeSet* additional_nodes_to_avoid,
                    const GraphLinkSet* additional_links_to_avoid);

  // The source.
  GraphNodeIndex src_;

  // For each node, the link that leads to it in the SP tree.
  GraphNodeMap<const AdjacencyList::LinkInfo*> previous_;

  // Delays from the source to each destination node.
  GraphNodeMap<DistanceFromSource> min_delays_;

  // The destinations.
  GraphNodeSet destinations_;
};

// Computes shortest paths between all pairs of nodes, can also be used to
// figure out if the graph is partitioned.
class AllPairShortestPath {
 public:
  AllPairShortestPath(const ConstraintSet& constraints,
                      const AdjacencyList& adj_list,
                      const GraphNodeSet* additional_nodes_to_avoid,
                      const GraphLinkSet* additional_links_to_avoid) {
    ComputePaths(constraints, adj_list, additional_nodes_to_avoid,
                 additional_links_to_avoid);
  }

  // Returns the shortest path between src and dst.
  LinkSequence GetPath(GraphNodeIndex src, GraphNodeIndex dst) const;

  // Returns the length of the shortest path between src and dst.
  Delay GetDistance(GraphNodeIndex src, GraphNodeIndex dst) const;

 private:
  struct SPData {
    SPData() : distance(Delay::max()) {}

    // Distance between the 2 endpoints.
    Delay distance;

    // Successor in the SP.
    GraphLinkIndex next_link;
    GraphNodeIndex next_node;
  };

  // Populates data_.
  void ComputePaths(const ConstraintSet& constraints,
                    const AdjacencyList& adj_list,
                    const GraphNodeSet* additional_nodes_to_avoid,
                    const GraphLinkSet* additional_links_to_avoid);

  // Distances to the destination.
  GraphNodeMap<GraphNodeMap<SPData>> data_;
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

  // Calls a callback with all paths between a source and a destination.
  void Paths(GraphNodeIndex src, GraphNodeIndex dst, PathCallback path_callback,
             Delay max_distance = Delay::max(),
             size_t max_hops = std::numeric_limits<size_t>::max()) const;

  // The set of nodes that are reachable from a given node.
  GraphNodeSet ReachableNodes(GraphNodeIndex src) const;

  // The shortest path between two nodes.
  LinkSequence ShortestPath(GraphNodeIndex src, GraphNodeIndex dst) const;

  const DirectedGraph* parent() const { return parent_; }

  const ConstraintSet* constraints() const { return constraints_; }

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

struct KShortestPathGeneratorStats {
  // Number of shortest paths kept in memory.
  size_t k = 0;

  // The number of bytes occupied by the K paths.
  size_t paths_size_bytes = 0;

  // Stats of the trie that lets the KSP algorithm do quick path prefix lookups.
  TrieStats trie_stats;

  // Number of candidate paths.
  size_t candidate_count = 0;

  // Total memory occupied by this KSP generator, includes the list of k paths,
  // the candidates, and the trie.
  size_t total_size_bytes = 0;

  std::string ToString() {
    return Substitute(
        "k: $0 ($1 bytes), trie: $2, candidates: $3, total: $4 bytes", k,
        paths_size_bytes, trie_stats.ToString(), candidate_count,
        total_size_bytes);
  }
};

// Generates shortest paths in increasing order.
class KShortestPathsGenerator {
 public:
  KShortestPathsGenerator(GraphNodeIndex src, GraphNodeIndex dst,
                          SubGraph* sub_graph)
      : src_(src), dst_(dst), sub_graph_(sub_graph) {}

  // Returns the Kth shortest path.
  LinkSequence KthShortestPath(size_t k);

  KShortestPathGeneratorStats GetStats() const {
    KShortestPathGeneratorStats stats;
    stats.k = k_paths_.size();
    stats.paths_size_bytes = PathContainerSize(k_paths_);

    stats.trie_stats = k_paths_trie_.GetStats();
    stats.candidate_count = candidates_.size();

    size_t candidate_overhead = PathContainerSize(candidates_.containter());
    stats.total_size_bytes = sizeof(*this) + stats.paths_size_bytes +
                             stats.trie_stats.size_bytes + candidate_overhead;

    return stats;
  }

 private:
  using PathAndStartIndex = std::pair<LinkSequence, size_t>;

  static size_t PathContainerSize(
      const std::vector<PathAndStartIndex>& container) {
    size_t total = container.capacity() * sizeof(PathAndStartIndex);
    for (const auto& path : container) {
      total += path.first.InMemBytesEstimate() - sizeof(LinkSequence);
    }

    return total;
  }

  // Shortest path between two nodes.
  LinkSequence ShortestPath(GraphNodeIndex src, GraphNodeIndex dst,
                            const GraphNodeSet& nodes_to_avoid,
                            const GraphLinkSet& links_to_avoid);

  // Adds the next shortest paths to the K shortest paths list. Returns true if
  // no more shortest paths exist.
  bool NextPath();

  // Returns a set of links that contains: for any path in k_paths_ that starts
  // with the same links as root_path pick the next link -- the one after.
  void GetLinkExclusionSet(const Links& root_path, GraphLinkSet* out);

  // Stores the K shortest paths in order.
  std::vector<PathAndStartIndex> k_paths_;

  // The K shortest paths, in a trie for quick prefix lookup.
  Trie<GraphLinkIndex, uint32_t> k_paths_trie_;

  // Stores candidates for K shortest paths.
  VectorPriorityQueue<PathAndStartIndex, std::greater<PathAndStartIndex>>
      candidates_;

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
