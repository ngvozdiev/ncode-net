#ifndef NCODE_NET_ALGO_H
#define NCODE_NET_ALGO_H

#include <stdint.h>
#include <limits>
#include <queue>

#include "net_common.h"
#include "trie.h"

namespace nc {
namespace net {

// Nodes/links that should be excluded.
class ExclusionSet {
 public:
  // Adds a new set of links to be excluded.
  void Links(const GraphLinkSet& set) { links_to_exclude_.InsertAll(set); }

  // Adds a new set of nodes to be excluded.
  void Nodes(const GraphNodeSet& set) { nodes_to_exclude_.InsertAll(set); }

  // Returns true if the link should be excluded.
  bool ShouldExcludeLink(const GraphLinkIndex link) const;

  // Returns true if the node should be excluded.
  bool ShouldExcludeNode(const GraphNodeIndex node) const;

  const GraphLinkSet& links_to_exclude() const { return links_to_exclude_; }

  const GraphNodeSet& nodes_to_exclude() const { return nodes_to_exclude_; }

  std::string ToString(const GraphStorage* storage) const {
    return Substitute("exclude links: $0, exclude nodes: $1",
                      GraphLinkSetToString(links_to_exclude_, storage),
                      GraphNodeSetToString(nodes_to_exclude_, storage));
  }

 private:
  // Links/nodes that will be excluded from the graph.
  GraphLinkSet links_to_exclude_;
  GraphNodeSet nodes_to_exclude_;
};

// A set of constraints.
class ConstraintSet {
 public:
  ConstraintSet() {}

  // The exclusion set.
  ExclusionSet& Exclude() { return exclusion_set_; }

  const ExclusionSet& exclusion_set() const { return exclusion_set_; }

  // Adds a new set of nodes to be visited.
  void AddToVisitSet(const GraphNodeSet& set);

  // If the links partially satisfy the visit constraints will return the index
  // of the last constraint that the links satisfy + 1. If the links fully
  // satisfy the constraints will return to_visit().size(). If the links do not
  // satisfy the constraints, or there are no visit constraints will return 0.
  // Assumes that the links form a path.
  size_t MinVisit(const Links& links, const GraphStorage* graph_storage) const;

  bool ShouldExcludeLink(const GraphLinkIndex link) const {
    return exclusion_set_.ShouldExcludeLink(link);
  }

  bool ShouldExcludeNode(const GraphNodeIndex node) const {
    return exclusion_set_.ShouldExcludeNode(node);
  }

  // Sets of nodes to visit, in the order given.
  const std::vector<GraphNodeSet>& to_visit() const { return to_visit_; }

  // Will make sure that the constrains do not have 'src' in the first set to
  // visit and 'dst' in the last set to visit.
  ConstraintSet SanitizeConstraints(GraphNodeIndex src,
                                    GraphNodeIndex dst) const {
    if (to_visit_.empty()) {
      return *this;
    }

    ConstraintSet to_return;
    to_return.Exclude().Links(exclusion_set_.links_to_exclude());
    to_return.Exclude().Nodes(exclusion_set_.nodes_to_exclude());

    for (size_t i = 0; i < to_visit_.size(); ++i) {
      GraphNodeSet set = to_visit_[i];

      if (i == 0 && set.Contains(src)) {
        set.Clear();
      } else if (i == to_visit_.size() - 1 && set.Contains(dst)) {
        set.Clear();
      } else {
        CHECK(!set.Contains(src) || !set.Contains(dst))
            << "src/dst can only be part of the first/last set";
      }

      if (set.Empty()) {
        continue;
      }

      to_return.AddToVisitSet(set);
    }

    return to_return;
  }

  std::string ToString(const GraphStorage* storage) const {
    std::vector<std::string> visit_constraints;
    for (const auto& set_to_visit : to_visit_) {
      visit_constraints.emplace_back(
          GraphNodeSetToString(set_to_visit, storage));
    }

    return Substitute("$0, visit nodes $1", exclusion_set_.ToString(storage),
                      Join(visit_constraints, "->"));
  }

 private:
  // Links/nodes that will be excluded from the graph.
  ExclusionSet exclusion_set_;

  // Sets of nodes to visit.
  std::vector<GraphNodeSet> to_visit_;

  // Maps from a node index to the index of the node's set in to_visit_.
  // Building this map also helps figure out if each node is in at most one set.
  GraphNodeMap<size_t> node_to_visit_index_;
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

  void Clear() {
    adj_.Clear();
    all_nodes_.Clear();
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
  // An empty vector that GetNeighbors can return a reference to.
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
               const ExclusionSet& exclusion_set, const AdjacencyList& adj_list,
               const GraphNodeSet* additional_nodes_to_avoid,
               const GraphLinkSet* additional_links_to_avoid)
      : src_(src), destinations_(dst_nodes) {
    ComputePaths(exclusion_set, adj_list, additional_nodes_to_avoid,
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

  void ComputePaths(const ExclusionSet& exclusion_set,
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
  AllPairShortestPath(const ExclusionSet& exclusion_set,
                      const AdjacencyList& adj_list,
                      const GraphNodeSet* additional_nodes_to_avoid,
                      const GraphLinkSet* additional_links_to_avoid) {
    ComputePaths(exclusion_set, adj_list, additional_nodes_to_avoid,
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
  void ComputePaths(const ExclusionSet& exclusion_set,
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

// Configuration for a DFS.
struct DFSConfig {
  bool simple = true;
  Delay max_distance = Delay::max();
  size_t max_hops = std::numeric_limits<size_t>::max();
};

// A directed graph and a set of constraints. Contains basic DFS and SP
// functionality.
class SubGraph {
 public:
  using PathCallback = std::function<void(const LinkSequence&)>;

  SubGraph(const DirectedGraph* parent, const ConstraintSet* constraints)
      : parent_(parent), constraints_(constraints) {}

  // Calls a callback with all paths between a source and a destination.
  void Paths(GraphNodeIndex src, GraphNodeIndex dst, PathCallback path_callback,
             const DFSConfig& dfs_config) const;

  // The set of nodes that are reachable from a given node.
  GraphNodeSet ReachableNodes(GraphNodeIndex src) const;

  // The shortest path between two nodes.
  LinkSequence ShortestPath(GraphNodeIndex src, GraphNodeIndex dst) const;

  const DirectedGraph* parent() const { return parent_; }

  const ConstraintSet* constraints() const { return constraints_; }

 private:
  void PathsRecursive(const DFSConfig& dfs_config, GraphNodeIndex at,
                      GraphNodeIndex dst, PathCallback path_callback,
                      GraphLinkSet* links_seen, GraphNodeSet* nodes_seen,
                      Links* current, Delay* total_distance) const;

  void ReachableNodesRecursive(GraphNodeIndex at,
                               GraphNodeSet* nodes_seen) const;

  // The complete graph.
  const DirectedGraph* parent_;

  // Nodes/links to exclude.
  const ConstraintSet* constraints_;
};

// A directed graph and multiple sets of constraints, connected by a logical OR
// class DisjointSubGraph {
// public:
//  DisjointSubGraph(const DirectedGraph* parent,
//                   const std::vector<const ConstraintSet*>& constraints) {
//    CHECK(!constraints.empty());
//    for (const ConstraintSet* constraint_set : constraints) {
//      sub_graphs_.emplace_back(parent, constraint_set);
//    }
//  }
//
//  // Returns all paths between a source and a destination, sorted in order
//  // of increasing delay. This function does not allow a callback to be called
//  // with each path because the sets of paths returned by each subgraph's DFS
//  // may overlap---need to store them and make sure they are unique.
//  std::set<LinkSequence> Paths(GraphNodeIndex src, GraphNodeIndex dst,
//                               const DFSConfig& dfs_config) const {
//    std::set<LinkSequence> all_paths;
//    for (const auto& sub_graph : sub_graphs_) {
//      sub_graph.Paths(src, dst, [&all_paths](const LinkSequence& links) {
//        all_paths.emplace(links);
//      }, dfs_config);
//    }
//
//    return all_paths;
//  }
//
//  // The set of nodes that are reachable from a given node.
//  GraphNodeSet ReachableNodes(GraphNodeIndex src) const {
//    GraphNodeSet out;
//    for (const auto& sub_graph : sub_graphs_) {
//      out.InsertAll(sub_graph.ReachableNodes(src));
//    }
//
//    return out;
//  }
//
//  // The shortest path between two nodes.
//  LinkSequence ShortestPath(GraphNodeIndex src, GraphNodeIndex dst) const {
//    LinkSequence shortest;
//    Delay shortest_delay = Delay::max();
//
//    for (const auto& sub_graph : sub_graphs_) {
//      LinkSequence sub_graph_sp = sub_graph.ShortestPath(src, dst);
//      if (sub_graph_sp.delay() < shortest_delay) {
//        shortest = sub_graph_sp;
//        shortest_delay = sub_graph_sp.delay();
//      }
//    }
//
//    return shortest;
//  }
//
// private:
//  std::vector<SubGraph> sub_graphs_;
//};

struct SubGraphShortestPathState {
  void Clear() {
    to_exclude.Clear();
    path_graph_adj_list.Clear();
    link_map.Clear();
    sp_trees.Clear();
  }

  // Nodes to exclude.
  GraphNodeSet to_exclude;

  // The adjacency list for the graph that is created from shortest paths from
  // each node in 'to_visit' to destinations.
  AdjacencyList path_graph_adj_list;

  // Maps a pair of src, dst with the link that represents the shortest path
  // between them in the new graph. The link index is not into the original
  // graph (from graph_storage) but one of the ones generated by
  // path_graph_link_index_gen.
  GraphLinkMap<std::pair<GraphNodeIndex, GraphNodeIndex>> link_map;

  // Shortest path trees.
  GraphNodeMap<std::unique_ptr<net::ShortestPath>> sp_trees;
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
      : src_(src),
        dst_(dst),
        constraints_(sub_graph->constraints()->SanitizeConstraints(src, dst)),
        graph_(sub_graph->parent()) {}

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
                            const GraphLinkSet& links_to_avoid,
                            const Links& links_so_far);

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

  // The sanitized constraints.
  const ConstraintSet constraints_;

  // The graph.
  const DirectedGraph* graph_;

  // State for the SP calls.
  SubGraphShortestPathState sp_state_;
};

// Like KShortestPathsGenerator above, but for a DisjointSubGraph.
// class DisjointKShortestPathsGenerator {
// public:
//  // Returns the next shortest path, starting at the shortest.
//  LinkSequence Next() {
//    if (queue_.empty()) {
//      return {};
//    }
//
//    PathGenAndPath top = queue_.top();
//    queue_.pop();
//
//    CHECK(!top.candidate.empty());
//    LinkSequence to_return = std::move(top.candidate);
//
//    top.candidate = top.generator->NextPath();
//    if (!top.candidate.empty()) {
//      queue_.push(top);
//    }
//    return to_return;
//  }
//
// private:
//  struct PathGenAndPath {
//    PathGenAndPath(KShortestPathsGenerator* generator, LinkSequence candidate)
//        : generator(generator), candidate(candidate) {}
//
//    ShortestPathGenerator* generator;
//    LinkSequence candidate;
//  };
//
//  struct Comparator {
//    bool operator()(const PathGenAndPath& lhs, const PathGenAndPath& rhs) {
//      return lhs.candidate.delay() < rhs.candidate.delay();
//    }
//  };
//
//  // A priority queue that has as many elements as there are KSP generators.
//  // When the next path is generated the minimum of the queue is taken and
//  // another path is generated from the generator that generated the minimum.
//  std::priority_queue<PathGenAndPath, std::vector<PathGenAndPath>, Comparator>
//      queue_;
//
//  // The generators.
//  std::vector<KShortestPathsGenerator> ksp_generators_;
//};

}  // namespace nc
}  // namespace ncode

#endif
