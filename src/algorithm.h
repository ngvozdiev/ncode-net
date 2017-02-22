#ifndef NCODE_NET_ALGO_H
#define NCODE_NET_ALGO_H

#include <stdint.h>
#include <limits>
#include <queue>

#include "net_common.h"

namespace nc {
namespace net {

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

 private:
  void ConstructAdjacencyList();

  const GraphStorage* graph_storage_;

  // For each node the edges that leave the node. Some edges may lead to the
  // same neighbor if simple_ is false.
  GraphNodeMap<std::vector<GraphLinkIndex>> adjacency_list_;

  // True if there are no multiple edges between any two nodes.
  bool simple_;
};

class GraphSearchAlgorithmConfig {
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

class GraphSearchAlgorithm {
 protected:
  GraphSearchAlgorithm(const GraphSearchAlgorithmConfig& config,
                       const DirectedGraph* graph);

  // The graph.
  const DirectedGraph* graph_;

  // Configuration for the algorithm.
  const GraphSearchAlgorithmConfig config_;
};

struct DistanceClusterTag {};
using DistanceClusterIndex = Index<DistanceClusterTag, uint16_t>;
using DistanceClusterSet = PerfectHashSet<uint16_t, DistanceClusterTag>;

template <typename V>
using DistanceClusterMap = PerfectHashMap<uint16_t, DistanceClusterTag, V>;

// Clusters a graph based on some distance threshold.
class DistanceClusteredGraph : public GraphSearchAlgorithm {
 public:
  DistanceClusteredGraph(const GraphSearchAlgorithmConfig& config,
                         Delay threshold, DirectedGraph* parent)
      : GraphSearchAlgorithm(config, parent) {
    Cluster(threshold);
  }

  // Returns the set of nodes that belong to a cluster.
  const GraphNodeSet& GetCluster(DistanceClusterIndex cluster_index) const {
    return cluster_store_.GetItemOrDie(cluster_index);
  }

  // Returns the index of the cluster that a node belongs to. Each node can be
  // part of at most one cluster.
  DistanceClusterIndex GetClusterForNode(GraphNodeIndex node_index) const {
    return node_to_cluster_.GetValueOrDie(node_index);
  }

  // Returns a set that includes only links that start/end at the given cluster.
  GraphLinkSet GetClusterLinkSet(DistanceClusterIndex cluster_index) const;

  const GraphNodeMap<DistanceClusterIndex>& node_to_cluster() const {
    return node_to_cluster_;
  }

  DistanceClusterSet AllClusters() const {
    return DistanceClusterSet::FullSetFromStore(cluster_store_);
  }

  GraphStorage* clustered_storage() { return clustered_storage_.get(); }

  const GraphStorage* clustered_storage() const {
    return clustered_storage_.get();
  }

  const GraphLinkMap<GraphLinkIndex>& real_to_clustered_links() const {
    return real_to_clustered_links_;
  }

  const GraphLinkMap<GraphLinkIndex>& clustered_to_real_links() const {
    return clustered_to_real_links_;
  }

  const GraphNodeMap<GraphNodeIndex>& real_to_clustered_nodes() const {
    return real_to_clustered_nodes_;
  }

 private:
  static bool IsInClusters(const std::vector<GraphNodeSet>& clusters,
                           GraphNodeIndex node);

  void Cluster(Delay threshold);

  // Clusters are stored here. Populated upon construction.
  PerfectHashStore<GraphNodeSet, uint16_t, DistanceClusterTag> cluster_store_;

  // Relates from node to the node's cluster index.
  GraphNodeMap<DistanceClusterIndex> node_to_cluster_;

  // The graph composed of only the clustered nodes.
  std::unique_ptr<GraphStorage> clustered_storage_;

  GraphLinkMap<GraphLinkIndex> real_to_clustered_links_;
  GraphLinkMap<GraphLinkIndex> clustered_to_real_links_;
  GraphNodeMap<GraphNodeIndex> real_to_clustered_nodes_;
};

// Computes shortest paths between all pairs of nodes, can also be used to
// figure out if the graph is partitioned.
class AllPairShortestPath : public GraphSearchAlgorithm {
 public:
  AllPairShortestPath(const GraphSearchAlgorithmConfig& config,
                      const DirectedGraph* graph)
      : GraphSearchAlgorithm(config, graph) {
    CHECK(graph->IsSimple()) << "All pairs SP will only work on simple graphs";
    ComputePaths();
  }

  // Returns the shortest path between src and dst. The second return value will
  // be set to false if the path fails to avoid all depref links/nodes,
  // otherwise unchanged.
  LinkSequence GetPath(GraphNodeIndex src, GraphNodeIndex dst) const;

  // Returns the length of the shortest path between src and dst.
  Delay GetDistance(GraphNodeIndex src, GraphNodeIndex dst) const;

 private:
  static constexpr Delay kMaxDistance = Delay::max();

  struct SPData {
    SPData() : distance(kMaxDistance) {}

    // Distance between the 2 endpoints.
    Delay distance;

    // Successor in the SP.
    GraphLinkIndex next_link;
    GraphNodeIndex next_node;
  };

  // Populates data_.
  void ComputePaths();

  // Distances to the destination.
  GraphNodeMap<GraphNodeMap<SPData>> data_;
};

// Single source shortest path.
class ShortestPath : public GraphSearchAlgorithm {
 public:
  ShortestPath(const GraphSearchAlgorithmConfig& config, GraphNodeIndex src,
               const DirectedGraph* graph)
      : GraphSearchAlgorithm(config, graph), src_(src) {
    ComputePaths();
  }

  // Returns the shortest path to the destination.
  LinkSequence GetPath(GraphNodeIndex dst) const;

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
};

// Returns the single shortest path that goes through a series of links in the
// given order or returns an empty path if no such path exists.
LinkSequence WaypointShortestPath(const GraphSearchAlgorithmConfig& config,
                                  Links::const_iterator waypoints_from,
                                  Links::const_iterator waypoints_to,
                                  GraphNodeIndex src, GraphNodeIndex dst,
                                  const DirectedGraph* graph);

// K shortest paths that optionally go through a set of waypoints.
class KShortestPaths : public GraphSearchAlgorithm {
 public:
  KShortestPaths(const GraphSearchAlgorithmConfig& config,
                 const Links& waypoints, GraphNodeIndex src, GraphNodeIndex dst,
                 const DirectedGraph* graph);

  // Returns the next path.
  LinkSequence NextPath();

 private:
  using PathAndStartIndex = std::pair<LinkSequence, size_t>;

  // Returns true if prefix_path[0:index] == path[0:index]
  static bool HasPrefix(const Links& path, const Links& prefix);

  // Returns a set of links that contains: for any path in k_paths_ that starts
  // with the same links as root_path pick the next link -- the one after.
  void GetLinkExclusionSet(const Links& root_path, GraphLinkSet* out);

  // Waypoints.
  const std::vector<GraphLinkIndex> waypoints_;

  // The source.
  GraphNodeIndex src_;

  // The destination.
  GraphNodeIndex dst_;

  // Stores the K shortest paths in order.
  std::vector<PathAndStartIndex> k_paths_;

  // Stores candidates for K shortest paths.
  std::priority_queue<PathAndStartIndex, std::vector<PathAndStartIndex>,
                      std::greater<PathAndStartIndex>> candidates_;
};

// Simple depth-limited DFS.
class DFS : public GraphSearchAlgorithm {
 public:
  using PathCallback = std::function<void(const LinkSequence&)>;

  // If the last argument is true will compute an all-pairs shortest path and
  // use the shortest distance from any node to the destination to prune paths
  // that are too long early. The downside is that it may be slower and more
  // memory hungry, especially if you are only interested in reachability.
  DFS(const GraphSearchAlgorithmConfig& config, const DirectedGraph* graph,
      bool prune_distance = true);

  // Calls a callback on all paths between a source and a destination.
  void Paths(GraphNodeIndex src, GraphNodeIndex dst, Delay max_distance,
             size_t max_hops, PathCallback path_callback) const;

  // The set of nodes that are reachable from a given node.
  GraphNodeSet ReachableNodes(GraphNodeIndex src);

 private:
  void PathsRecursive(Delay max_distance, size_t max_hops, GraphNodeIndex at,
                      GraphNodeIndex dst, PathCallback path_callback,
                      GraphNodeSet* nodes_seen, Links* current,
                      Delay* total_distance) const;

  void ReachableNodesRecursive(GraphNodeIndex at,
                               GraphNodeSet* nodes_seen) const;

  // The graph storage.
  const GraphStorage* storage_;

  // The shortest paths are used to prune the DFS like in A*.
  std::unique_ptr<AllPairShortestPath> all_pair_sp_;
};

}  // namespace nc
}  // namespace ncode

#endif
