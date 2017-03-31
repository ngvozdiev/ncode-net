#include "algorithm.h"

#include <ncode/ncode_common/common.h>
#include <ncode/ncode_common/logging.h>
#include <ncode/ncode_common/perfect_hash.h>
#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace nc {
namespace net {

constexpr Delay AllPairShortestPath::kMaxDistance;

DirectedGraph::DirectedGraph(const GraphStorage* parent)
    : graph_storage_(parent) {
  ConstructAdjacencyList();
  CacheSP();
}

void DirectedGraph::ConstructAdjacencyList() {
  simple_ = true;
  for (GraphLinkIndex link : graph_storage_->AllLinks()) {
    const GraphLink* link_ptr = graph_storage_->GetLink(link);
    GraphNodeIndex src = link_ptr->src();
    GraphNodeIndex dst = link_ptr->dst();

    std::vector<net::GraphLinkIndex>& out_links = adjacency_list_[src];
    for (net::GraphLinkIndex link : out_links) {
      const GraphLink* link_ptr = graph_storage_->GetLink(link);
      if (link_ptr->dst() == dst) {
        simple_ = false;
      }
    }

    out_links.emplace_back(link);
  }
}

void DirectedGraph::CacheSP() {
  CHECK(shortest_paths_.Empty()) << "Already cached";

  for (GraphNodeIndex src : graph_storage_->AllNodes()) {
    ShortestPath sp(&adjacency_list_, graph_storage_, src);
    for (GraphNodeIndex dst : graph_storage_->AllNodes()) {
      if (src != dst) {
        shortest_paths_[src][dst] = sp.GetPath(dst);
      }
    }
  }
}

GraphSearchAlgorithm::GraphSearchAlgorithm(
    const GraphSearchAlgorithmConfig& config, const DirectedGraph* graph)
    : graph_(graph), config_(config) {}

net::LinkSequence AllPairShortestPath::GetPath(GraphNodeIndex src,
                                               GraphNodeIndex dst) const {
  Delay dist = data_[src][dst].distance;
  if (dist == kMaxDistance) {
    return {};
  }

  Links links;
  GraphNodeIndex next = src;
  while (next != dst) {
    const SPData& datum = data_[next][dst];
    links.emplace_back(datum.next_link);
    next = datum.next_node;
  }

  return {links, dist};
}

Delay AllPairShortestPath::GetDistance(GraphNodeIndex src,
                                       GraphNodeIndex dst) const {
  return data_[src][dst].distance;
}

void AllPairShortestPath::ComputePaths() {
  const GraphStorage* graph_storage = graph_->graph_storage();

  const GraphNodeSet nodes = graph_storage->AllNodes();
  for (GraphNodeIndex node : nodes) {
    if (config_.CanExcludeNode(node)) {
      continue;
    }

    SPData& node_data = data_[node][node];
    node_data.distance = Delay::zero();
  }

  for (GraphLinkIndex link : graph_storage->AllLinks()) {
    if (config_.CanExcludeLink(link)) {
      continue;
    }

    const GraphLink* link_ptr = graph_storage->GetLink(link);
    Delay distance = link_ptr->delay();

    SPData& sp_data = data_[link_ptr->src()][link_ptr->dst()];
    sp_data.distance = distance;
    sp_data.next_link = link;
    sp_data.next_node = link_ptr->dst();
  }

  for (GraphNodeIndex k : nodes) {
    for (GraphNodeIndex i : nodes) {
      for (GraphNodeIndex j : nodes) {
        Delay i_k = data_[i][k].distance;
        Delay k_j = data_[k][j].distance;

        bool any_max = (i_k == kMaxDistance || k_j == kMaxDistance);
        Delay alt_distance = any_max ? kMaxDistance : i_k + k_j;

        SPData& i_j_data = data_[i][j];
        if (alt_distance < i_j_data.distance) {
          i_j_data.distance = alt_distance;

          const SPData& i_k_data = data_[i][k];
          i_j_data.next_link = i_k_data.next_link;
          i_j_data.next_node = i_k_data.next_node;
        }
      }
    }
  }
}

DFS::DFS(const GraphSearchAlgorithmConfig& config, const DirectedGraph* graph,
         bool prune_distance)
    : GraphSearchAlgorithm(config, graph), storage_(graph->graph_storage()) {
  if (prune_distance) {
    all_pair_sp_ = make_unique<AllPairShortestPath>(config, graph_);
  }
}

void DFS::Paths(GraphNodeIndex src, GraphNodeIndex dst, Delay max_distance,
                size_t max_hops, PathCallback path_callback) const {
  Delay total_distance = Delay::zero();
  GraphNodeSet nodes_seen;
  Links scratch_path;
  PathsRecursive(max_distance, max_hops, src, dst, path_callback, &nodes_seen,
                 &scratch_path, &total_distance);
}

void DFS::PathsRecursive(Delay max_distance, size_t max_hops, GraphNodeIndex at,
                         GraphNodeIndex dst, PathCallback path_callback,
                         GraphNodeSet* nodes_seen, Links* current,
                         Delay* total_distance) const {
  if (current->size() > max_hops) {
    return;
  }

  if (at == dst) {
    path_callback(LinkSequence(*current, *total_distance));
    return;
  }

  Delay min_distance = *total_distance;
  if (all_pair_sp_) {
    min_distance += all_pair_sp_->GetDistance(at, dst);
  }

  if (min_distance > max_distance) {
    return;
  }

  if (nodes_seen->Contains(at)) {
    return;
  }
  nodes_seen->Insert(at);

  const auto& adjacency_list = graph_->AdjacencyList();
  const std::vector<GraphLinkIndex>& outgoing_links = adjacency_list[at];

  for (GraphLinkIndex out_link : outgoing_links) {
    if (config_.CanExcludeLink(out_link)) {
      continue;
    }

    const GraphLink* next_link = storage_->GetLink(out_link);
    GraphNodeIndex next_hop = next_link->dst();
    if (config_.CanExcludeNode(next_hop)) {
      continue;
    }

    current->push_back(out_link);
    *total_distance += next_link->delay();
    PathsRecursive(max_distance, max_hops, next_hop, dst, path_callback,
                   nodes_seen, current, total_distance);
    *total_distance -= next_link->delay();
    current->pop_back();
  }

  nodes_seen->Remove(at);
}

void DFS::ReachableNodesRecursive(GraphNodeIndex at,
                                  GraphNodeSet* nodes_seen) const {
  if (nodes_seen->Contains(at)) {
    return;
  }
  nodes_seen->Insert(at);

  const auto& adjacency_list = graph_->AdjacencyList();
  const std::vector<GraphLinkIndex>& outgoing_links = adjacency_list[at];

  for (GraphLinkIndex out_link : outgoing_links) {
    if (config_.CanExcludeLink(out_link)) {
      continue;
    }

    const GraphLink* next_link = storage_->GetLink(out_link);
    GraphNodeIndex next_hop = next_link->dst();
    if (config_.CanExcludeNode(next_hop)) {
      continue;
    }

    ReachableNodesRecursive(next_hop, nodes_seen);
  }
}

LinkSequence ShortestPath::GetPath(GraphNodeIndex dst) const {
  Links links_reverse;

  GraphNodeIndex current = dst;
  while (current != src_) {
    if (!previous_.HasValue(current)) {
      return {};
    }

    GraphLinkIndex link = previous_[current];
    const GraphLink* link_ptr = graph_storage_->GetLink(link);

    links_reverse.emplace_back(link);
    current = link_ptr->src();
  }

  std::reverse(links_reverse.begin(), links_reverse.end());
  Delay distance = min_delays_[dst].distance;
  return {links_reverse, distance};
}

void ShortestPath::ComputePaths() {
  using DelayAndIndex = std::pair<Delay, GraphNodeIndex>;
  std::priority_queue<DelayAndIndex, std::vector<DelayAndIndex>,
                      std::greater<DelayAndIndex>> vertex_queue;

  min_delays_.Resize(graph_storage_->NodeCount());
  previous_.Resize(graph_storage_->NodeCount());

  min_delays_.UnsafeAccess(src_).distance = Delay::zero();
  vertex_queue.emplace(Delay::zero(), src_);

  while (!vertex_queue.empty()) {
    Delay distance;
    GraphNodeIndex current;
    std::tie(distance, current) = vertex_queue.top();
    vertex_queue.pop();

    if (!adj_list_->HasValue(current)) {
      // A leaf.
      continue;
    }

    if (distance > min_delays_.UnsafeAccess(current).distance) {
      // Bogus leftover node, since we never delete nodes from the heap.
      continue;
    }

    const std::vector<GraphLinkIndex>& neighbors =
        adj_list_->UnsafeAccess(current);
    for (GraphLinkIndex out_link : neighbors) {
      const GraphLink* out_link_ptr = graph_storage_->GetLink(out_link);
      GraphNodeIndex neighbor_node = out_link_ptr->dst();

      const Delay link_delay = out_link_ptr->delay();
      const Delay distance_via_neighbor = distance + link_delay;
      Delay& curr_min_distance =
          min_delays_.UnsafeAccess(neighbor_node).distance;

      if (distance_via_neighbor < curr_min_distance) {
        curr_min_distance = distance_via_neighbor;
        previous_.UnsafeAccess(neighbor_node) = out_link;
        vertex_queue.emplace(curr_min_distance, neighbor_node);
      }
    }
  }
}

LinkSequence DirectedGraph::ShortestPathWithConstraints(
    const GraphSearchAlgorithmExclusionSet& to_exclude, GraphNodeIndex src,
    GraphNodeIndex dst) const {
  using DelayAndIndex = std::pair<Delay, GraphNodeIndex>;
  std::priority_queue<DelayAndIndex, std::vector<DelayAndIndex>,
                      std::greater<DelayAndIndex>> open_set;
  GraphNodeSet closed_set;
  GraphNodeMap<Delay> g_score;
  g_score.Resize(graph_storage_->NodeCount());
  for (GraphNodeIndex node_index : graph_storage_->AllNodes()) {
    g_score.UnsafeAccess(node_index) = Delay::max();
  }
  g_score[src] = Delay::zero();
  open_set.emplace(ShortestPath(src, dst).delay(), src);

  GraphNodeMap<GraphLinkIndex> previous;
  previous.Resize(graph_storage_->NodeCount());
  while (!open_set.empty()) {
    Delay distance;
    GraphNodeIndex current;
    std::tie(distance, current) = open_set.top();
    open_set.pop();

    if (closed_set.Contains(current)) {
      continue;
    }
    closed_set.Insert(current);

    if (!adjacency_list_.HasValue(current)) {
      // A leaf.
      continue;
    }

    const std::vector<GraphLinkIndex>& neighbors =
        adjacency_list_.UnsafeAccess(current);
    for (GraphLinkIndex out_link : neighbors) {
      const GraphLink* out_link_ptr = graph_storage_->GetLink(out_link);
      GraphNodeIndex neighbor_node = out_link_ptr->dst();
      if (closed_set.Contains(neighbor_node)) {
        continue;
      }

      const Delay link_delay = out_link_ptr->delay();
      const Delay tentative_g_score = g_score[current] + link_delay;
      Delay& curr_min_distance =
          min_delays_.UnsafeAccess(neighbor_node).distance;

      if (distance_via_neighbor < curr_min_distance) {
        curr_min_distance = distance_via_neighbor;
        previous_.UnsafeAccess(neighbor_node) = out_link;
        vertex_queue.emplace(curr_min_distance, neighbor_node);
      }
    }
  }
}

static void AddFromPath(const DirectedGraph& graph, const LinkSequence& path,
                        Links* out, GraphNodeSet* nodes) {
  const GraphStorage* graph_storage = graph.graph_storage();
  for (GraphLinkIndex link_in_path : path.links()) {
    const GraphLink* link_ptr = graph_storage->GetLink(link_in_path);

    out->emplace_back(link_in_path);
    nodes->Insert(link_ptr->src());
    nodes->Insert(link_ptr->dst());
  }
}

LinkSequence WaypointShortestPath(const GraphSearchAlgorithmConfig& config,
                                  Links::const_iterator waypoints_from,
                                  Links::const_iterator waypoints_to,
                                  GraphNodeIndex src, GraphNodeIndex dst,
                                  const DirectedGraph* graph) {
  CHECK(src != dst);
  const GraphStorage* graph_storage = graph->graph_storage();

  // As new paths are discovered nodes will be added to this set to make sure
  // the next paths do not include any nodes from previous paths.
  GraphSearchAlgorithmConfig config_copy = config;
  GraphNodeSet nodes_to_exclude;
  config_copy.AddToExcludeNodes(&nodes_to_exclude);

  // The shortest path is the combination of the shortest paths between the
  // nodes that we have to visit.
  GraphNodeIndex current_point = src;
  Links path;
  Delay total_delay = Delay::zero();
  for (auto it = waypoints_from; it < waypoints_to; ++it) {
    // The next SP is that between the current point and the source of the
    // edge, the path is then concatenated with the edge and the current point
    // is set to the end of the edge.
    GraphLinkIndex link_index = *it;
    const GraphLink* link_ptr = graph_storage->GetLink(link_index);

    // If the waypoint link is in the excluded set we cannot find a path through
    // the waypoints and we return an empty path.
    if (config_copy.CanExcludeLink(link_index)) {
      return {};
    }

    // If the source of the waypoint is the same as the src, but this is not
    // the first waypoint, obviously there is nothing we can do.
    if (src == link_ptr->src() && it != waypoints_from) {
      return {};
    }

    if (src != link_ptr->src() && current_point != link_ptr->src()) {
      ShortestPath sp(config_copy, current_point, graph);
      LinkSequence pathlet = sp.GetPath(link_ptr->src());
      pathlet.ToString(graph_storage);
      if (pathlet.empty()) {
        return {};
      }

      AddFromPath(*graph, pathlet, &path, &nodes_to_exclude);
      total_delay += pathlet.delay();
    }

    path.emplace_back(link_index);
    total_delay += link_ptr->delay();
    current_point = link_ptr->dst();
  }

  if (current_point != dst) {
    // Have to connect the last hop with the destination.
    ShortestPath sp(config_copy, current_point, graph);
    LinkSequence pathlet = sp.GetPath(dst);
    if (pathlet.empty()) {
      return {};
    }

    AddFromPath(*graph, pathlet, &path, &nodes_to_exclude);
    total_delay += pathlet.delay();
  }

  return LinkSequence(path, total_delay);
}

KShortestPaths::KShortestPaths(const GraphSearchAlgorithmConfig& config,
                               const std::vector<GraphLinkIndex>& waypoints,
                               GraphNodeIndex src, GraphNodeIndex dst,
                               const DirectedGraph* graph)
    : GraphSearchAlgorithm(config, graph),
      waypoints_(waypoints),
      src_(src),
      dst_(dst) {}

LinkSequence KShortestPaths::NextPath() {
  const GraphStorage* graph_storage = graph_->graph_storage();
  if (k_paths_.empty()) {
    LinkSequence path = WaypointShortestPath(
        config_, waypoints_.begin(), waypoints_.end(), src_, dst_, graph_);
    k_paths_.emplace_back(path, 0);
    return path;
  }

  const PathAndStartIndex& last_path_and_start_index = k_paths_.back();
  const LinkSequence& last_path = last_path_and_start_index.first;
  const Links& last_path_links = last_path.links();
  size_t start_index = last_path_and_start_index.second;

  GraphSearchAlgorithmConfig config_copy = config_;
  GraphLinkSet links_to_exclude;
  GraphNodeSet nodes_to_exclude;
  config_copy.AddToExcludeLinks(&links_to_exclude);
  config_copy.AddToExcludeNodes(&nodes_to_exclude);

  Links root_path;
  Links::const_iterator waypoints_from = waypoints_.begin();
  for (size_t i = 0; i < last_path_links.size(); ++i) {
    GraphLinkIndex link_index = last_path_links[i];
    const GraphLink* link = graph_storage->GetLink(link_index);
    GraphNodeIndex spur_node = link->src();
    if (i < start_index) {
      nodes_to_exclude.Insert(spur_node);
      root_path.emplace_back(link_index);

      if (waypoints_from != waypoints_.end() && link_index == *waypoints_from) {
        std::advance(waypoints_from, 1);
      }
      continue;
    }

    GetLinkExclusionSet(root_path, &links_to_exclude);
    LinkSequence spur_path = WaypointShortestPath(
        config_copy, waypoints_from, waypoints_.end(), spur_node, dst_, graph_);
    if (!spur_path.empty()) {
      const Links& spur_path_links = spur_path.links();

      Links candidate_links = root_path;
      candidate_links.insert(candidate_links.end(), spur_path_links.begin(),
                             spur_path_links.end());
      LinkSequence candidate_path(
          candidate_links, TotalDelayOfLinks(candidate_links, graph_storage));
      candidates_.emplace(candidate_path, i);
    }

    links_to_exclude.Clear();
    nodes_to_exclude.Insert(spur_node);
    root_path.emplace_back(link_index);
    if (waypoints_from != waypoints_.end() && link_index == *waypoints_from) {
      std::advance(waypoints_from, 1);
    }
  }

  if (candidates_.empty()) {
    return {};
  }

  PathAndStartIndex min_candidate = candidates_.top();
  candidates_.pop();
  k_paths_.emplace_back(min_candidate);
  return min_candidate.first;
}

bool KShortestPaths::HasPrefix(const Links& path, const Links& prefix) {
  CHECK(prefix.size() <= path.size()) << prefix.size() << " vs " << path.size();
  for (size_t i = 0; i < prefix.size(); ++i) {
    if (path[i] != prefix[i]) {
      return false;
    }
  }

  return true;
}

void KShortestPaths::GetLinkExclusionSet(const Links& root_path,
                                         GraphLinkSet* out) {
  for (const PathAndStartIndex& k_path_and_start_index : k_paths_) {
    const LinkSequence& k_path = k_path_and_start_index.first;
    if (k_path.size() < root_path.size()) {
      continue;
    }

    const Links& k_path_links = k_path.links();
    if (HasPrefix(k_path_links, root_path)) {
      CHECK(k_path_links.size() > root_path.size());
      out->Insert(k_path_links[root_path.size()]);
    }
  }
}

void DistanceClusteredGraph::Cluster(Delay threshold) {
  const GraphStorage* graph_storage = graph_->graph_storage();
  GraphNodeSet all_nodes = graph_storage->AllNodes();
  AllPairShortestPath all_pair_sp(config_, graph_);

  // Map from a node to all nodes that are within threshold distance of it.
  std::map<GraphNodeIndex, std::vector<GraphNodeIndex>> node_to_close_nodes;

  // Nodes that are within 'threshold' of eachother.
  for (GraphNodeIndex src : all_nodes) {
    for (GraphNodeIndex dst : all_nodes) {
      Delay distance = all_pair_sp.GetDistance(src, dst);
      if (distance <= threshold) {
        node_to_close_nodes[src].emplace_back(dst);
      }
    }
  }

  // GetDisjointSets used standard maps/sets instead of GraphNodeMap/Set, will
  // have to convert.
  std::vector<std::set<GraphNodeIndex>> clusters =
      GetDisjointSets(node_to_close_nodes);
  std::vector<GraphNodeSet> clusters_graph_sets;

  size_t total = 0;
  for (const std::set<GraphNodeIndex>& cluster : clusters) {
    GraphNodeSet cluster_set(cluster);
    clusters_graph_sets.emplace_back(cluster_set);

    DistanceClusterIndex cluster_index = cluster_store_.AddItem(cluster_set);
    for (GraphNodeIndex node : cluster) {
      ++total;
      node_to_cluster_[node] = cluster_index;
    }
  }

  // Each node should be in one cluster.
  CHECK(total == graph_storage->NodeCount());
  clustered_storage_ = graph_storage->ClusterNodes(clusters_graph_sets,
                                                   &real_to_clustered_links_,
                                                   &real_to_clustered_nodes_);
  for (const auto& real_and_clustered_link : real_to_clustered_links_) {
    GraphLinkIndex real_link = real_and_clustered_link.first;
    GraphLinkIndex clustered_link = *real_and_clustered_link.second;
    clustered_to_real_links_.Add(clustered_link, real_link);
  }
}

bool DistanceClusteredGraph::IsInClusters(
    const std::vector<GraphNodeSet>& clusters, GraphNodeIndex node) {
  for (const auto& cluster : clusters) {
    if (cluster.Contains(node)) {
      return true;
    }
  }

  return false;
}

GraphLinkSet DistanceClusteredGraph::GetClusterLinkSet(
    DistanceClusterIndex cluster_index) const {
  GraphLinkSet out;

  const GraphStorage* graph_storage = graph_->graph_storage();
  GraphNodeSet nodes_in_cluster = GetCluster(cluster_index);
  GraphLinkSet all_links = graph_storage->AllLinks();
  for (GraphLinkIndex link : all_links) {
    const GraphLink* link_ptr = graph_storage->GetLink(link);
    if (nodes_in_cluster.Contains(link_ptr->src()) ||
        nodes_in_cluster.Contains(link_ptr->dst())) {
      out.Insert(link);
    }
  }

  return out;
}

}  // namespace nc
}  // namespace ncode
