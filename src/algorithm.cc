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
    shortest_paths_[src] =
        make_unique<net::ShortestPath>(&adjacency_list_, graph_storage_, src);
  }
}

LinkSequence DirectedGraph::ShortestPath(GraphNodeIndex from,
                                         GraphNodeIndex to) const {
  return shortest_paths_[from]->GetPath(to);
}

Delay DirectedGraph::ShortestPathDelay(GraphNodeIndex from,
                                       GraphNodeIndex to) const {
  return shortest_paths_[from]->GetPathDistance(to);
}

void SubGraph::Paths(GraphNodeIndex src, GraphNodeIndex dst,
                     PathCallback path_callback, Delay max_distance,
                     size_t max_hops) const {
  Delay total_distance = Delay::zero();
  GraphNodeSet nodes_seen;
  Links scratch_path;
  PathsRecursive(max_distance, max_hops, src, dst, path_callback, &nodes_seen,
                 &scratch_path, &total_distance);
}

void SubGraph::PathsRecursive(Delay max_distance, size_t max_hops,
                              GraphNodeIndex at, GraphNodeIndex dst,
                              PathCallback path_callback,
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
  if (min_distance > max_distance) {
    return;
  }

  if (nodes_seen->Contains(at)) {
    return;
  }
  nodes_seen->Insert(at);

  const auto& adjacency_list = parent_->AdjacencyList();
  const std::vector<GraphLinkIndex>& outgoing_links = adjacency_list[at];
  const GraphStorage* storage = parent_->graph_storage();

  for (GraphLinkIndex out_link : outgoing_links) {
    if (exclusion_set_->CanExcludeLink(out_link)) {
      continue;
    }

    const GraphLink* next_link = storage->GetLink(out_link);
    GraphNodeIndex next_hop = next_link->dst();
    if (exclusion_set_->CanExcludeNode(next_hop)) {
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

void SubGraph::ReachableNodesRecursive(GraphNodeIndex at,
                                       GraphNodeSet* nodes_seen) const {
  if (nodes_seen->Contains(at)) {
    return;
  }
  nodes_seen->Insert(at);

  const auto& adjacency_list = parent_->AdjacencyList();
  const std::vector<GraphLinkIndex>& outgoing_links = adjacency_list[at];
  const GraphStorage* storage = parent_->graph_storage();

  for (GraphLinkIndex out_link : outgoing_links) {
    if (exclusion_set_->CanExcludeLink(out_link)) {
      continue;
    }

    const GraphLink* next_link = storage->GetLink(out_link);
    GraphNodeIndex next_hop = next_link->dst();
    if (exclusion_set_->CanExcludeNode(next_hop)) {
      continue;
    }

    ReachableNodesRecursive(next_hop, nodes_seen);
  }
}

static Links RecoverPath(GraphNodeIndex src, GraphNodeIndex dst,
                         const GraphNodeMap<GraphLinkIndex>& previous,
                         const GraphStorage* graph_storage) {
  Links links_reverse;

  GraphNodeIndex current = dst;
  while (current != src) {
    if (!previous.HasValue(current)) {
      return {};
    }

    GraphLinkIndex link = previous[current];
    const GraphLink* link_ptr = graph_storage->GetLink(link);

    links_reverse.emplace_back(link);
    current = link_ptr->src();
  }

  std::reverse(links_reverse.begin(), links_reverse.end());
  return links_reverse;
}

LinkSequence ShortestPath::GetPath(GraphNodeIndex dst) const {
  Links links = RecoverPath(src_, dst, previous_, graph_storage_);
  Delay distance = min_delays_[dst].distance;
  return {links, distance};
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

LinkSequence SubGraph::ShortestPath(GraphNodeIndex src,
                                    GraphNodeIndex dst) const {
  using DelayAndIndex = std::pair<Delay, GraphNodeIndex>;
  const GraphStorage* graph_storage = parent_->graph_storage();
  const auto& adjacency_list = parent_->AdjacencyList();

  std::priority_queue<DelayAndIndex, std::vector<DelayAndIndex>,
                      std::greater<DelayAndIndex>> frontier;
  GraphNodeMap<Delay> cost_so_far;
  cost_so_far.Resize(graph_storage->NodeCount());
  for (GraphNodeIndex node_index : graph_storage->AllNodes()) {
    cost_so_far.UnsafeAccess(node_index) = Delay::max();
  }
  cost_so_far[src] = Delay::zero();
  frontier.emplace(Delay::zero(), src);

  GraphNodeMap<GraphLinkIndex> came_from;
  came_from.Resize(graph_storage->NodeCount());
  while (!frontier.empty()) {
    Delay distance;
    GraphNodeIndex current;
    std::tie(distance, current) = frontier.top();
    frontier.pop();

    // Recover path.
    if (current == dst) {
      break;
    }

    if (!adjacency_list.HasValue(current)) {
      // A leaf.
      continue;
    }

    const std::vector<GraphLinkIndex>& neighbors =
        adjacency_list.UnsafeAccess(current);
    for (GraphLinkIndex out_link : neighbors) {
      const GraphLink* out_link_ptr = graph_storage->GetLink(out_link);
      GraphNodeIndex neighbor_node = out_link_ptr->dst();

      const Delay link_delay = out_link_ptr->delay();
      const Delay new_cost = cost_so_far[current] + link_delay;
      if (new_cost < cost_so_far[neighbor_node]) {
        cost_so_far[neighbor_node] = new_cost;
        Delay priority =
            new_cost + parent_->ShortestPathDelay(neighbor_node, dst);
        frontier.emplace(priority, neighbor_node);
        came_from[neighbor_node] = out_link;
      }
    }
  }

  Links links = RecoverPath(src, dst, came_from, graph_storage);
  return {links, graph_storage};
}

// static void AddFromPath(const DirectedGraph& graph, const LinkSequence& path,
//                        Links* out, GraphNodeSet* nodes) {
//  const GraphStorage* graph_storage = graph.graph_storage();
//  for (GraphLinkIndex link_in_path : path.links()) {
//    const GraphLink* link_ptr = graph_storage->GetLink(link_in_path);
//
//    out->emplace_back(link_in_path);
//    nodes->Insert(link_ptr->src());
//    nodes->Insert(link_ptr->dst());
//  }
//}

// LinkSequence WaypointShortestPath(const GraphSearchAlgorithmConfig& config,
//                                  Links::const_iterator waypoints_from,
//                                  Links::const_iterator waypoints_to,
//                                  GraphNodeIndex src, GraphNodeIndex dst,
//                                  const DirectedGraph* graph) {
//  CHECK(src != dst);
//  const GraphStorage* graph_storage = graph->graph_storage();
//
//  // As new paths are discovered nodes will be added to this set to make sure
//  // the next paths do not include any nodes from previous paths.
//  GraphSearchAlgorithmConfig config_copy = config;
//  GraphNodeSet nodes_to_exclude;
//  config_copy.AddToExcludeNodes(&nodes_to_exclude);
//
//  // The shortest path is the combination of the shortest paths between the
//  // nodes that we have to visit.
//  GraphNodeIndex current_point = src;
//  Links path;
//  Delay total_delay = Delay::zero();
//  for (auto it = waypoints_from; it < waypoints_to; ++it) {
//    // The next SP is that between the current point and the source of the
//    // edge, the path is then concatenated with the edge and the current point
//    // is set to the end of the edge.
//    GraphLinkIndex link_index = *it;
//    const GraphLink* link_ptr = graph_storage->GetLink(link_index);
//
//    // If the waypoint link is in the excluded set we cannot find a path
//    through
//    // the waypoints and we return an empty path.
//    if (config_copy.CanExcludeLink(link_index)) {
//      return {};
//    }
//
//    // If the source of the waypoint is the same as the src, but this is not
//    // the first waypoint, obviously there is nothing we can do.
//    if (src == link_ptr->src() && it != waypoints_from) {
//      return {};
//    }
//
//    if (src != link_ptr->src() && current_point != link_ptr->src()) {
//      ShortestPath sp(config_copy, current_point, graph);
//      LinkSequence pathlet = sp.GetPath(link_ptr->src());
//      pathlet.ToString(graph_storage);
//      if (pathlet.empty()) {
//        return {};
//      }
//
//      AddFromPath(*graph, pathlet, &path, &nodes_to_exclude);
//      total_delay += pathlet.delay();
//    }
//
//    path.emplace_back(link_index);
//    total_delay += link_ptr->delay();
//    current_point = link_ptr->dst();
//  }
//
//  if (current_point != dst) {
//    // Have to connect the last hop with the destination.
//    ShortestPath sp(config_copy, current_point, graph);
//    LinkSequence pathlet = sp.GetPath(dst);
//    if (pathlet.empty()) {
//      return {};
//    }
//
//    AddFromPath(*graph, pathlet, &path, &nodes_to_exclude);
//    total_delay += pathlet.delay();
//  }
//
//  return LinkSequence(path, total_delay);
//}

// KShortestPaths::KShortestPaths(const GraphSearchAlgorithmConfig& config,
//                               const std::vector<GraphLinkIndex>& waypoints,
//                               GraphNodeIndex src, GraphNodeIndex dst,
//                               const DirectedGraph* graph)
//    : GraphSearchAlgorithm(config, graph),
//      waypoints_(waypoints),
//      src_(src),
//      dst_(dst) {}
//
// LinkSequence KShortestPaths::NextPath() {
//  const GraphStorage* graph_storage = graph_->graph_storage();
//  if (k_paths_.empty()) {
//    LinkSequence path = WaypointShortestPath(
//        config_, waypoints_.begin(), waypoints_.end(), src_, dst_, graph_);
//    k_paths_.emplace_back(path, 0);
//    return path;
//  }
//
//  const PathAndStartIndex& last_path_and_start_index = k_paths_.back();
//  const LinkSequence& last_path = last_path_and_start_index.first;
//  const Links& last_path_links = last_path.links();
//  size_t start_index = last_path_and_start_index.second;
//
//  GraphSearchAlgorithmConfig config_copy = config_;
//  GraphLinkSet links_to_exclude;
//  GraphNodeSet nodes_to_exclude;
//  config_copy.AddToExcludeLinks(&links_to_exclude);
//  config_copy.AddToExcludeNodes(&nodes_to_exclude);
//
//  Links root_path;
//  Links::const_iterator waypoints_from = waypoints_.begin();
//  for (size_t i = 0; i < last_path_links.size(); ++i) {
//    GraphLinkIndex link_index = last_path_links[i];
//    const GraphLink* link = graph_storage->GetLink(link_index);
//    GraphNodeIndex spur_node = link->src();
//    if (i < start_index) {
//      nodes_to_exclude.Insert(spur_node);
//      root_path.emplace_back(link_index);
//
//      if (waypoints_from != waypoints_.end() && link_index == *waypoints_from)
//      {
//        std::advance(waypoints_from, 1);
//      }
//      continue;
//    }
//
//    GetLinkExclusionSet(root_path, &links_to_exclude);
//    LinkSequence spur_path = WaypointShortestPath(
//        config_copy, waypoints_from, waypoints_.end(), spur_node, dst_,
//        graph_);
//    if (!spur_path.empty()) {
//      const Links& spur_path_links = spur_path.links();
//
//      Links candidate_links = root_path;
//      candidate_links.insert(candidate_links.end(), spur_path_links.begin(),
//                             spur_path_links.end());
//      LinkSequence candidate_path(
//          candidate_links, TotalDelayOfLinks(candidate_links, graph_storage));
//      candidates_.emplace(candidate_path, i);
//    }
//
//    links_to_exclude.Clear();
//    nodes_to_exclude.Insert(spur_node);
//    root_path.emplace_back(link_index);
//    if (waypoints_from != waypoints_.end() && link_index == *waypoints_from) {
//      std::advance(waypoints_from, 1);
//    }
//  }
//
//  if (candidates_.empty()) {
//    return {};
//  }
//
//  PathAndStartIndex min_candidate = candidates_.top();
//  candidates_.pop();
//  k_paths_.emplace_back(min_candidate);
//  return min_candidate.first;
//}
//
// bool KShortestPaths::HasPrefix(const Links& path, const Links& prefix) {
//  CHECK(prefix.size() <= path.size()) << prefix.size() << " vs " <<
//  path.size();
//  for (size_t i = 0; i < prefix.size(); ++i) {
//    if (path[i] != prefix[i]) {
//      return false;
//    }
//  }
//
//  return true;
//}
//
// void KShortestPaths::GetLinkExclusionSet(const Links& root_path,
//                                         GraphLinkSet* out) {
//  for (const PathAndStartIndex& k_path_and_start_index : k_paths_) {
//    const LinkSequence& k_path = k_path_and_start_index.first;
//    if (k_path.size() < root_path.size()) {
//      continue;
//    }
//
//    const Links& k_path_links = k_path.links();
//    if (HasPrefix(k_path_links, root_path)) {
//      CHECK(k_path_links.size() > root_path.size());
//      out->Insert(k_path_links[root_path.size()]);
//    }
//  }
//}

}  // namespace nc
}  // namespace ncode
