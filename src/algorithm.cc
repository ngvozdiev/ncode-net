#include "algorithm.h"

#include <ncode/ncode_common/common.h>
#include <ncode/ncode_common/logging.h>
#include <ncode/ncode_common/perfect_hash.h>
#include <ncode/ncode_common/strutil.h>
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

DirectedGraph::DirectedGraph(const GraphStorage* storage)
    : graph_storage_(storage) {
  PopulateAdjacencyList();
}

void DirectedGraph::PopulateAdjacencyList() {
  simple_ = true;
  for (GraphLinkIndex link : graph_storage_->AllLinks()) {
    const GraphLink* link_ptr = graph_storage_->GetLink(link);
    GraphNodeIndex src = link_ptr->src();
    GraphNodeIndex dst = link_ptr->dst();
    for (const auto& link_info : adjacency_list_.GetNeighbors(src)) {
      if (link_info.dst_index == dst) {
        simple_ = false;
      }
    }

    adjacency_list_.AddLink(link, src, dst, link_ptr->delay());
  }
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

  const AdjacencyList& adjacency_list = parent_->AdjacencyList();
  const std::vector<AdjacencyList::LinkInfo>& outgoing_links =
      adjacency_list.GetNeighbors(at);

  for (const AdjacencyList::LinkInfo& out_link_info : outgoing_links) {
    if (constraints_->CanExcludeLink(out_link_info.link_index)) {
      continue;
    }

    GraphNodeIndex next_hop = out_link_info.dst_index;
    if (constraints_->CanExcludeNode(next_hop)) {
      continue;
    }

    current->push_back(out_link_info.link_index);
    *total_distance += out_link_info.delay;
    PathsRecursive(max_distance, max_hops, next_hop, dst, path_callback,
                   nodes_seen, current, total_distance);
    *total_distance -= out_link_info.delay;
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

  const AdjacencyList& adjacency_list = parent_->AdjacencyList();
  const std::vector<AdjacencyList::LinkInfo>& outgoing_links =
      adjacency_list.GetNeighbors(at);

  for (const AdjacencyList::LinkInfo& out_link_info : outgoing_links) {
    if (constraints_->CanExcludeLink(out_link_info.link_index)) {
      continue;
    }

    GraphNodeIndex next_hop = out_link_info.dst_index;
    if (constraints_->CanExcludeNode(next_hop)) {
      continue;
    }

    ReachableNodesRecursive(next_hop, nodes_seen);
  }
}

static Links RecoverPath(
    GraphNodeIndex src, GraphNodeIndex dst,
    const GraphNodeMap<const AdjacencyList::LinkInfo*>& previous) {
  Links links_reverse;

  GraphNodeIndex current = dst;
  while (current != src) {
    if (!previous.HasValue(current)) {
      return {};
    }

    const AdjacencyList::LinkInfo* link_info = previous[current];

    links_reverse.emplace_back(link_info->link_index);
    current = link_info->src_index;
  }

  std::reverse(links_reverse.begin(), links_reverse.end());
  return links_reverse;
}

LinkSequence ShortestPath::GetPath(GraphNodeIndex dst) const {
  CHECK(destinations_.Contains(dst)) << "Bad destination";

  Links links = RecoverPath(src_, dst, previous_);
  if (links.empty()) {
    return {};
  }

  Delay distance = min_delays_[dst].distance;
  return {links, distance};
}

Delay ShortestPath::GetPathDistance(GraphNodeIndex dst) const {
  CHECK(destinations_.Contains(dst)) << "Bad destination";

  // This is the tree that starts at 'src_'. It may be possible that there are
  // nodes in the graph that are not reachable from 'src_'. Those nodes will
  // not have a distance set in 'min_delays_'.
  if (!min_delays_.HasValue(dst)) {
    return Delay::max();
  }

  return min_delays_.UnsafeAccess(dst).distance;
}

void ShortestPath::ComputePaths() {
  using DelayAndIndex = std::pair<Delay, GraphNodeIndex>;
  std::priority_queue<DelayAndIndex, std::vector<DelayAndIndex>,
                      std::greater<DelayAndIndex>> vertex_queue;

  min_delays_.Resize(adj_list_->MaxNodeIndex() + 1);
  previous_.Resize(adj_list_->MaxNodeIndex() + 1);

  if (constraints_->CanExcludeNode(src_)) {
    return;
  }

  min_delays_.UnsafeAccess(src_).distance = Delay::zero();
  vertex_queue.emplace(Delay::zero(), src_);

  size_t destinations_remaining = destinations_.Count();
  while (!vertex_queue.empty()) {
    Delay distance;
    GraphNodeIndex current;
    std::tie(distance, current) = vertex_queue.top();
    vertex_queue.pop();

    if (distance > min_delays_.UnsafeAccess(current).distance) {
      // Bogus leftover node, since we never delete nodes from the heap.
      continue;
    }

    LOG(ERROR) << "CC " << current;

    if (destinations_.Contains(current)) {
      --destinations_remaining;
      if (destinations_remaining == 0) {
        break;
      }
    }

    const std::vector<AdjacencyList::LinkInfo>& neighbors =
        adj_list_->GetNeighbors(current);
    for (const AdjacencyList::LinkInfo& out_link_info : neighbors) {
      LOG(ERROR) << "N of " << current << " -> " << out_link_info.dst_index;
      GraphLinkIndex out_link = out_link_info.link_index;
      if (constraints_->CanExcludeLink(out_link)) {
        LOG(ERROR) << "AAA";
        continue;
      }

      GraphNodeIndex neighbor_node = out_link_info.dst_index;
      if (constraints_->CanExcludeNode(neighbor_node)) {
        LOG(ERROR) << "BBB";
        continue;
      }

      const Delay link_delay = out_link_info.delay;
      const Delay distance_via_neighbor = distance + link_delay;
      Delay& curr_min_distance =
          min_delays_.UnsafeAccess(neighbor_node).distance;

      if (distance_via_neighbor < curr_min_distance) {
        curr_min_distance = distance_via_neighbor;
        previous_.UnsafeAccess(neighbor_node) = &out_link_info;
        vertex_queue.emplace(curr_min_distance, neighbor_node);
      }
    }
  }
}

LinkSequence SubGraph::ShortestPath(GraphNodeIndex src,
                                    GraphNodeIndex dst) const {
  // Stores SP trees. It will contain an SP tree rooted at the source, as well
  // as every node in every set to visit. The paths in the tree will become
  // edges in a new graph, the shortest path in which will be the end-to-end
  // shortest path in the original graph.
  GraphNodeMap<std::unique_ptr<net::ShortestPath>> paths;

  GraphNodeSet to_exclude;
  ConstraintSet constraint_set_cpy = *constraints_;
  constraint_set_cpy.AddToExcludeNodes(&to_exclude);

  // The adjacency list for the graph that is created from shortest paths from
  // each node in 'to_visit' to destinations.
  AdjacencyList path_graph_adj_list;

  // Generates sequential link indices.
  size_t path_graph_link_index_gen = -1;

  // Maps a pair of src, dst with the link that represents the shortest path
  // between them in the new graph. The link index is not into the original
  // graph (from graph_storage) but one of the ones generated by
  // path_graph_link_index_gen.
  std::map<GraphLinkIndex, std::pair<GraphNodeIndex, GraphNodeIndex>> link_map;

  const std::vector<const GraphNodeSet*>& to_visit = constraints_->to_visit();
  LOG(ERROR) << "I " << src << " " << dst;
  for (size_t i = -1; i != to_visit.size(); ++i) {
    LOG(ERROR) << "I " << i;

    // For each set we will compute the SP trees rooted at each node. Each of
    // those SP trees should avoid nodes from other sets, except for the next
    // set, and be for destinations in the next set.
    to_exclude.Clear();
    for (size_t j = 0; j < to_visit.size(); ++j) {
      if (i != j && (i + 1) != j) {
        to_exclude.InsertAll(*to_visit[j]);
      }
    }

    GraphNodeSet destinations;
    if (i == to_visit.size() - 1) {
      destinations.Insert(dst);
    } else {
      destinations.InsertAll(*to_visit[i + 1]);
    }

    GraphNodeSet sources;
    if (i == static_cast<size_t>(-1)) {
      sources.Insert(src);
    } else {
      const GraphNodeSet& set_to_visit = *to_visit[i];
      sources.InsertAll(set_to_visit);
    }

    std::string out;
    for (GraphNodeIndex n : destinations) {
      out += " " + std::to_string(n);
    }
    LOG(ERROR) << "D " << out;

    out = "";
    for (GraphNodeIndex n : sources) {
      out += " " + std::to_string(n);
    }
    LOG(ERROR) << "S " << out;

    out = "";
    for (GraphNodeIndex n : to_exclude) {
      out += " " + std::to_string(n);
    }
    LOG(ERROR) << "TE " << out;

    for (GraphNodeIndex node_to_visit : sources) {
      LOG(ERROR) << "Will run SP";
      auto sp_tree = make_unique<net::ShortestPath>(node_to_visit, destinations,
                                                    &constraint_set_cpy,
                                                    &parent_->AdjacencyList());
      for (GraphNodeIndex destination : destinations) {
        GraphLinkIndex new_link_index(++path_graph_link_index_gen);
        Delay sp_delay = sp_tree->GetPathDistance(destination);
        path_graph_adj_list.AddLink(new_link_index, node_to_visit, destination,
                                    sp_delay);
        link_map[new_link_index] = {node_to_visit, destination};
        LOG(ERROR) << "SP " << node_to_visit << " -> " << destination << " li "
                   << new_link_index << " delay " << sp_delay.count();
      }
      paths[node_to_visit] = std::move(sp_tree);
    }
  }

  // Now we can find out the shortest path through the new graph, the links of
  // which will tell us which paths we need to stitch together in order to form
  // the final end-to-end path.
  ConstraintSet dummy;
  net::ShortestPath path_graph_sp(src, {dst}, &dummy, &path_graph_adj_list);
  LinkSequence sp = path_graph_sp.GetPath(dst);

  Links final_path;
  Delay total_delay = Delay::zero();
  for (GraphLinkIndex path_graph_link : sp.links()) {
    GraphNodeIndex sub_path_from;
    GraphNodeIndex sub_path_to;

    std::tie(sub_path_from, sub_path_to) = link_map[path_graph_link];
    LinkSequence sub_path = paths[sub_path_from]->GetPath(sub_path_to);
    final_path.insert(final_path.end(), sub_path.links().begin(),
                      sub_path.links().end());
    total_delay += sub_path.delay();
  }

  return {final_path, total_delay};
}

bool KShortestPathsGenerator::NextPath() {
  const DirectedGraph* parent = sub_graph_->parent();
  const GraphStorage* graph_storage = parent->graph_storage();

  if (k_paths_.empty()) {
    LinkSequence path = sub_graph_->ShortestPath(src_, dst_);
    k_paths_.emplace_back(path, 0);
    return true;
  }

  const PathAndStartIndex& last_path_and_start_index = k_paths_.back();
  const LinkSequence& last_path = last_path_and_start_index.first;
  const Links& last_path_links = last_path.links();
  size_t start_index = last_path_and_start_index.second;

  ConstraintSet constraint_set_copy = *(sub_graph_->exclusion_set());
  GraphLinkSet links_to_exclude;
  GraphNodeSet nodes_to_exclude;
  constraint_set_copy.AddToExcludeLinks(&links_to_exclude);
  constraint_set_copy.AddToExcludeNodes(&nodes_to_exclude);

  Links root_path;
  for (size_t i = 0; i < last_path_links.size(); ++i) {
    GraphLinkIndex link_index = last_path_links[i];
    const GraphLink* link = graph_storage->GetLink(link_index);
    GraphNodeIndex spur_node = link->src();
    if (i < start_index) {
      nodes_to_exclude.Insert(spur_node);
      root_path.emplace_back(link_index);
      continue;
    }

    GetLinkExclusionSet(root_path, &links_to_exclude);
    SubGraph new_sub_graph(parent, &constraint_set_copy);

    LinkSequence spur_path = new_sub_graph.ShortestPath(spur_node, dst_);
    if (!spur_path.empty()) {
      const Links& spur_path_links = spur_path.links();

      Links candidate_links = root_path;
      candidate_links.insert(candidate_links.end(), spur_path_links.begin(),
                             spur_path_links.end());
      LinkSequence candidate_path(candidate_links, graph_storage);
      candidates_.emplace(candidate_path, i);
    }

    links_to_exclude.Clear();
    nodes_to_exclude.Insert(spur_node);
    root_path.emplace_back(link_index);
  }

  if (candidates_.empty()) {
    return false;
  }

  PathAndStartIndex min_candidate = candidates_.top();
  candidates_.pop();
  k_paths_.emplace_back(min_candidate);
  return true;
}

LinkSequence KShortestPathsGenerator::KthShortestPath(size_t k) {
  if (k < k_paths_.size()) {
    return k_paths_[k].first;
  }

  size_t delta = k + 1 - k_paths_.size();
  for (size_t i = 0; i < delta; ++i) {
    if (!NextPath()) {
      return {};
    }
  }

  return k_paths_.back().first;
}

// Returns true if prefix_path[0:index] == path[0:index]
static bool HasPrefix(const Links& path, const Links& prefix) {
  CHECK(prefix.size() <= path.size()) << prefix.size() << " vs " << path.size();
  for (size_t i = 0; i < prefix.size(); ++i) {
    if (path[i] != prefix[i]) {
      return false;
    }
  }

  return true;
}

void KShortestPathsGenerator::GetLinkExclusionSet(const Links& root_path,
                                                  GraphLinkSet* out) {
  for (const PathAndStartIndex& k_path_and_index : k_paths_) {
    const LinkSequence& k_path = k_path_and_index.first;
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

}  // namespace nc
}  // namespace ncode
