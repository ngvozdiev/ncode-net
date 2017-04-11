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

bool ExclusionSet::ShouldExcludeLink(const GraphLinkIndex link) const {
  return links_to_exclude_.Contains(link);
}

bool ExclusionSet::ShouldExcludeNode(const GraphNodeIndex node) const {
  return nodes_to_exclude_.Contains(node);
}

void ConstraintSet::AddToVisitSet(const GraphNodeSet& set) {
  size_t set_index = to_visit_.size();
  to_visit_.emplace_back(set);

  for (GraphNodeIndex node_index : set) {
    CHECK(!node_to_visit_index_.HasValue(node_index));
    node_to_visit_index_[node_index] = set_index;
  }
}

bool ConstraintSet::OrderOk(const Links& links,
                            const GraphStorage* graph_storage) const {
  size_t current_index = -1;
  for (GraphLinkIndex link : links) {
    const GraphLink* link_ptr = graph_storage->GetLink(link);
    GraphNodeIndex src = link_ptr->src();
    GraphNodeIndex dst = link_ptr->dst();

    if (node_to_visit_index_.HasValue(src)) {
      size_t i = node_to_visit_index_.GetValueOrDie(src);
      if (current_index != i && (current_index + 1) != i) {
        return false;
      }

      current_index = i;
    }

    if (node_to_visit_index_.HasValue(dst)) {
      size_t i = node_to_visit_index_.GetValueOrDie(dst);
      if (current_index != i && (current_index + 1) != i) {
        return false;
      }

      current_index = i;
    }
  }

  return current_index == (to_visit_.size() - 1ul);
}

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
    if (!constraints_->OrderOk(*current, parent_->graph_storage())) {
      return;
    }

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
    if (constraints_->ShouldExcludeLink(out_link_info.link_index)) {
      continue;
    }

    GraphNodeIndex next_hop = out_link_info.dst_index;
    if (constraints_->ShouldExcludeNode(next_hop)) {
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
    if (constraints_->ShouldExcludeLink(out_link_info.link_index)) {
      continue;
    }

    GraphNodeIndex next_hop = out_link_info.dst_index;
    if (constraints_->ShouldExcludeNode(next_hop)) {
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

static bool CanExcludeNode(GraphNodeIndex node, const ExclusionSet& constraints,
                           const GraphNodeSet* additional_nodes_to_avoid) {
  if (additional_nodes_to_avoid != nullptr &&
      additional_nodes_to_avoid->Contains(node)) {
    return true;
  }

  return constraints.ShouldExcludeNode(node);
}

static bool CanExcludeLink(GraphLinkIndex link, const ExclusionSet& constraints,
                           const GraphLinkSet* additional_links_to_avoid) {
  if (additional_links_to_avoid != nullptr &&
      additional_links_to_avoid->Contains(link)) {
    return true;
  }

  return constraints.ShouldExcludeLink(link);
}

void ShortestPath::ComputePaths(const ExclusionSet& exclusion_set,
                                const AdjacencyList& adj_list,
                                const GraphNodeSet* additional_nodes_to_avoid,
                                const GraphLinkSet* additional_links_to_avoid) {
  using DelayAndIndex = std::pair<Delay, GraphNodeIndex>;
  std::priority_queue<DelayAndIndex, std::vector<DelayAndIndex>,
                      std::greater<DelayAndIndex>> vertex_queue;

  min_delays_.Resize(adj_list.AllNodes().Count());
  previous_.Resize(adj_list.AllNodes().Count());

  if (CanExcludeNode(src_, exclusion_set, additional_nodes_to_avoid)) {
    return;
  }

  min_delays_[src_].distance = Delay::zero();
  vertex_queue.emplace(Delay::zero(), src_);

  size_t destinations_remaining = destinations_.Count();
  while (!vertex_queue.empty()) {
    Delay distance;
    GraphNodeIndex current;
    std::tie(distance, current) = vertex_queue.top();
    vertex_queue.pop();

    if (distance > min_delays_[current].distance) {
      // Bogus leftover node, since we never delete nodes from the heap.
      continue;
    }

    if (destinations_.Contains(current)) {
      --destinations_remaining;
      if (destinations_remaining == 0) {
        break;
      }
    }

    const std::vector<AdjacencyList::LinkInfo>& neighbors =
        adj_list.GetNeighbors(current);
    for (const AdjacencyList::LinkInfo& out_link_info : neighbors) {
      GraphLinkIndex out_link = out_link_info.link_index;
      if (CanExcludeLink(out_link, exclusion_set, additional_links_to_avoid)) {
        continue;
      }

      GraphNodeIndex neighbor_node = out_link_info.dst_index;
      if (CanExcludeNode(neighbor_node, exclusion_set,
                         additional_nodes_to_avoid)) {
        continue;
      }

      const Delay link_delay = out_link_info.delay;
      const Delay distance_via_neighbor = distance + link_delay;
      Delay& curr_min_distance = min_delays_[neighbor_node].distance;

      if (distance_via_neighbor < curr_min_distance) {
        curr_min_distance = distance_via_neighbor;
        previous_[neighbor_node] = &out_link_info;
        vertex_queue.emplace(curr_min_distance, neighbor_node);
      }
    }
  }
}

net::LinkSequence AllPairShortestPath::GetPath(GraphNodeIndex src,
                                               GraphNodeIndex dst) const {
  Delay dist = data_[src][dst].distance;
  if (dist == Delay::max()) {
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

void AllPairShortestPath::ComputePaths(
    const ExclusionSet& exclusion_set, const AdjacencyList& adj_list,
    const GraphNodeSet* additional_nodes_to_avoid,
    const GraphLinkSet* additional_links_to_avoid) {
  const GraphNodeSet nodes = adj_list.AllNodes();
  for (GraphNodeIndex node : nodes) {
    if (CanExcludeNode(node, exclusion_set, additional_nodes_to_avoid)) {
      continue;
    }

    SPData& node_data = data_[node][node];
    node_data.distance = Delay::zero();
  }

  for (const auto& node_and_neighbors : adj_list.Adjacencies()) {
    for (const AdjacencyList::LinkInfo& link_info :
         *node_and_neighbors.second) {
      GraphLinkIndex link = link_info.link_index;
      if (CanExcludeLink(link, exclusion_set, additional_links_to_avoid)) {
        continue;
      }

      Delay distance = link_info.delay;
      SPData& sp_data = data_[link_info.src_index][link_info.dst_index];
      sp_data.distance = distance;
      sp_data.next_link = link;
      sp_data.next_node = link_info.dst_index;
    }
  }

  for (GraphNodeIndex k : nodes) {
    for (GraphNodeIndex i : nodes) {
      for (GraphNodeIndex j : nodes) {
        Delay i_k = data_[i][k].distance;
        Delay k_j = data_[k][j].distance;

        bool any_max = (i_k == Delay::max() || k_j == Delay::max());
        Delay alt_distance = any_max ? Delay::max() : i_k + k_j;

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

std::pair<GraphNodeSet, GraphLinkSet> ShortestPath::ElementsInTree() const {
  GraphNodeSet nodes;
  GraphLinkSet links;

  for (const auto& node_and_info : previous_) {
    const AdjacencyList::LinkInfo* link_info = *node_and_info.second;
    nodes.Insert(link_info->src_index);
    nodes.Insert(link_info->dst_index);
    links.Insert(link_info->link_index);
  }

  return {nodes, links};
}

static LinkSequence ShortestPathStatic(GraphNodeIndex src, GraphNodeIndex dst,
                                       const GraphNodeSet& nodes_to_avoid,
                                       const GraphLinkSet& links_to_avoid,
                                       const SubGraph& sub_graph,
                                       SubGraphShortestPathState* sp_state) {
  const ConstraintSet& constraints = *sub_graph.constraints();
  const AdjacencyList& adj_list = sub_graph.parent()->AdjacencyList();
  const std::vector<GraphNodeSet>& to_visit = constraints.to_visit();
  if (to_visit.empty()) {
    net::ShortestPath sp_tree(src, {dst}, constraints.exclusion_set(), adj_list,
                              &nodes_to_avoid, &links_to_avoid);
    return sp_tree.GetPath(dst);
  }

  // Clean up previous state.
  sp_state->Clear();

  // Nodes to exclude.
  GraphNodeSet& to_exclude = sp_state->to_exclude;

  // The adjacency list for the graph that is created from shortest paths from
  // each node in 'to_visit' to destinations.
  AdjacencyList& path_graph_adj_list = sp_state->path_graph_adj_list;

  // Generates sequential link indices.
  size_t path_graph_link_index_gen = -1;

  // Shortest path trees.
  GraphNodeMap<std::unique_ptr<net::ShortestPath>> sp_trees;

  // Maps a pair of src, dst with the link that represents the shortest path
  // between them in the new graph. The link index is not into the original
  // graph (from graph_storage) but one of the ones generated by
  // path_graph_link_index_gen.
  GraphLinkMap<std::pair<GraphNodeIndex, GraphNodeIndex>>& link_map =
      sp_state->link_map;

  // Will not allow the front/back to contain the src/dst, as it makes it easier
  // to reason about order.
  CHECK(!to_visit.front().Contains(src));
  CHECK(!to_visit.back().Contains(dst));

  for (size_t i = -1; i != to_visit.size(); ++i) {
    LOG(ERROR) << "I " << i;

    // For each set we will compute the SP trees rooted at each node. Each of
    // those SP trees should avoid nodes from other sets, except for the next
    // set.
    to_exclude.Clear();
    for (size_t j = 0; j < to_visit.size(); ++j) {
      if (i != j && (i + 1) != j) {
        to_exclude.InsertAll(to_visit[j]);
      }
    }

    GraphNodeSet destinations;
    if (i == to_visit.size() - 1) {
      destinations.Insert(dst);
    } else {
      destinations.InsertAll(to_visit[i + 1]);
    }

    if (last_set_contains_dst) {
      if (i != to_visit.size() - 1 && i != to_visit.size() - 2) {
        to_exclude.Insert(dst);
      }
    } else {
      if (i != to_visit.size() - 1) {
        to_exclude.Insert(dst);
      }
    }

    GraphNodeSet sources;
    if (i == static_cast<size_t>(-1)) {
      sources.Insert(src);
    } else {
      const GraphNodeSet& set_to_visit = to_visit[i];
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

    CHECK(!sources.Empty());
    CHECK(!destinations.Empty());
    to_exclude.InsertAll(nodes_to_avoid);

    bool no_path_found = true;
    for (GraphNodeIndex node_to_visit : sources) {
      LOG(ERROR) << "Will run SP rooted at " << node_to_visit;
      const net::ShortestPath* tree;
      auto new_tree = make_unique<net::ShortestPath>(
          node_to_visit, destinations, constraints.exclusion_set(), adj_list,
          &to_exclude, &links_to_avoid);
      tree = new_tree.get();
      sp_trees[node_to_visit] = std::move(new_tree);

      for (GraphNodeIndex destination : destinations) {
        GraphLinkIndex new_link_index(++path_graph_link_index_gen);

        Delay sp_delay = tree->GetPathDistance(destination);
        if (sp_delay == Delay::max()) {
          //          LOG(ERROR) << "No path found";
          continue;
        }

        path_graph_adj_list.AddLink(new_link_index, node_to_visit, destination,
                                    sp_delay);
        link_map[new_link_index] = {node_to_visit, destination};
        LOG(ERROR) << "SP " << node_to_visit << " -> " << destination << " li "
                   << new_link_index << " delay " << sp_delay.count() << " => "
                   << tree->GetPath(destination)
                          .ToStringNoPorts(sub_graph.parent()->graph_storage());
        no_path_found = false;
      }
    }

    if (no_path_found) {
      return {};
    }
  }

  // Now we can find out the shortest path through the new graph, the links of
  // which will tell us which paths we need to stitch together in order to form
  // the final end-to-end path.
  ExclusionSet dummy;
  net::ShortestPath path_graph_sp(src, {dst}, dummy, path_graph_adj_list,
                                  nullptr, nullptr);
  LinkSequence sp = path_graph_sp.GetPath(dst);

  Links final_path;
  Delay total_delay = Delay::zero();
  for (GraphLinkIndex path_graph_link : sp.links()) {
    GraphNodeIndex sub_path_from;
    GraphNodeIndex sub_path_to;
    std::tie(sub_path_from, sub_path_to) = link_map[path_graph_link];
    LOG(ERROR) << "Subpath from " << sub_path_from << " to " << sub_path_to;

    const net::ShortestPath* sp_tree =
        sp_trees.GetValueOrDie(sub_path_from).get();
    LinkSequence sub_path = sp_tree->GetPath(sub_path_to);
    final_path.insert(final_path.end(), sub_path.links().begin(),
                      sub_path.links().end());
    total_delay += sub_path.delay();
  }

  return {final_path, total_delay};
}

LinkSequence SubGraph::ShortestPath(GraphNodeIndex src,
                                    GraphNodeIndex dst) const {
  SubGraphShortestPathState sp_state;
  return ShortestPathStatic(src, dst, {}, {}, *this, &sp_state);
}

LinkSequence KShortestPathsGenerator::ShortestPath(
    GraphNodeIndex src, GraphNodeIndex dst, const GraphNodeSet& nodes_to_avoid,
    const GraphLinkSet& links_to_avoid) {
  return ShortestPathStatic(src, dst, nodes_to_avoid, links_to_avoid,
                            *sub_graph_, &sp_state_);
}

bool KShortestPathsGenerator::NextPath() {
  const DirectedGraph* parent = sub_graph_->parent();
  const GraphStorage* graph_storage = parent->graph_storage();

  if (k_paths_.empty()) {
    LinkSequence path = ShortestPath(src_, dst_, {}, {});
    k_paths_trie_.Add(path.links(), 0);
    k_paths_.emplace_back(path, 0);
    return true;
  }

  const PathAndStartIndex& last_path_and_start_index = k_paths_.back();
  const LinkSequence& last_path = last_path_and_start_index.first;
  const Links& last_path_links = last_path.links();
  size_t start_index = last_path_and_start_index.second;

  GraphLinkSet links_to_exclude;
  GraphNodeSet nodes_to_exclude;

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
    LinkSequence spur_path =
        ShortestPath(spur_node, dst_, nodes_to_exclude, links_to_exclude);

    if (!spur_path.empty()) {
      const Links& spur_path_links = spur_path.links();

      Links candidate_links = root_path;
      candidate_links.insert(candidate_links.end(), spur_path_links.begin(),
                             spur_path_links.end());
      LinkSequence candidate_path(candidate_links, graph_storage);
      candidates_.emplace(candidate_path, i);
    }

    //    links_to_exclude.Clear();
    nodes_to_exclude.Insert(spur_node);
    root_path.emplace_back(link_index);
  }

  if (candidates_.empty()) {
    return false;
  }

  PathAndStartIndex min_candidate = candidates_.top();
  candidates_.pop();
  k_paths_trie_.Add(min_candidate.first.links(), k_paths_.size());
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

void KShortestPathsGenerator::GetLinkExclusionSet(const Links& root_path,
                                                  GraphLinkSet* out) {
  if (root_path.empty()) {
    for (const auto& path : k_paths_) {
      const Links& k_path_links = path.first.links();
      out->Insert(k_path_links[0]);
    }

    return;
  }

  const std::vector<uint32_t>& paths_with_same_prefix =
      k_paths_trie_.SequencesWithPrefix(root_path);
  for (uint32_t k : paths_with_same_prefix) {
    const Links& k_path_links = k_paths_[k].first.links();
    CHECK(k_path_links.size() > root_path.size());
    out->Insert(k_path_links[root_path.size()]);
  }
}

}  // namespace nc
}  // namespace ncode
