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

size_t ConstraintSet::MinVisit(const Links& links,
                               const GraphStorage* graph_storage) const {
//  LOG(ERROR) << "AAAA";
  int current_index = -1;
  if (links.empty()) {
    return 0;
  }

  const GraphLink* link_ptr = graph_storage->GetLink(links.front());
  GraphNodeIndex path_src = link_ptr->src();
  if (node_to_visit_index_.HasValue(path_src)) {
    int i = node_to_visit_index_.GetValueOrDie(path_src);
//    LOG(ERROR) << "I " << i << " CI " << current_index;
    if (current_index != i && (current_index + 1) != i) {
      return current_index + 1;
    }

    current_index = i;
  }

  for (GraphLinkIndex link : links) {
    const GraphLink* link_ptr = graph_storage->GetLink(link);
    GraphNodeIndex dst = link_ptr->dst();

    if (node_to_visit_index_.HasValue(dst)) {
      int i = node_to_visit_index_.GetValueOrDie(dst);
//      LOG(ERROR) << "I " << i << " CI " << current_index;
      if (i < current_index) {
        return i + 1;
      }

      if (current_index != i && (current_index + 1) != i) {
        return current_index + 1;
      }

      current_index = i;
    }
  }

  return current_index + 1;
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
                     PathCallback path_callback,
                     const DFSConfig& config) const {
  Delay total_distance = Delay::zero();
  GraphLinkSet links_seen;
  GraphNodeSet nodes_seen;
  Links scratch_path;
  PathsRecursive(config, src, dst, path_callback, &links_seen, &nodes_seen,
                 &scratch_path, &total_distance);
}

void SubGraph::PathsRecursive(const DFSConfig& config, GraphNodeIndex at,
                              GraphNodeIndex dst, PathCallback path_callback,
                              GraphLinkSet* links_seen,
                              GraphNodeSet* nodes_seen, Links* current,
                              Delay* total_distance) const {
  if (current->size() > config.max_hops) {
    return;
  }

  if (at == dst) {
    size_t min_v = constraints_->MinVisit(*current, parent_->graph_storage());
    if (min_v != constraints_->to_visit().size()) {
      return;
    }

//    LOG(ERROR) << "min v " << min_v;
    path_callback(LinkSequence(*current, *total_distance));
    return;
  }

  Delay min_distance = *total_distance;
  if (min_distance > config.max_distance) {
    return;
  }

  const AdjacencyList& adjacency_list = parent_->AdjacencyList();
  const std::vector<AdjacencyList::LinkInfo>& outgoing_links =
      adjacency_list.GetNeighbors(at);

  if (config.simple) {
    if (nodes_seen->Contains(at)) {
      return;
    }
    nodes_seen->Insert(at);
  }

  for (const AdjacencyList::LinkInfo& out_link_info : outgoing_links) {
    GraphLinkIndex link_index = out_link_info.link_index;
    if (constraints_->ShouldExcludeLink(link_index)) {
      continue;
    }

    GraphNodeIndex next_hop = out_link_info.dst_index;
    if (constraints_->ShouldExcludeNode(next_hop)) {
      continue;
    }

    if (!config.simple) {
      if (links_seen->Contains(link_index)) {
        continue;
      }
      links_seen->Insert(link_index);
    }

    current->push_back(link_index);
    *total_distance += out_link_info.delay;
    PathsRecursive(config, next_hop, dst, path_callback, links_seen, nodes_seen,
                   current, total_distance);
    *total_distance -= out_link_info.delay;
    current->pop_back();

    if (!config.simple) {
      links_seen->Remove(link_index);
    }
  }

  if (config.simple) {
    nodes_seen->Remove(at);
  }
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

using VisitList = std::vector<GraphNodeSet>;

static LinkSequence ShortestPathStatic(
    const GraphNodeIndex src, const GraphNodeIndex dst,
    const GraphNodeSet& nodes_to_avoid, const GraphLinkSet& links_to_avoid,
    const ExclusionSet& exclusion_set, VisitList::const_iterator to_visit_from,
    VisitList::const_iterator to_visit_to, const AdjacencyList& adj_list,
    SubGraphShortestPathState* sp_state, const GraphStorage* graph_storage,
    bool explore_alt_on_duplicate) {
  size_t to_visit_count = std::distance(to_visit_from, to_visit_to);
  if (to_visit_count == 0) {
    net::ShortestPath sp_tree(src, {dst}, exclusion_set, adj_list,
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
  GraphNodeMap<std::unique_ptr<net::ShortestPath>>& sp_trees =
      sp_state->sp_trees;

  // Maps a pair of src, dst with the link that represents the shortest path
  // between them in the new graph. The link index is not into the original
  // graph (from graph_storage) but one of the ones generated by
  // path_graph_link_index_gen.
  GraphLinkMap<std::pair<GraphNodeIndex, GraphNodeIndex>>& link_map =
      sp_state->link_map;

  // Will assume that the front/back do not contain the src/dst, as it makes it
  // easier to reason about order. This is enforced elsewhere.
  for (size_t i = -1; i != to_visit_count; ++i) {
    // For each set we will compute the SP trees rooted at each node. Each of
    // those SP trees should avoid nodes from other sets, except for the next
    // set.
    to_exclude.Clear();
    for (size_t j = 0; j < to_visit_count; ++j) {
      if (i != j && (i + 1) != j) {
        to_exclude.InsertAll(*std::next(to_visit_from, j));
      }
    }

    if (i != to_visit_count - 1) {
      to_exclude.Insert(dst);
    }

    GraphNodeSet destinations;
    if (i == to_visit_count - 1) {
      destinations.Insert(dst);
    } else {
      destinations.InsertAll(*std::next(to_visit_from, i + 1));
    }

    GraphNodeSet sources;
    if (i == static_cast<size_t>(-1)) {
      sources.Insert(src);
    } else {
      const GraphNodeSet& set_to_visit = *std::next(to_visit_from, i);
      sources.InsertAll(set_to_visit);
    }

    CHECK(!sources.Empty());
    CHECK(!destinations.Empty());
    to_exclude.InsertAll(nodes_to_avoid);

    bool no_path_found = true;
    for (GraphNodeIndex node_to_visit : sources) {
      std::unique_ptr<net::ShortestPath>& tree = sp_trees[node_to_visit];
      tree = make_unique<net::ShortestPath>(node_to_visit, destinations,
                                            exclusion_set, adj_list,
                                            &to_exclude, &links_to_avoid);

      for (GraphNodeIndex destination : destinations) {
        GraphLinkIndex new_link_index(++path_graph_link_index_gen);

        Delay sp_delay = tree->GetPathDistance(destination);
        if (sp_delay == Delay::max()) {
          continue;
        }

        path_graph_adj_list.AddLink(new_link_index, node_to_visit, destination,
                                    sp_delay);
        link_map[new_link_index] = {node_to_visit, destination};
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
    //    LOG(ERROR) << "Subpath from " << sub_path_from << " to " <<
    //    sub_path_to;

    const net::ShortestPath* sp_tree =
        sp_trees.GetValueOrDie(sub_path_from).get();
    LinkSequence sub_path = sp_tree->GetPath(sub_path_to);
    final_path.insert(final_path.end(), sub_path.links().begin(),
                      sub_path.links().end());
    total_delay += sub_path.delay();
  }

  if (HasDuplicateLinks(final_path)) {
    if (!explore_alt_on_duplicate) {
      return {};
    }

    // If the path contains a duplicate link the crude way to try to get off the
    // hook is to exclude all links from the path in order, recursively call
    // this function and pick the shortest path (if any).
    Delay best_alt_path_delay = Delay::max();
    LinkSequence best_alt_path = {};
    for (GraphLinkIndex link_to_exclude : final_path) {
      GraphLinkSet alt_links_to_avoid = links_to_avoid;
      alt_links_to_avoid.Insert(link_to_exclude);
      LinkSequence alt_path = ShortestPathStatic(
          src, dst, nodes_to_avoid, alt_links_to_avoid, exclusion_set,
          to_visit_from, to_visit_to, adj_list, sp_state, graph_storage, false);
      if (!alt_path.empty() && alt_path.delay() < best_alt_path_delay) {
        best_alt_path = alt_path;
        best_alt_path_delay = alt_path.delay();
      }
    }

    return best_alt_path;
  }

  return {final_path, total_delay, false};
}

LinkSequence SubGraph::ShortestPath(GraphNodeIndex src,
                                    GraphNodeIndex dst) const {
  SubGraphShortestPathState sp_state;
  ConstraintSet sanitized_constraints =
      constraints_->SanitizeConstraints(src, dst);

  return ShortestPathStatic(
      src, dst, {}, {}, sanitized_constraints.exclusion_set(),
      sanitized_constraints.to_visit().begin(),
      sanitized_constraints.to_visit().end(), parent()->AdjacencyList(),
      &sp_state, parent()->graph_storage(), true);
}

LinkSequence KShortestPathsGenerator::ShortestPath(
    GraphNodeIndex src, GraphNodeIndex dst, const GraphNodeSet& nodes_to_avoid,
    const GraphLinkSet& links_to_avoid, const Links& links_so_far) {
  size_t to_visit_index =
      constraints_.MinVisit(links_so_far, graph_->graph_storage());
  //  LOG(ERROR) << "KSP from " << to_visit_index << " links_so_far "
  //             << LinksToString(links_so_far, graph_->graph_storage());
  const VisitList& to_visit = constraints_.to_visit();
  auto start_it = std::next(to_visit.begin(), to_visit_index);

  return ShortestPathStatic(src, dst, nodes_to_avoid, links_to_avoid,
                            constraints_.exclusion_set(), start_it,
                            to_visit.end(), graph_->AdjacencyList(), &sp_state_,
                            graph_->graph_storage(), true);
}

bool KShortestPathsGenerator::NextPath() {
  const GraphStorage* graph_storage = graph_->graph_storage();

  if (k_paths_.empty()) {
    LinkSequence path = ShortestPath(src_, dst_, {}, {}, {});
    if (path.empty()) {
      return false;
    }

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

  //  LOG(ERROR) << "Last path " << last_path.ToStringNoPorts(graph_storage);

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
    LinkSequence spur_path = ShortestPath(spur_node, dst_, nodes_to_exclude,
                                          links_to_exclude, root_path);
    //    LOG(ERROR) << "Spur " << graph_storage->GetNode(spur_node)->id()
    //               << " to exclude "
    //               << GraphLinkSetToString(links_to_exclude, graph_storage)
    //               << " spur path " <<
    //               spur_path.ToStringNoPorts(graph_storage);

    if (!spur_path.empty()) {
      const Links& spur_path_links = spur_path.links();

      Links candidate_links = root_path;
      candidate_links.insert(candidate_links.end(), spur_path_links.begin(),
                             spur_path_links.end());
      LinkSequence candidate_path(candidate_links, graph_storage);
      //      CLOG(ERROR, BLUE) << "Candidate "
      //                        <<
      //                        candidate_path.ToStringNoPorts(graph_storage);
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
  //  CLOG(ERROR, YELLOW) << "Produced "
  //                      << min_candidate.first.ToStringNoPorts(graph_storage);

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
