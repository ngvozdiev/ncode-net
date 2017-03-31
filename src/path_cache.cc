#include "path_cache.h"

#include <ncode/ncode_common/logging.h>
#include <ncode/ncode_common/perfect_hash.h>
#include <algorithm>
#include <chrono>
#include <utility>

namespace nc {
namespace net {

LinkSequence NodePairPathCache::KthShortestPath(
    size_t k, const GraphLinkSet* to_exclude) const {
  std::unique_ptr<ShortestPathGenerator> generator = PathGenerator(to_exclude);

  LinkSequence next_path;
  for (size_t i = 0; i < k + 1; ++i) {
    next_path = generator->NextPath();
    if (next_path.empty()) {
      return next_path;
    }
  }

  return next_path;
}

std::vector<const LinkSequence*> NodePairPathCache::Paths(
    size_t start_k, size_t* next_index, const GraphLinkSet* exclude) {
  std::vector<const LinkSequence*> out;

  // Before starting to enumerate paths will figure out if there exists any path
  // that avoids to_exclude.
  //  LinkSequence shortest_path = KthShortestPath(1, exclude);
  //  if (shortest_path.empty()) {
  //    return {};
  //  }

  size_t i = start_k;
  while (true) {
    const LinkSequence* next_path = GetPathAtIndexOrNull(i++);
    if (next_path == nullptr) {
      return {};
    }

    out.emplace_back(next_path);
    if (exclude && next_path->ContainsAny(*exclude)) {
      continue;
    }

    *next_index = i;
    break;
  }

  return out;
}

std::vector<const LinkSequence*> NodePairPathCache::PathsRange(size_t start_k,
                                                               size_t count) {
  std::vector<const LinkSequence*> out;
  for (size_t i = start_k; i < start_k + count; ++i) {
    const LinkSequence* next_path = GetPathAtIndexOrNull(i);
    if (next_path == nullptr) {
      break;
    }

    out.emplace_back(next_path);
  }

  return out;
}

std::unique_ptr<NodePairPathCache> NodePairPathCache::ExcludeLinks(
    const GraphLinkSet& links) const {
  return std::unique_ptr<NodePairPathCache>(new NodePairPathCache(
      key_, max_num_paths_, constraint_->ExcludeLinks(links), graph_,
      graph_storage_));
}

const LinkSequence* NodePairPathCache::GetPathAtIndexOrNull(size_t i) {
  if (i > max_num_paths_) {
    return nullptr;
  }

  while (i >= paths_.size()) {
    auto next_path = make_unique<LinkSequence>(path_generator_->NextPath());
    if (next_path->empty()) {
      // Ran out of paths before reaching i.
      return nullptr;
    }

    paths_.emplace_back(std::move(next_path));
  }

  return paths_[i].get();
}

std::vector<LinkSequence> NodePairPathCache::PathsKHopsFromShortest(
    size_t k) const {
  size_t shortest_path_hop_count = PathGenerator(nullptr)->NextPath().size();
  size_t limit = shortest_path_hop_count + k;
  return AllPaths(limit);
}

std::vector<LinkSequence> NodePairPathCache::AllPaths(size_t max_hops) const {
  std::vector<LinkSequence> out;

  //  DFS dfs({}, graph_);
  //  dfs.Paths(std::get<0>(key_), std::get<1>(key_), Delay::max(), max_hops,
  //            [this, &out](const LinkSequence& path) {
  //              if (constraint_->PathComplies(path)) {
  //                out.emplace_back(path);
  //              }
  //            });
  std::sort(out.begin(), out.end());
  return out;
}

std::unique_ptr<ShortestPathGenerator> NodePairPathCache::PathGenerator(
    const GraphLinkSet* exclude) const {
  return constraint_->PathGenerator(*graph_, std::get<0>(key_),
                                    std::get<1>(key_), exclude);
}

NodePairPathCache::NodePairPathCache(const NodePair& key, size_t max_num_paths,
                                     std::unique_ptr<Constraint> constraint,
                                     const DirectedGraph* graph,
                                     GraphStorage* path_storage)
    : key_(key),
      graph_(graph),
      graph_storage_(path_storage),
      constraint_(std::move(constraint)),
      max_num_paths_(max_num_paths) {
  path_generator_ = PathGenerator(nullptr);
}

PathCache::PathCache(const GraphLinkSet& links_to_exclude,
                     GraphStorage* graph_storage, size_t max_num_paths_per_pair)
    : graph_(graph_storage),
      graph_storage_(graph_storage),
      links_to_exclude_(links_to_exclude),
      max_num_paths_per_pair_(max_num_paths_per_pair) {}

PathCache::PathCache(GraphStorage* graph_storage, size_t max_num_paths_per_pair,
                     ConstraintMap* constraint_map)
    : PathCache({}, graph_storage, max_num_paths_per_pair) {
  if (constraint_map) {
    for (auto& key_and_constraint : *constraint_map) {
      const NodePair& ie_key = key_and_constraint.first;
      std::unique_ptr<Constraint> constraint =
          std::move(key_and_constraint.second);
      CHECK(constraint) << "No constraint";

      std::unique_ptr<NodePairPathCache>& ie_cache_ptr = ie_caches_[ie_key];
      ie_cache_ptr = std::unique_ptr<NodePairPathCache>(new NodePairPathCache(
          ie_key, max_num_paths_per_pair_, std::move(constraint), &graph_,
          graph_storage_));
    }
  }
}

NodePairPathCache* PathCache::NodePairCache(const NodePair& ie_key) {
  std::unique_ptr<NodePairPathCache>& ie_cache_ptr = ie_caches_[ie_key];
  if (!ie_cache_ptr) {
    ie_cache_ptr = std::unique_ptr<NodePairPathCache>(new NodePairPathCache(
        ie_key, max_num_paths_per_pair_, &graph_, graph_storage_));
    if (!links_to_exclude_.Empty()) {
      ie_cache_ptr = ie_cache_ptr->ExcludeLinks(links_to_exclude_);
    }
  }

  return ie_cache_ptr.get();
}

std::unique_ptr<PathCache> PathCache::ExcludeLinks(
    const GraphLinkSet& links) const {
  GraphLinkSet all_links = links_to_exclude_;
  all_links.InsertAll(links);
  auto out = std::unique_ptr<PathCache>(
      new PathCache(all_links, graph_storage_, max_num_paths_per_pair_));
  for (const auto& node_pair_and_cache : ie_caches_) {
    const NodePair node_pair = node_pair_and_cache.first;
    out->ie_caches_[node_pair] =
        node_pair_and_cache.second->ExcludeLinks(links);
  }

  return out;
}

}  // namespace net
}  // namespace nc
