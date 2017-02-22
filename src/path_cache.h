#ifndef NCODE_NET_PATH_CACHE_H
#define NCODE_NET_PATH_CACHE_H

#include <ncode/ncode_common/common.h>
#include <stddef.h>
#include <cstdint>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include "algorithm.h"
#include "constraint.h"
#include "net_common.h"

namespace nc {
namespace net {

using NodePair = std::tuple<GraphNodeIndex, GraphNodeIndex, uint64_t>;

// Caches paths between two nodes in the graph.
class NodePairPathCache {
 public:
  // Returns the paths between start_k (including) and the first  path that
  // complies with 'exclude'. This function will also populate 'next_index' with
  // the index after that of the path that complies.  Start_k  and next_index
  // are indices into the sorted list of all paths not only the  ones that
  // comply. This function will enumerate all paths until it finds a compliant
  // one and may be very slow if there is no compliant path and the graph is
  // large. In this case and empty vector will be returned.
  std::vector<const LinkSequence*> Paths(size_t start_k, size_t* next_index,
                                         const GraphLinkSet* exclude = nullptr);

  // Returns a range of the k shortest paths sequence that contains count paths
  // starting at start_k.
  std::vector<const LinkSequence*> PathsRange(size_t start_k, size_t count);

  // Will return the Kth lowest delay path. If 'exclude' is specified will
  // return the lowest delay compliant path -- the one that does not cross any
  // of the links to exclude. This (and the following) methods are different
  // from Paths which will enumerate all paths up to the compliant one and put
  // them in the cache.
  LinkSequence KthShortestPath(size_t k,
                               const GraphLinkSet* exclude = nullptr) const;

  // Will return the lowest delay path (P) and any paths that are up to
  // hop_count(P) + k hops long.
  std::vector<LinkSequence> PathsKHopsFromShortest(size_t k) const;

  // Caches all paths between the source and the destination. All paths up to
  // 'max_hops' will be enumerated. This will be slow.
  std::vector<LinkSequence> AllPaths(size_t max_hops) const;

  // Return a NodePair cache that will exclude the given links.
  std::unique_ptr<NodePairPathCache> ExcludeLinks(
      const GraphLinkSet& links) const;

 private:
  NodePairPathCache(const NodePair& key, size_t max_num_paths,
                    std::unique_ptr<Constraint> constraint,
                    const DirectedGraph* graph, GraphStorage* graph_storage);

  NodePairPathCache(const NodePair& key, size_t max_num_paths,
                    const DirectedGraph* graph, GraphStorage* graph_storage)
      : NodePairPathCache(key, max_num_paths, DummyConstraint(), graph,
                          graph_storage) {}

  std::unique_ptr<ShortestPathGenerator> PathGenerator(
      const GraphLinkSet* exclude) const;

  // Returns the path at index i from the cache, or if the cache does not extend
  // up to index i, caches all paths up to, and including i. Will return empty
  // path if no path at index i exists.
  const LinkSequence* GetPathAtIndexOrNull(size_t i);

  const NodePair key_;
  const DirectedGraph* graph_;
  GraphStorage* graph_storage_;

  // Constraint.
  std::unique_ptr<Constraint> constraint_;

  // Generates paths in order.
  std::unique_ptr<ShortestPathGenerator> path_generator_;

  // Paths, ordered in increasing delay. All of these satisfy constraint_, but
  // do not exclude any links (i.e. they are generated as if calling
  // GetKLowestDelayPaths with a very large K and no to_exclude).
  std::vector<std::unique_ptr<LinkSequence>> paths_;

  // The cache will not grow above this number of paths.
  size_t max_num_paths_;

  friend class PathCache;
  DISALLOW_COPY_AND_ASSIGN(NodePairPathCache);
};

// An entity that can be queried for paths and will cache paths between a source
// and a destination.
class PathCache {
 public:
  static constexpr size_t kDefaultMaxNumPathsPerPair = 1000;

  using ConstraintMap = std::map<NodePair, std::unique_ptr<Constraint>>;

  // Creates a new cache.
  PathCache(GraphStorage* path_storage,
            size_t max_num_paths_per_pair = kDefaultMaxNumPathsPerPair,
            ConstraintMap* constraint_map = nullptr);

  // The graph.
  const DirectedGraph* graph() const { return &graph_; }

  // Path storage.
  GraphStorage* graph_storage() { return graph_storage_; }

  // Returns the cache between two nodes.
  NodePairPathCache* NodePairCache(const NodePair& key);

  // Returns a new path cache with constraints that exclude the given links. The
  // cache will contain no paths.
  std::unique_ptr<PathCache> ExcludeLinks(const GraphLinkSet& links) const;

 private:
  PathCache(const GraphLinkSet& links_to_exclude, GraphStorage* graph_storage,
            size_t max_num_paths_per_pair);

  const DirectedGraph graph_;
  GraphStorage* graph_storage_;

  // All NodePairPathCache instances that this cache generates will have
  // constraints that exclude these links.
  const GraphLinkSet links_to_exclude_;

  // Each NodePair cache will have at most this many paths.
  size_t max_num_paths_per_pair_;

  // Stores cached between a source and a destination.
  std::map<NodePair, std::unique_ptr<NodePairPathCache>> ie_caches_;

  DISALLOW_COPY_AND_ASSIGN(PathCache);
};

}  // namespace nc
}  // namespace ncode

#endif /* NCODE_NET_PATH_CACHE */
