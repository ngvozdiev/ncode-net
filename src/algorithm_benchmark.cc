#include <chrono>
#include <ncode/ncode_common/substitute.h>

#include "net.pb.h"
#include "algorithm.h"
#include "net_common.h"
#include "net_gen.h"

using namespace nc;
using namespace std::chrono;

static void TimeMs(const std::string& msg, std::function<void()> f) {
  auto start = high_resolution_clock::now();
  f();
  auto end = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(end - start);
  LOG(INFO) << msg << ": " << duration.count() << "ms";
}

static void TimeToString(std::string* out, uint32_t id, uint32_t x,
                         std::function<void()> f) {
  auto start = high_resolution_clock::now();
  f();
  auto end = high_resolution_clock::now();
  auto duration = duration_cast<nanoseconds>(end - start);
  SubstituteAndAppend(out, "$0 $1 $2\n", id, x, duration.count());
}

net::GraphNodeSet RandomSample(const net::GraphStorage& graph, size_t count,
                               std::mt19937* rnd) {
  std::vector<net::GraphNodeIndex> all_nodes;
  for (net::GraphNodeIndex node : graph.AllNodes()) {
    all_nodes.emplace_back(node);
  }

  std::shuffle(all_nodes.begin(), all_nodes.end(), *rnd);
  all_nodes.resize(std::min(all_nodes.size(), count));

  net::GraphNodeSet to_return;
  for (net::GraphNodeIndex node : all_nodes) {
    to_return.Insert(node);
  }

  return to_return;
}

size_t CountSimplePaths(const std::vector<net::LinkSequence>& paths,
                        const net::GraphStorage* storage) {
  size_t i = 0;
  for (const auto& path : paths) {
    if (!path.HasDuplicateNodes(storage)) {
      ++i;
    }
  }

  return i;
}

// Compares two lists of paths and returns true if the first count simple paths
// from both are the same.
static double Compare(const std::vector<net::LinkSequence>& paths_one,
                      const std::vector<net::LinkSequence>& paths_two,
                      size_t count, const net::GraphStorage* storage) {
  CHECK(paths_one.size() >= count);
  CHECK(paths_two.size() >= count);
  double total_delta = 0;

  size_t i_one = 0;
  size_t i_two = 0;
  for (size_t i = 0; i < count; ++i) {
    const net::LinkSequence* ls_one;
    while (true) {
      CHECK(paths_one.size() > i_one);
      ls_one = &paths_one[i_one++];
      if (!ls_one->HasDuplicateNodes(storage)) {
        break;
      }

      //      LOG(ERROR) << "Skipping1 " << ls_one->ToStringNoPorts(storage);
    }

    const net::LinkSequence* ls_two;
    while (true) {
      CHECK(paths_two.size() > i_two);
      ls_two = &paths_two[i_two++];
      if (!ls_two->HasDuplicateNodes(storage)) {
        break;
      }

      //      LOG(ERROR) << "Skipping2 " << ls_two->ToStringNoPorts(storage);
    }

    double delta = std::abs(ls_one->delay().count() - ls_two->delay().count());

    total_delta += delta;
    //    if (*ls_one != *ls_two) {
    //      CLOG(ERROR, RED) << ls_one->ToStringNoPorts(storage) << " vs "
    //                       << ls_two->ToStringNoPorts(storage);
    //    } else {
    //      CLOG(ERROR, GREEN) << ls_one->ToStringNoPorts(storage) << " vs "
    //                         << ls_two->ToStringNoPorts(storage);
    //    }
  }

  return total_delta / count;
}

std::string SetToString(const net::GraphLinkSet& set) {
  std::vector<std::string> out;
  for (net::GraphLinkIndex link : set) {
    out.emplace_back(std::to_string(link));
  }

  return nc::Join(out, ",");
}

net::GraphNodeSet NodesInPath(const net::LinkSequence& path,
                              const net::GraphStorage* storage) {
  net::GraphNodeSet out;
  for (const auto& link : path.links()) {
    const net::GraphLink* link_ptr = storage->GetLink(link);
    out.Insert(link_ptr->src());
    out.Insert(link_ptr->dst());
  }

  return out;
}

void PrintPathStats(net::GraphNodeIndex src, net::GraphNodeIndex dst,
                    const std::vector<net::LinkSequence>& paths,
                    const net::DirectedGraph* graph) {
  size_t edge_count = 0;
  size_t delta_edge_count = paths[0].links().size();
  net::GraphLinkSet all_edges;
  size_t max_len = 0;
  for (size_t i = 0; i < paths.size() - 1; ++i) {
    const net::LinkSequence& path = paths[i];
    const net::LinkSequence& next_path = paths[i + 1];

    net::GraphLinkSet links_one(path.links());
    all_edges.InsertAll(links_one);
    net::GraphLinkSet links_two(next_path.links());
    net::GraphLinkSet new_links = links_one.Difference(links_two);

    edge_count += path.links().size();
    delta_edge_count += new_links.Count();

    max_len = std::max(max_len, path.links().size());
    LOG(ERROR) << path.ToStringNoPorts(graph->graph_storage()) << " ed "
               << new_links.Count() << " " << SetToString(new_links) << " "
               << i;
  }

  net::GraphLinkSet links_last(paths.back().links());
  all_edges.InsertAll(links_last);

  edge_count += paths.back().links().size();

  LOG(ERROR) << "Edge count " << edge_count << " path count " << paths.size()
             << " edge delta " << delta_edge_count << " unique edges "
             << all_edges.Count();

  net::GraphLinkSet edges_in_graph = graph->graph_storage()->AllLinks();
  edges_in_graph.RemoveAll(all_edges);

  net::ConstraintSet constraints;
  constraints.Exclude().Links(edges_in_graph);

  auto start = high_resolution_clock::now();
  std::vector<net::LinkSequence> paths_recovered;
  net::SubGraph sub_graph(graph, &constraints);
  sub_graph.Paths(src, dst, [&paths_recovered](const net::LinkSequence& links) {
    paths_recovered.emplace_back(links);
  }, true, paths.back().delay(), max_len);
  auto done = high_resolution_clock::now();
  std::sort(paths_recovered.begin(), paths_recovered.end());

  LOG(ERROR) << "PR " << paths_recovered.size() << " in "
             << duration_cast<microseconds>(done - start).count();
}

static bool Unique(const std::vector<net::LinkSequence>& paths) {
  std::set<net::LinkSequence> paths_set(paths.begin(), paths.end());
  return paths_set.size() == paths.size();
}

static net::GraphNodeMap<net::GraphLinkIndex> GetDag(
    const std::vector<net::LinkSequence>& paths,
    const net::GraphStorage* storage) {
  net::GraphNodeMap<net::GraphLinkIndex> out;
  for (const auto& path : paths) {
    const net::Links& links = path.links();
    for (const auto& link : links) {
    }
  }
}

//
//// Compressed sequence of paths.
// class PackedPathSequence {
// public:
//  PackedPathSequence(const std::vector<net::LinkSequence>& paths) {
//    Pack(paths);
//  }
//
// private:
//  // A packed path is a set of links that need to be added to another path to
//  // get the original one.
//  struct PackedPath {
//    PackedPath(size_t path, size_t path_from, const net::Links& links)
//        : path(path), path_from(path_from), links(links) {}
//
//    size_t path;
//    size_t path_from;
//    net::Links links;
//  };
//
//  void Pack(const std::vector<net::LinkSequence>& paths) {
//    if (paths.empty()) {
//      return;
//    }
//
//    std::vector<bool> already_packed(paths.size(), false);
//
//    // Everything is packed in offsets from the shortest (first) path.
//    already_packed[0] = true;
//
//    while (packed_paths_.size() != paths.size() - 1) {
//      std::map<size_t, std::unique_ptr<PackedPath>> newly_packed =
//          MinDelta(paths, already_packed);
//      for (auto& index_and_packed_path : newly_packed) {
//        size_t index = index_and_packed_path.first;
//        already_packed[index] = true;
//
//        packed_paths_.emplace_back(std::move(index_and_packed_path.second));
//        LOG(ERROR) << "Packed " << index;
//      }
//
//      LOG(ERROR) << packed_paths_.size() << " vs " << paths.size();
//    }
//
//    for (const auto& packed_path : packed_paths_) {
//      LOG(ERROR) << "P " << packed_path->path_from << " -> "
//                 << packed_path->path << " links "
//                 << packed_path->links.Count();
//    }
//  }
//
//  // Given a path and a number of other paths returns the paths that differ
//  the
//  // least from the given one.
//  std::map<size_t, std::unique_ptr<PackedPath>> MinDelta(
//      const std::vector<net::LinkSequence>& all_paths,
//      const std::vector<bool>& already_packed) {
//    size_t min_delta = std::numeric_limits<size_t>::max();
//    std::map<size_t, std::unique_ptr<PackedPath>> min_delta_paths;
//
//    for (size_t index_from = 0; index_from < all_paths.size(); ++index_from) {
//      if (!already_packed[index_from]) {
//        continue;
//      }
//
//      const net::LinkSequence& path = all_paths[index_from];
//      net::GraphLinkSet links_one(path.links());
//
//      for (size_t index_to = 0; index_to < all_paths.size(); ++index_to) {
//        if (already_packed[index_to]) {
//          continue;
//        }
//
//        const net::LinkSequence& other_path = all_paths[index_to];
//        net::GraphLinkSet links_two(other_path.links());
//        net::GraphLinkSet new_links = links_one.Difference(links_two);
//        size_t delta = new_links.Count();
//
//        if (delta < min_delta) {
//          min_delta_paths.clear();
//          min_delta = delta;
//        }
//
//        if (delta != min_delta) {
//          continue;
//        }
//
//        std::unique_ptr<PackedPath>& current = min_delta_paths[index_to];
//        if (current && current->links.Count() < delta) {
//          continue;
//        }
//
//        current = make_unique<PackedPath>(index_to, index_from, new_links);
//      }
//    }
//
//    return min_delta_paths;
//  }
//
//  std::vector<std::unique_ptr<PackedPath>> packed_paths_;
//};

int main(int argc, char** argv) {
  Unused(argc);
  Unused(argv);

  //  net::PBNet net =
  //  net::GenerateHE(net::Bandwidth::FromBitsPerSecond(1000000),
  //                                   net::Delay::zero(), 2.0);
  net::PBNet net = net::GenerateNTT(net::Delay::zero(), 2.0);
  net::GraphStorage path_storage(net);
  LOG(ERROR) << "Graph with " << path_storage.NodeCount() << " nodes and "
             << path_storage.LinkCount() << " links";

  net::GraphNodeIndex london_node = path_storage.NodeFromStringOrDie("london");
  net::GraphNodeIndex tokyo_node = path_storage.NodeFromStringOrDie("tokyo");
  net::DirectedGraph graph(&path_storage);

  std::vector<net::LinkSequence> all_paths;
  TimeMs("DFS for all paths", [&graph, &london_node, &tokyo_node, &all_paths] {
    net::ConstraintSet constraints;
    //           size_t all_nodes_count =
    //           path_storage.AllNodes().Count();

    //           std::mt19937 rnd(1);
    //           net::GraphNodeSet to_visit =
    //               RandomSample(path_storage, all_nodes_count / 20,
    //               &rnd);
    //           for (net::GraphNodeIndex node_to_visit : to_visit) {
    //             LOG(ERROR) << "TV " <<
    //             path_storage.GetNode(node_to_visit)->id();
    //           }
    //           to_visit.Remove(london_node);
    //           to_visit.Remove(tokyo_node);
    //           constraints.AddToVisitSet(to_visit);

    net::SubGraph sub_graph(&graph, &constraints);
    sub_graph.Paths(london_node, tokyo_node,
                    [&all_paths](const net::LinkSequence& links) {
                      all_paths.emplace_back(links);
                    });
  });
  LOG(ERROR) << "Got " << all_paths.size() << " paths";
  std::sort(all_paths.begin(), all_paths.end());

  std::vector<net::LinkSequence> paths;
  TimeMs("KSP", [&graph, &london_node, &tokyo_node, &paths] {
    net::ConstraintSet constraints;
    //    size_t all_nodes_count = path_storage.AllNodes().Count();
    //
    //    std::mt19937 rnd(1);
    //    net::GraphNodeSet to_visit =
    //        RandomSample(path_storage, all_nodes_count / 20, &rnd);
    //    for (net::GraphNodeIndex node_to_visit : to_visit) {
    //      LOG(ERROR) << "TV " << path_storage.GetNode(node_to_visit)->id();
    //    }
    //    to_visit.Remove(london_node);
    //    to_visit.Remove(tokyo_node);
    //    constraints.AddToVisitSet(to_visit);

    net::SubGraph sub_graph(&graph, &constraints);
    net::KShortestPathsGenerator ksp(london_node, tokyo_node, &sub_graph);

    for (size_t i = 0; i < 100; ++i) {
      paths.emplace_back(ksp.KthShortestPath(i));
    }

    LOG(ERROR) << ksp.GetStats().ToString();
  });
  CHECK(Unique(all_paths));
  CHECK(Unique(paths));

  PrintPathStats(london_node, tokyo_node, paths, &graph);
  //  PackedPathSequence seq(paths);

  size_t simple_paths = CountSimplePaths(paths, &path_storage);
  double delta = Compare(all_paths, paths, simple_paths, &path_storage);
  LOG(ERROR) << "KSP " << paths.size() << " simple " << simple_paths
             << " simple paths delta " << delta;
}
