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

std::vector<net::GraphNodeSet> RandomSample(const net::GraphStorage& graph,
                                            size_t count, std::mt19937* rnd) {
  CHECK(count > 0);
  std::vector<net::GraphNodeIndex> all_nodes;
  for (net::GraphNodeIndex node : graph.AllNodes()) {
    all_nodes.emplace_back(node);
  }

  std::shuffle(all_nodes.begin(), all_nodes.end(), *rnd);

  std::vector<net::GraphNodeSet> out;
  out.emplace_back();

  for (net::GraphNodeIndex node_index : all_nodes) {
    net::GraphNodeSet& last_set = out.back();
    last_set.Insert(node_index);

    if (last_set.Count() == count) {
      out.emplace_back();
    }
  }

  return out;
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

void SinglePassSingleConstraint(const net::GraphStorage& storage,
                                net::GraphNodeIndex src_node,
                                net::GraphNodeIndex dst_node,
                                size_t to_visit_size, size_t count,
                                bool single) {
  net::DirectedGraph graph(&storage);
  net::ConstraintSet constraints;

  if (to_visit_size > 0) {
    std::mt19937 rnd(1);
    std::vector<net::GraphNodeSet> to_visit_sets =
        RandomSample(storage, to_visit_size, &rnd);

    if (single) {
      net::GraphNodeSet to_visit = to_visit_sets.front();
      to_visit.Remove(src_node);
      to_visit.Remove(dst_node);
      constraints.AddToVisitSet(to_visit);
    } else {
      for (const auto& set : to_visit_sets) {
        constraints.AddToVisitSet(set);
      }
    }
  }

  LOG(ERROR) << constraints.ToString(&storage);

  std::vector<net::LinkSequence> paths;
  std::vector<net::LinkSequence> all_paths;

  net::SubGraph sub_graph(&graph, &constraints);
  sub_graph.Paths(src_node, dst_node,
                  [&all_paths, &storage](const net::LinkSequence& links) {
                    all_paths.emplace_back(links);
                  },
                  {});
  std::sort(all_paths.begin(), all_paths.end());

  std::string tag = Substitute("KSP $0 $1", to_visit_size, count);
  TimeMs(tag, [src_node, dst_node, &sub_graph, &paths, count] {
    net::KShortestPathsGenerator ksp(src_node, dst_node, &sub_graph);
    for (size_t i = 0; i < count; ++i) {
      net::LinkSequence p = ksp.KthShortestPath(i);
      if (!p.empty()) {
        paths.emplace_back(p);
      }
    }
  });

  size_t simple_paths = CountSimplePaths(paths, &storage);
  LOG(ERROR) << "KSP " << paths.size() << " simple " << simple_paths
             << " all paths " << all_paths.size();
  double delta = Compare(all_paths, paths, simple_paths, &storage);
  LOG(ERROR) << "delta " << delta;
}

int main(int argc, char** argv) {
  Unused(argc);
  Unused(argv);

  net::PBNet net = net::GenerateNTT(net::Delay::zero(), 2.0);
  net::GraphStorage path_storage(net);
  LOG(ERROR) << "Graph with " << path_storage.NodeCount() << " nodes and "
             << path_storage.LinkCount() << " links";

  net::GraphNodeIndex london_node = path_storage.NodeFromStringOrDie("london");
  net::GraphNodeIndex tokyo_node = path_storage.NodeFromStringOrDie("tokyo");
  size_t all_nodes_count = path_storage.AllNodes().Count();
  size_t limit = all_nodes_count;

  for (size_t i = 0; i < limit; ++i) {
    SinglePassSingleConstraint(path_storage, london_node, tokyo_node, i,
                               1000000, true);
  }
}
