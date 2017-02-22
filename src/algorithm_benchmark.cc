#include <chrono>

#include "net.pb.h"
#include "../common/common.h"
#include "../common/file.h"
#include "../common/logging.h"
#include "../common/substitute.h"
#include "algorithm.h"
#include "net_common.h"
#include "net_gen.h"

using namespace ncode;
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

int main(int argc, char** argv) {
  Unused(argc);
  Unused(argv);

  net::PBNet net = net::GenerateSprint(
      net::Bandwidth::FromBitsPerSecond(1000000), net::Delay::zero(), 2.0);
  net::GraphStorage path_storage(net);
  net::DirectedGraph graph(&path_storage);
  LOG(ERROR) << "Graph with " << path_storage.NodeCount() << " nodes and "
             << path_storage.LinkCount() << " links";

  net::DistanceClusteredGraph clustered_graph({}, milliseconds(2), &graph);
  net::GraphStats stats = clustered_graph.clustered_storage()->Stats();
  LOG(ERROR) << stats.multiple_links << " " << stats.unidirectional_links
             << clustered_graph.clustered_storage()->AllNodes().Count() << " "
             << clustered_graph.clustered_storage()->AllLinks().Count();

  TimeMs("All pair shortest path",
         [&graph] { net::AllPairShortestPath all_pair_sp({}, &graph); });

  net::GraphNodeIndex london_node =
      path_storage.NodeFromStringOrDie("London4045");
  net::GraphNodeIndex tokyo_node =
      path_storage.NodeFromStringOrDie("Tokyo4070");

  TimeMs("1000 calls to shortest path", [&graph, &london_node, &tokyo_node] {
    for (size_t i = 0; i < 1000; ++i) {
      net::ShortestPath sp({}, london_node, &graph);
      sp.GetPath(tokyo_node);
    }
  });

  //  std::vector<net::LinkSequence> paths;
  //  paths.reserve(10000000);
  //  TimeMs("DFS, all paths between a pair of endpoints", [&graph,
  //  &london_node,
  //                                                        &tokyo_node, &paths]
  //                                                        {
  //    net::DFS dfs({}, &graph);
  //    dfs.Paths(
  //        london_node, tokyo_node, duration_cast<net::Delay>(seconds(1)), 20,
  //        [&paths](const net::LinkSequence& path) { paths.emplace_back(path);
  //        });
  //  });
  //  std::sort(paths.begin(), paths.end());

  std::vector<net::LinkSequence> k_paths;
  k_paths.reserve(1000);
  TimeMs("1000 shortest paths", [&graph, &london_node, &tokyo_node, &k_paths] {
    net::KShortestPaths ksp({}, {}, london_node, tokyo_node, &graph);
    for (size_t i = 0; i < 1000; ++i) {
      k_paths.emplace_back(ksp.NextPath());
    }
  });

  //  for (size_t i = 0; i < 1000; ++i) {
  //    CHECK(k_paths[i] == paths[i]) << i << "th shortest path differs";
  //  }

  std::string out;
  size_t id = 0;
  for (net::GraphNodeIndex src : path_storage.AllNodes()) {
    for (net::GraphNodeIndex dst : path_storage.AllNodes()) {
      if (src == dst) {
        continue;
      }

      for (size_t i = 0; i < 10; ++i) {
        size_t count = 100 * i;
        std::string message = Substitute("$0 shortest paths", count);
        TimeToString(&out, id, count, [&graph, &src, &dst, count] {
          net::KShortestPaths ksp({}, {}, src, dst, &graph);
          for (size_t j = 0; j < count; ++j) {
            ksp.NextPath();
          }
        });
      }

      LOG(ERROR) << "I " << id;

      ++id;

      if (id == 100) {
        break;
      }
    }

    if (id == 100) {
      break;
    }
    LOG(ERROR) << "Done " << src;
  }

  File::WriteStringToFile(out, "benchmark_output");
}
