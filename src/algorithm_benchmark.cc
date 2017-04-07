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

  std::vector<net::LinkSequence> all_paths;
  TimeMs("DFS for all paths",
         [&path_storage, &london_node, &tokyo_node, &all_paths] {
           net::DirectedGraph graph(&path_storage);
           net::ConstraintSet constraints;
           net::SubGraph sub_graph(&graph, &constraints);

           sub_graph.Paths(london_node, tokyo_node,
                           [&all_paths](const net::LinkSequence& links) {
                             all_paths.emplace_back(links);
                           });
         });
  LOG(ERROR) << "Got " << all_paths.size() << " paths";
  std::sort(all_paths.begin(), all_paths.end());

  std::vector<net::LinkSequence> paths;
  TimeMs("1000 calls to shortest path", [&path_storage, &london_node,
                                         &tokyo_node, &paths] {
    net::DirectedGraph graph(&path_storage);

    net::ConstraintSet constraints;
    size_t all_nodes_count = path_storage.AllNodes().Count();

    std::mt19937 rnd(1);
    net::GraphNodeSet to_visit =
        RandomSample(path_storage, all_nodes_count / 2, &rnd);
    constraints.AddToVisitSet(&to_visit);

    net::SubGraph sub_graph(&graph, &constraints);
    net::KShortestPathsGenerator ksp(london_node, tokyo_node, &sub_graph);

    for (size_t i = 0; i < 50000; ++i) {
      paths.emplace_back(ksp.KthShortestPath(i));
    }

    LOG(ERROR) << ksp.GetStats().ToString();
  });

  for (size_t i = 0; i < 1000; ++i) {
    CHECK(all_paths[i] == paths[i]);
  }

  LOG(ERROR) << "KSP " << paths.size();
}
