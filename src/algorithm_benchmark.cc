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

int main(int argc, char** argv) {
  Unused(argc);
  Unused(argv);

  net::PBNet net = net::GenerateHE(net::Bandwidth::FromBitsPerSecond(1000000),
                                   net::Delay::zero(), 2.0);
  net::GraphStorage path_storage(net);
  LOG(ERROR) << "Graph with " << path_storage.NodeCount() << " nodes and "
             << path_storage.LinkCount() << " links";

  net::GraphNodeIndex london_node = path_storage.NodeFromStringOrDie("London");
  net::GraphNodeIndex tokyo_node = path_storage.NodeFromStringOrDie("Tokyo");

  TimeMs("1000 calls to DirectedGraph",
         [&path_storage, &london_node, &tokyo_node] {
           for (size_t i = 0; i < 1000; ++i) {
             net::DirectedGraph graph(&path_storage);
             graph.ShortestPath(london_node, tokyo_node);
           }
         });

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
  TimeMs("10000 calls to shortest path", [&path_storage, &london_node,
                                          &tokyo_node, &paths] {
    net::DirectedGraph graph(&path_storage);
    net::ConstraintSet constraints;
    net::SubGraph sub_graph(&graph, &constraints);
    net::KShortestPathsGenerator ksp(london_node, tokyo_node, &sub_graph);

    for (size_t i = 0; i < 10000; ++i) {
      paths.emplace_back(ksp.KthShortestPath(i));
    }
  });

  for (size_t i = 0; i < 10000; ++i) {
    CHECK(all_paths[i] == paths[i]);
  }

  LOG(ERROR) << "KSP " << paths.size();
}
