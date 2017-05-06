# ncode-net
Network-related features of ncode

This is a collection of graph algorithms and common grph functionality. It is
intended to be used in small physical network-like graphs (hundreds to thousands
of nodes). It includes:

- An efficient graph representation that allows for O(1) map/set operations on the graph
- Network topology generators, and a couple of real-life ISP networks
- Basic depth-limited DFS
- Single source shortest path (Dijkstra's algorithm)
- All pairs shortest paths (Floyd–Warshall's algorithm)
- K simple shortest paths (Yen's algorithm)
- The ability to get the K shortest paths based on an imprecise path definition

The library works with graphs that are assumed to be small with less than 65,535
(2^16 - 1) edges and links. While graphs are also assumed to be static, all
algorithms have the ability to work on subsets of the graph if needed. Graphs
are created using a `GraphBuilder`, and are maintained in `GraphStorage`. To
create a new graph with a single unidirectional edge you can do this:

```c++
#include "net_common.h"
using namespace nc::net;

GraphBuilder builder;
builder.AddLink({"A", "B", Bandwidth::FromGBitsPerSecond(10),
                std::chrono::microseconds(500)});

GraphStorage storage(builder);
```

Note that in addition to a source and a destination each edge has a bandwidth
and a delay value. This is because the library is meant to work with computer
network-like graphs. Source and destination ports can also be specified if
needed (by default the builder will assign them automatically).

A number of synthetic networks (and a couple of real-life ones) can be generated
using the functions in `net_gen.h`. For example to generate a network that
resembles that of the POP-level topology of NTT (a large ISP) you can do:

```c++
#include "net_common.h"
#include "net_gen.h"
using namespace nc::net;

GraphBuilder builder = GenerateNTT();
GraphStorage storage(builder);
```

Since there is a small number of graph elements, each is represented by a
sequential index, so that all map/set operations on the graph can be completed
in constant time. For example a set of links is just a vector with true/false
values for each one of the possible links in the graph. The `GraphStorage`
object is the one that relates between in-memory link objects (of class
`GraphLink`) and their indices (of type `GraphLinkIndex`). For example to create
a new set that will contain the london->amsterdam link from the graph above you
can do:

```c++
#include "net_common.h"
#include "net_gen.h"
using namespace nc::net;

GraphBuilder builder = GenerateNTT();
GraphStorage storage(builder);

GraphLinkSet link_set;
GraphLinkIndex link_index = storage.LinkOrDie("london", "amsterdam");
link_set.Insert(link_index);
```

The `LinkOrDie` function returns the index of the first (if there are multiple)
links between a source and a destination. It is also possible to use the indices
to index into a special `GraphLinkMap` map-like class like so:

```c++
GraphLinkMap<std::string> link_to_str;
GraphLinkIndex link_index = storage.LinkOrDie("london", "amsterdam");
link_to_str[link_index] = "Some label";
```

Under the hood the map is not really a map in the c++ sense (a tree or a hash
table), but simply a vector of strings.

## Paths

The `Walk` object is a wrapper around a vector of link indices that represents
graph walk. To get all paths between a source and a destination with a DFS you
can do:

```c++
ConstraintSet constraints;
SubGraph sub_graph(&storage, &constraints);

GraphNodeIndex src = storage.NodeFromStringOrDie("london");
GraphNodeIndex dst = storage.NodeFromStringOrDie("amsterdam");

std::vector<std::unique_ptr<net::Walk>> all_paths;
sub_graph.Paths(src, dst, [&all_paths](std::unique_ptr<net::Walk> walk) {
    all_paths.emplace_back(std::move(walk));
  }, {});
```

The `SubGraph` object combines a graph with a set of constraints (more on that
later). In this case there are no constraints, so this code will generate all
simple paths between the two nodes (about 3 million of them).
