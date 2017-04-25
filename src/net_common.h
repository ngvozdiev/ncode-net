#ifndef NCODE_NET_COMMON_H
#define NCODE_NET_COMMON_H

#include <google/protobuf/repeated_field.h>
#include <ncode/ncode_common/common.h>
#include <ncode/ncode_common/logging.h>
#include <ncode/ncode_common/perfect_hash.h>
#include <stddef.h>
#include <cassert>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <chrono>

#include "net.pb.h"

namespace nc {
namespace net {

// An IP address is just an integer -- currently v6 addresses are not supported.
struct IPAddressTag {};
using IPAddress = TypesafeUintWrapper<IPAddressTag, uint32_t, '*'>;

// The access-layer port is a 16bit integer.
struct AccessLayerPortTag {};
using AccessLayerPort = TypesafeUintWrapper<AccessLayerPortTag, uint16_t, '*'>;

// The IP protocol is an 8bit integer.
struct IPProtoTag {};
using IPProto = TypesafeUintWrapper<IPProtoTag, uint8_t, '*'>;

// The number of a network port on a device.
struct DevicePortNumberTag {};
using DevicePortNumber =
    TypesafeUintWrapper<DevicePortNumberTag, uint32_t, '*'>;

// Bandwidth.
struct BandwidthTag {};
class Bandwidth : public TypesafeUintWrapper<BandwidthTag, uint64_t> {
 public:
  static constexpr Bandwidth FromBitsPerSecond(uint64_t bps) {
    return Bandwidth(bps);
  }
  static constexpr Bandwidth FromMBitsPerSecond(double Mbps) {
    return Bandwidth(Mbps * 1000.0 * 1000.0);
  }
  static constexpr Bandwidth FromGBitsPerSecond(double Gbps) {
    return Bandwidth(Gbps * 1000.0 * 1000.0 * 1000.0);
  }
  static constexpr Bandwidth Zero() { return Bandwidth::FromBitsPerSecond(0); }
  static constexpr Bandwidth Max() {
    return Bandwidth::FromBitsPerSecond(std::numeric_limits<uint64_t>::max());
  }
  uint64_t bps() const { return m_val_; }
  double Mbps() const { return m_val_ / 1000.0 / 1000.0; }
  double Gbps() const { return m_val_ / 1000.0 / 1000.0 / 1000.0; }

  constexpr Bandwidth() : TypesafeUintWrapper<BandwidthTag, uint64_t>(0ul) {}

 private:
  constexpr Bandwidth(uint64_t value_bps)
      : TypesafeUintWrapper<BandwidthTag, uint64_t>(value_bps) {}
};

// Delay in microseconds.
using Delay = std::chrono::microseconds;

// Define some constants.
static constexpr IPProto kProtoTCP = IPProto(6);
static constexpr IPProto kProtoUDP = IPProto(17);
static constexpr IPProto kProtoICMP = IPProto(1);

// Adds a single edge to the graph. Useful for testing. Port numbers will be
// auto-assigned.
void AddEdgeToGraph(const std::string& src, const std::string& dst, Delay delay,
                    Bandwidth bandwidth, PBNet* graph);

// Like AddEdgeToGraph, but also adds an edge for dst->src.
void AddBiEdgeToGraph(const std::string& src, const std::string& dst,
                      Delay delay, Bandwidth bandwidth, PBNet* graph);

// A version of AddEdgeToGraph that takes a list of src, dst strings. All edges
// will have the same delay and bw, port numbers will be auto-assigned.
void AddEdgesToGraph(
    const std::vector<std::pair<std::string, std::string>>& edges, Delay delay,
    Bandwidth bw, PBNet* graph);

// Like AddEdgesToGraph, but for each edge also adds an opposite edge.
void AddBiEdgesToGraph(
    const std::vector<std::pair<std::string, std::string>>& edges, Delay delay,
    Bandwidth bw, PBNet* graph);

// Finds the edge between a source and a destination.
PBGraphLink* FindEdgeOrDie(const std::string& src, const std::string& dst,
                           PBNet* graph);

// Returns a set with the nodes that are in the same region as 'node'. If
// 'node' is not in any cluster will die. If it is alone in a cluster will
// return an empty set.
std::set<std::string> NodesInSameRegionOrDie(const PBNet& graph,
                                             const std::string& node);

// Returns a set with all nodes that are not in the same region as 'node'. If
// 'node' is not in any cluster will die.
std::set<std::string> NodesInOtherRegionsOrDie(const PBNet& graph,
                                               const std::string& node);

// Returns true if a link is between two nodes of the same cluster.
bool IsIntraClusterLink(const PBNet& graph, const PBGraphLink& link);

// Returns true if there is an edge with an endpoint equal to 'node' in the
// graph.
bool IsNodeInGraph(const PBNet& graph, const std::string& node);

// Returns true if there exists at least one node that is not reachable from
// every other node.
bool IsPartitioned(const PBNet& graph);

// Forward reference for the class that produces GraphLinkIndices and
// GraphNodeIndices.
class GraphStorage;

// The node is just an id.
class GraphNode {
 public:
  const std::string& id() const { return id_; }

 private:
  GraphNode(const std::string& id) : id_(id) {}

  std::string id_;

  friend class GraphStorage;
  DISALLOW_COPY_AND_ASSIGN(GraphNode);
};

using GraphNodeIndex = Index<GraphNode, uint16_t>;
using GraphNodeSet = PerfectHashSet<uint16_t, GraphNode>;

template <typename V>
using GraphNodeMap = PerfectHashMap<uint16_t, GraphNode, V>;

// A wrapper for PBGraphLink.
class GraphLink {
 public:
  GraphNodeIndex src() const { return src_; }

  const GraphNode* src_node() const { return src_node_; }

  GraphNodeIndex dst() const { return dst_; }

  const GraphNode* dst_node() const { return dst_node_; }

  DevicePortNumber src_port() const { return src_port_; }

  DevicePortNumber dst_port() const { return dst_port_; }

  Delay delay() const { return delay_; }

  Bandwidth bandwidth() const { return bandwidth_; }

  // Returns a string in the form A:sport->B:dport
  std::string ToString() const;

  // Returns a string in the form A->B
  std::string ToStringNoPorts() const;

  PBGraphLink ToProtobuf() const;

 private:
  GraphLink(const net::PBGraphLink& link_pb, GraphNodeIndex src,
            GraphNodeIndex dst, const GraphNode* src_node,
            const GraphNode* dst_node);

  GraphLink(GraphNodeIndex src, GraphNodeIndex dst, DevicePortNumber src_port,
            DevicePortNumber dst_port, Bandwidth bw, Delay delay,
            const GraphNode* src_node, const GraphNode* dst_node);

  GraphNodeIndex src_;
  GraphNodeIndex dst_;
  DevicePortNumber src_port_;
  DevicePortNumber dst_port_;
  Bandwidth bandwidth_;
  Delay delay_;

  // For convenience we also store pointers to the edpoints.
  const GraphNode* src_node_;
  const GraphNode* dst_node_;

  friend class GraphStorage;
  DISALLOW_COPY_AND_ASSIGN(GraphLink);
};

// Will assign indices to graph links and the indices should be used instead of
// pointers to GraphLink objects. The GraphStorage class will be able to relate
// from indices back to GraphLink instances. This has two advantages -- the ids
// can be shorter than the 8 bytes required to hold a pointer, which results in
// significant memory savings if we store a lot of paths, and the indices can be
// allocated sequentially allowing for O(1) set/map operations with links.
using GraphLinkIndex = Index<GraphLink, uint16_t>;
using GraphLinkSet = PerfectHashSet<uint16_t, GraphLink>;

// Prints the set of links.
std::string GraphLinkSetToString(const GraphLinkSet& links,
                                 const GraphStorage* graph_storage);

template <typename V>
using GraphLinkMap = PerfectHashMap<uint16_t, GraphLink, V>;

// Just a bunch of links.
using Links = std::vector<GraphLinkIndex>;

// Sums up the delay along a series of links.
Delay TotalDelayOfLinks(const Links& links, const GraphStorage* graph_storage);

// Returns true if the given array of links has duplicates.
bool HasDuplicateLinks(const Links& links);

// Returns true if the given array of links has nodes.
bool HasDuplicateNodes(const Links& links, const GraphStorage* graph_storage);

// Returns the detour of 'path_two' from 'path_one'. Paths should start/end at
// the same node. The detour is returned as a pair of indices into path_two that
// indicate the start/end of the detour. If the two paths are the same {-1, -1}
// is returned.
std::pair<size_t, size_t> LinksDetour(const Links& path_one,
                                      const Links& path_two);

// A sequence of links along with a delay. Similar to GraphPath (below), but
// without a tag.
class LinkSequence {
 public:
  LinkSequence();
  LinkSequence(const Links& links, Delay delay,
               bool check_for_duplicates = true);
  LinkSequence(const Links& links, const GraphStorage* storage,
               bool check_for_duplicates = true);

  // Returns true if any of the links in this sequence is equal to link.
  bool Contains(GraphLinkIndex link) const;

  // Returns true if any of the links in this sequence are in the set.
  bool ContainsAny(GraphLinkSet links) const;

  // The delay of all links in this sequence.
  Delay delay() const;

  // Number of links in the sequence.
  size_t size() const { return links_.size(); }

  // Whether or not there are any links in the sequence.
  bool empty() const { return links_.empty(); }

  // The list of links.
  const Links& links() const { return links_; }

  // String representation in the form [A:p1->B:p2, B:p3->C:p3]
  std::string ToString(const GraphStorage* storage) const;

  // Shorter string representation in the form [A->B->C]
  std::string ToStringNoPorts(const GraphStorage* storage) const;

  // Only prints out the ids of the edges on the path, not the string
  // representation of their nodes.
  std::string ToStringIdsOnly() const;

  // Id of the first node along the path.
  GraphNodeIndex FirstHop(const GraphStorage* storage) const;

  // Id of the last node along the path.
  GraphNodeIndex LastHop(const GraphStorage* storage) const;

  // Rough estimate of the number of bytes of memory this LinkSequence uses.
  size_t InMemBytesEstimate() const;

  // Returns true if this LinkSequence has duplicate hops. The link sequence is
  // assumed to be a path.
  bool HasDuplicateNodes(const GraphStorage* graph_storage) const;

  friend bool operator<(const LinkSequence& a, const LinkSequence& b);
  friend bool operator==(const LinkSequence& a, const LinkSequence& b);
  friend bool operator!=(const LinkSequence& a, const LinkSequence& b);

 private:
  // The links in this sequence.
  Links links_;

  // The total delay of all links in the sequence.
  Delay delay_;
};

// GraphPaths are heavier versions of LinkSequence that are assigned ids and are
// managed by a PathStorage instance.
class GraphPath {
 public:
  // String representation in the form [A:p1->B:p2, B:p3->C:p3]
  std::string ToString() const;

  // String representation in the form A -> B -> C
  std::string ToStringNoPorts() const;

  // The underlying link sequence.
  const LinkSequence& link_sequence() const { return link_sequence_; }

  // Delay of the path.
  Delay delay() const { return link_sequence_.delay(); }

  // True if path is empty.
  bool empty() const { return link_sequence_.links().empty(); }

  // Number of links.
  uint32_t size() const { return link_sequence_.links().size(); }

  // The storage object that keeps track of all paths.
  GraphStorage* storage() const { return storage_; }

  // A tag that identifies the path. Two paths with the same tag will have the
  // same memory location.
  uint32_t tag() const { return tag_; }

  // Id of the first node along the path.
  GraphNodeIndex FirstHop() const;

  // Id of the last node along the path.
  GraphNodeIndex LastHop() const;

  // Constructs an initially empty path.
  GraphPath(GraphStorage* storage) : tag_(0), storage_(storage) {}

  // Populates this object.
  void Populate(LinkSequence link_sequence, uint32_t tag);

 private:
  // The sequence of links that form this path.
  LinkSequence link_sequence_;

  // A number uniquely identifying the path.
  uint32_t tag_;

  // The parent storage.
  GraphStorage* storage_;

  DISALLOW_COPY_AND_ASSIGN(GraphPath);
};

struct PathComparator {
  bool operator()(const GraphPath* p1, const GraphPath* p2) {
    return p1->delay() < p2->delay();
  }
};

// General statistics about a graph.
struct GraphStats {
  size_t unidirectional_links;
  size_t multiple_links;
};

// Stores and maintains a graph. Also Stores paths and assigns tags to them.
// Will return the same path object for the same sequence of links with the
// same cookie. Additionally tags paths with a simple integer tag if the same
// paths should be made to map to different path objects.
class GraphStorage {
 public:
  explicit GraphStorage(const PBNet& graph);

  // Returns a new GraphStorage, with some nodes from this GraphStorage
  // clustered. Creating clusters may lead to multiple links between two nodes,
  // even if the original graph is simple. This function will also assign names
  // to cluster nodes. For example if a cluster combines nodes N1 and N2 the
  // name of the cluster node will be C_N1_N2. Each node from this graph should
  // be contained in exactly one cluster.
  std::unique_ptr<GraphStorage> ClusterNodes(
      const std::vector<GraphNodeSet>& clusters,
      GraphLinkMap<GraphLinkIndex>* real_to_clustered_links,
      GraphNodeMap<GraphNodeIndex>* real_to_clustered_nodes) const;

  // Returns the nodes that are in the same region as 'node'. If 'node' is not
  // in a region will die.
  GraphNodeSet NodesInSameRegionOrDie(GraphNodeIndex node) const;

  // Returns the nodes that are in other regions as 'node'. If 'node' is not in
  // a region will die.
  GraphNodeSet NodesInOtherRegionsOrDie(GraphNodeIndex node) const;

  // At least the src and the dst need to be populated in the link_pb. Will die
  // if there is no link from src to dst.
  GraphLinkIndex LinkFromProtobufOrDie(const PBGraphLink& link_pb) const;

  // If there is no link between src and dst the ports need to also be populated
  // in order to create a new one.
  GraphLinkIndex LinkFromProtobuf(const PBGraphLink& link_pb);

  // Finds a link between src and dst. If multiple links exist will return only
  // one of them. Will die if no links exist.
  GraphLinkIndex LinkOrDie(const std::string& src,
                           const std::string& dst) const;

  // Same as NodeFromString, but dies if the node does not exist.
  GraphNodeIndex NodeFromStringOrDie(const std::string& id) const;

  // Attempts to find the unique inverse of a link. If the link has no inverse,
  // or has multiple inverses will die.
  GraphLinkIndex FindUniqueInverseOrDie(const GraphLink* link) const;

  // Relates from indices to link objects.
  const GraphLink* GetLink(GraphLinkIndex index) const;

  // Relates from indices to node objects.
  const GraphNode* GetNode(GraphNodeIndex index) const;

  // Number of links stored.
  size_t LinkCount() const { return link_store_.size(); }

  // Number of nodes stored.
  size_t NodeCount() const { return node_store_.size(); }

  // Returns the set of all nodes in the graph.
  GraphNodeSet AllNodes() const {
    return GraphNodeSet::FullSetFromStore(node_store_);
  }

  // Returns the set of all links in the graph.
  GraphLinkSet AllLinks() const {
    return GraphLinkSet::FullSetFromStore(link_store_);
  }

  GraphStats Stats() const;

  // Combines LinkFromProtobufOrDie with GetLink for convenience.
  const GraphLink* LinkPtrFromProtobufOrDie(const PBGraphLink& link_pb) const;

  // Combines LinkFromProtobuf with GetLink for convenience.
  const GraphLink* LinkPtrFromProtobuf(const PBGraphLink& link_pb);

  // Node ID to node index.
  const std::map<std::string, GraphNodeIndex>& NodeIdToIndex() const {
    return nodes_;
  }

  // Returns a graph path from a string of the form [A->B, B->C]. Port
  // numbers cannot be specified -- do not use if double edges are possible.
  const GraphPath* PathFromStringOrDie(const std::string& path_string,
                                       uint64_t cookie);

  // Like PathFromStringOrDie, but simply returns a link sequence instead of a
  // graph path.
  LinkSequence LinkSequenceFromStringOrDie(
      const std::string& path_string) const;

  // Retuns a graph path from a sequence of links.
  const GraphPath* PathFromLinksOrDie(const LinkSequence& links,
                                      uint64_t cookie);

  // From a protobuf repetated field.
  const GraphPath* PathFromProtobufOrDie(
      const google::protobuf::RepeatedPtrField<PBGraphLink>& links,
      uint64_t cookie);

  // From a vector with protobufs.
  const GraphPath* PathFromProtobufOrDie(const std::vector<PBGraphLink>& links,
                                         uint64_t cookie);

  // Returns the empty path. There is only one empty path instance per
  // PathStorage.
  const GraphPath* EmptyPath() const { return empty_path_.get(); }

  // Dumps all paths in this path storage to a string.
  std::string DumpPaths() const;

  // Finds a path given its tag.
  const GraphPath* FindPathByTagOrNull(uint32_t tag) const;

 private:
  using LinkStore =
      PerfectHashStore<std::unique_ptr<GraphLink>, uint16_t, GraphLink>;
  using NodeStore =
      PerfectHashStore<std::unique_ptr<GraphNode>, uint16_t, GraphNode>;

  GraphStorage() : tag_generator_(0) {
    empty_path_ = make_unique<GraphPath>(this);
  }

  std::string GetClusterName(const GraphNodeSet& nodes) const;

  bool LinkFromProtobuf(const PBGraphLink& link_pb,
                        GraphLinkIndex* index) const;

  // Returns the index of a node identified by a string.
  GraphNodeIndex NodeFromString(const std::string& id);

  // The index of a node's region.
  size_t IndexOfRegionOrDie(GraphNodeIndex node) const;

  // A map from src to dst to a list of links between that (src, dst) pair. The
  // list will only contain more than one element if double edges are used.
  std::map<std::string, std::map<std::string, Links>> links_;
  std::map<std::string, NodeStore::IndexType> nodes_;
  LinkStore link_store_;
  NodeStore node_store_;

  // Non-empty paths. Grouped by cookie and then by links in the path.
  std::map<uint64_t, std::map<Links, GraphPath>> cookie_to_paths_;

  // There is only one empty path instance.
  std::unique_ptr<GraphPath> empty_path_;

  // Path tags come from here.
  uint32_t tag_generator_;

  // Regions, each node should be contained in at most one.
  std::vector<GraphNodeSet> regions_;

  DISALLOW_COPY_AND_ASSIGN(GraphStorage);
};

// A convenience function equivalent to calling StringToPath followed by
// std::find to check if haystack contains the path needle.
bool IsInPaths(const std::string& needle,
               const std::vector<LinkSequence>& haystack,
               GraphStorage* storage);

bool IsInPaths(const std::string& needle,
               const std::vector<const GraphPath*>& haystack, uint64_t cookie,
               GraphStorage* storage);

// A five-tuple is a combination of ip src/dst, access layer src/dst ports and
// protocol type. It uniquely identifies an IP connection and can be used for
// matching.
class FiveTuple {
 public:
  static const FiveTuple kDefaultTuple;

  static constexpr std::size_t GetHash(IPAddress ip_src, IPAddress ip_dst,
                                       IPProto ip_proto,
                                       AccessLayerPort src_port,
                                       AccessLayerPort dst_port) {
    return 37 * (37 * (37 * (37 * (37 * 17 + ip_proto.Raw()) + ip_src.Raw()) +
                       ip_dst.Raw()) +
                 src_port.Raw()) +
           dst_port.Raw();
  }

  constexpr FiveTuple()
      : ip_src_(0),
        ip_dst_(0),
        ip_proto_(0),
        src_port_(0),
        dst_port_(0),
        hash_(0) {}

  constexpr FiveTuple(IPAddress ip_src, IPAddress ip_dst, IPProto ip_proto,
                      AccessLayerPort src_port, AccessLayerPort dst_port)
      : ip_src_(ip_src),
        ip_dst_(ip_dst),
        ip_proto_(ip_proto),
        src_port_(src_port),
        dst_port_(dst_port),
        hash_(GetHash(ip_src, ip_dst, ip_proto, src_port, dst_port)) {}

  // The access layer destination port.
  AccessLayerPort dst_port() const { return dst_port_; }

  // The IP destination address.
  IPAddress ip_dst() const { return ip_dst_; }

  // The IP protocol.
  IPProto ip_proto() const { return ip_proto_; }

  // The IP source address.
  IPAddress ip_src() const { return ip_src_; }

  // The access layer source port.
  AccessLayerPort src_port() const { return src_port_; }

  // This tuple's hash value.
  std::size_t hash() const { return hash_; }

  // Returns a FiveTuple that will match the other side of the connection that
  // this tuple matches. The new tuple will have src/dst address and ports
  // swapped.
  FiveTuple Reverse() const {
    return FiveTuple(ip_dst_, ip_src_, ip_proto_, dst_port_, src_port_);
  }

  // Same as the << operator, but returns a string.
  std::string ToString() const;

  // Pretty-printing comparion and equality.
  friend std::ostream& operator<<(std::ostream& output, const FiveTuple& op);
  friend bool operator==(const FiveTuple& a, const FiveTuple& b);
  friend bool operator!=(const FiveTuple& a, const FiveTuple& b);
  friend bool operator<(const FiveTuple& a, const FiveTuple& b);
  friend bool operator>(const FiveTuple& a, const FiveTuple& b);

 private:
  IPAddress ip_src_;
  IPAddress ip_dst_;
  IPProto ip_proto_;
  AccessLayerPort src_port_;
  AccessLayerPort dst_port_;

  // The hash value of the tuple is cached here.
  std::size_t hash_;
};

// Hashes a FiveTuple.
struct FiveTupleHasher {
  std::size_t operator()(const FiveTuple& k) const { return k.hash(); }
};

// Converts between strings and IPAddresses.
std::string IPToStringOrDie(IPAddress ip);
IPAddress StringToIPOrDie(const std::string& str);

// Applies a mask to a given IPAddress.
static constexpr uint8_t kMaxIPAddressMaskLen = 32;
IPAddress MaskAddress(IPAddress ip_address, uint8_t mask);

// An IPv4 range (combination of address and mask).
class IPRange {
 public:
  static constexpr const char* kDelimiter = "/";

  IPRange(IPAddress address, uint8_t mask_len);

  IPRange(const std::string& range_str);

  // The number of bits in the mask.
  uint8_t mask_len() const { return mask_len_; }

  // The base address.
  IPAddress base_address() const { return base_address_; }

  std::string ToString() const;

 private:
  void Init(IPAddress address, uint8_t mask_len);

  IPAddress base_address_;
  uint8_t mask_len_;
};

// Helper function for GetDisjointSets below.
template <typename T, typename V>
void RecursiveAddToDisjointSets(
    const std::map<V, std::vector<T>>& value_to_keys,
    const std::map<T, std::vector<V>>& key_to_values, const T& key,
    std::set<T>* out) {
  if (ContainsKey(*out, key)) {
    return;
  }
  out->insert(key);

  const std::vector<V>& values = FindOrDie(key_to_values, key);
  for (const V& value : values) {
    const std::vector<T>& keys_for_value = FindOrDie(value_to_keys, value);
    for (const T& key_for_value : keys_for_value) {
      if (key_for_value == key) {
        continue;
      }

      RecursiveAddToDisjointSets(value_to_keys, key_to_values, key_for_value,
                                 out);
    }
  }
}

// Given a map from a generic type to a list of values, this type will return
// lists of keys that do not share any single value.
template <typename T, typename V>
std::vector<std::set<T>> GetDisjointSets(
    const std::map<T, std::vector<V>>& key_to_values) {
  std::map<V, std::vector<T>> value_to_keys;
  for (const auto& key_and_values : key_to_values) {
    const T& key = key_and_values.first;
    const std::vector<V>& values = key_and_values.second;
    CHECK(!values.empty());

    for (const V& value : values) {
      value_to_keys[value].emplace_back(key);
    }
  }

  std::vector<std::set<T>> out;
  for (const auto& key_and_values : key_to_values) {
    const T& key = key_and_values.first;
    bool found = false;
    for (const auto& output_set : out) {
      if (ContainsKey(output_set, key)) {
        found = true;
        break;
      }
    }

    if (found) {
      continue;
    }

    std::set<T> new_set;
    RecursiveAddToDisjointSets(value_to_keys, key_to_values, key, &new_set);
    out.emplace_back(new_set);
  }

  return out;
}

}  // namespace nc
}  // namespace ncode

#endif
