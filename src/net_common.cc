#include "net_common.h"

#include <arpa/inet.h>
#include <ncode/ncode_common/map_util.h>
#include <ncode/ncode_common/substitute.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <functional>
#include <iterator>
#include <limits>
#include <sstream>
#include <tuple>
#include <utility>

namespace nc {
namespace net {

void AddEdgeToGraph(const std::string& src, const std::string& dst, Delay delay,
                    Bandwidth bw, PBNet* graph) {
  using namespace std::chrono;

  CHECK(src != dst) << "Source same as destination: " << src;
  CHECK(!src.empty() && !dst.empty()) << "Source or destination ID missing.";
  PBGraphLink* edge = graph->add_links();
  edge->set_src(src);
  edge->set_dst(dst);
  edge->set_delay_sec(duration<double>(delay).count());
  edge->set_bandwidth_bps(bw.bps());

  uint32_t port_num = graph->links_size();
  edge->set_src_port(port_num);
  edge->set_dst_port(port_num);
}

void AddBiEdgeToGraph(const std::string& src, const std::string& dst,
                      Delay delay, Bandwidth bw, PBNet* graph) {
  AddEdgeToGraph(src, dst, delay, bw, graph);
  AddEdgeToGraph(dst, src, delay, bw, graph);
}

static void AddEdgesToGraphHelper(
    const std::vector<std::pair<std::string, std::string>>& edges, Delay delay,
    Bandwidth bw, bool bidirectional, PBNet* graph) {
  for (const auto& src_and_dst : edges) {
    const std::string& src = src_and_dst.first;
    const std::string& dst = src_and_dst.second;
    if (bidirectional) {
      AddBiEdgeToGraph(src, dst, delay, bw, graph);
    } else {
      AddEdgeToGraph(src, dst, delay, bw, graph);
    }
  }
}

void AddEdgesToGraph(
    const std::vector<std::pair<std::string, std::string>>& edges, Delay delay,
    Bandwidth bw, PBNet* graph) {
  AddEdgesToGraphHelper(edges, delay, bw, false, graph);
}

void AddBiEdgesToGraph(
    const std::vector<std::pair<std::string, std::string>>& edges, Delay delay,
    Bandwidth bw, PBNet* graph) {
  AddEdgesToGraphHelper(edges, delay, bw, true, graph);
}

std::chrono::microseconds TotalDelayOfLinks(const Links& links,
                                            const GraphStorage* graph_storage) {
  std::chrono::microseconds total(0);
  for (GraphLinkIndex link_index : links) {
    const GraphLink* link = graph_storage->GetLink(link_index);
    total += link->delay();
  }

  return total;
}

GraphStorage::GraphStorage(const PBNet& graph) : tag_generator_(0) {
  empty_path_ = make_unique<GraphPath>(this);
  for (const auto& link_pb : graph.links()) {
    LinkFromProtobuf(link_pb);
  }

  for (const PBNetRegion& region : graph.regions()) {
    GraphNodeSet nodes_in_region;
    for (const std::string& node_id : region.nodes()) {
      nodes_in_region.Insert(NodeFromStringOrDie(node_id));
    }

    regions_.emplace_back(nodes_in_region);
  }
}

GraphLinkIndex GraphStorage::FindUniqueInverseOrDie(
    const GraphLink* link) const {
  const std::string& src = GetNode(link->src())->id();
  const std::string& dst = GetNode(link->dst())->id();

  const auto& dst_to_links = FindOrDie(links_, dst);
  const Links& links = FindOrDie(dst_to_links, src);
  CHECK(links.size() == 1) << "Double edge";
  return links.front();
}

const GraphLink* GraphStorage::GetLink(GraphLinkIndex link_index) const {
  return link_store_.GetItemOrDie(link_index).get();
}

const GraphNode* GraphStorage::GetNode(GraphNodeIndex node_index) const {
  return node_store_.GetItemOrDie(node_index).get();
}

GraphNodeIndex GraphStorage::NodeFromString(const std::string& id) {
  auto it = nodes_.find(id);
  if (it != nodes_.end()) {
    return it->second;
  }

  auto node_ptr = std::unique_ptr<GraphNode>(new GraphNode(id));
  GraphNodeIndex index = node_store_.MoveItem(std::move(node_ptr));
  nodes_[id] = index;
  return index;
}

GraphNodeIndex GraphStorage::NodeFromStringOrDie(const std::string& id) const {
  return FindOrDie(nodes_, id);
}

std::string GraphStorage::GetClusterName(const GraphNodeSet& nodes) const {
  if (nodes.Count() == 1) {
    return GetNode(*nodes.begin())->id();
  }

  std::function<std::string(const GraphNodeIndex& node)> f = [this](
      GraphNodeIndex node) { return GetNode(node)->id(); };

  std::string out = "C";
  Join(nodes.begin(), nodes.end(), "_", f, &out);
  return out;
}

GraphStats GraphStorage::Stats() const {
  std::map<std::pair<GraphNodeIndex, GraphNodeIndex>, size_t> adjacency;
  for (GraphLinkIndex link : AllLinks()) {
    const GraphLink* link_ptr = GetLink(link);
    GraphNodeIndex src = link_ptr->src();
    GraphNodeIndex dst = link_ptr->dst();
    ++adjacency[{src, dst}];
    adjacency[{dst, src}];
  }

  size_t multiple_links = 0;
  size_t unidirectional_links = 0;
  for (const auto& src_and_dst_and_count : adjacency) {
    GraphNodeIndex src;
    GraphNodeIndex dst;
    std::tie(src, dst) = src_and_dst_and_count.first;

    if (src > dst) {
      size_t fw_count = src_and_dst_and_count.second;
      size_t rev_count = adjacency[{dst, src}];
      size_t diff =
          std::max(fw_count, rev_count) - std::min(fw_count, rev_count);
      unidirectional_links += diff;
      if (fw_count > 1) {
        ++multiple_links;
      }
      if (rev_count > 1) {
        ++rev_count;
      }
    }
  }

  return {unidirectional_links, multiple_links};
}

std::unique_ptr<GraphStorage> GraphStorage::ClusterNodes(
    const std::vector<GraphNodeSet>& clusters,
    GraphLinkMap<GraphLinkIndex>* real_to_clustered_links,
    GraphNodeMap<GraphNodeIndex>* real_to_clustered_nodes) const {
  GraphNodeMap<size_t> node_to_cluster;
  std::vector<std::string> cluster_names(clusters.size());

  auto new_storage = std::unique_ptr<GraphStorage>(new GraphStorage());
  for (size_t i = 0; i < clusters.size(); ++i) {
    const GraphNodeSet& cluster = clusters[i];
    std::string cluster_name = GetClusterName(cluster);
    cluster_names[i] = cluster_name;

    for (GraphNodeIndex node_in_cluster : cluster) {
      node_to_cluster[node_in_cluster] = i;
      real_to_clustered_nodes->Add(node_in_cluster,
                                   new_storage->NodeFromString(cluster_name));
    }
  }

  size_t port_num = 0;
  for (GraphLinkIndex link_index : AllLinks()) {
    const GraphLink* link = GetLink(link_index);
    GraphNodeIndex src_index = link->src();
    GraphNodeIndex dst_index = link->dst();

    size_t src_cluster = node_to_cluster[src_index];
    size_t dst_cluster = node_to_cluster[dst_index];

    // If both src and dst are in the same cluster we can skip adding the link.
    if (src_cluster == dst_cluster) {
      continue;
    }

    const std::string& src_cluster_name = cluster_names[src_cluster];
    const std::string& dst_cluster_name = cluster_names[dst_cluster];

    GraphNodeIndex src_cluster_index =
        new_storage->NodeFromString(src_cluster_name);
    GraphNodeIndex dst_cluster_index =
        new_storage->NodeFromString(dst_cluster_name);

    net::DevicePortNumber port(++port_num);
    auto link_ptr = std::unique_ptr<GraphLink>(new GraphLink(
        src_cluster_index, dst_cluster_index, port, port, link->bandwidth(),
        link->delay(), new_storage->GetNode(src_cluster_index),
        new_storage->GetNode(dst_cluster_index)));
    GraphLinkIndex clustered_link_index =
        new_storage->link_store_.MoveItem(std::move(link_ptr));
    real_to_clustered_links->Add(link_index, clustered_link_index);
    new_storage->links_[src_cluster_name][dst_cluster_name].emplace_back(
        clustered_link_index);
  }

  return new_storage;
}

bool GraphStorage::LinkFromProtobuf(const PBGraphLink& link_pb,
                                    GraphLinkIndex* index) const {
  // First try to find the link by the src and dst.
  CHECK(!link_pb.src().empty() && !link_pb.dst().empty())
      << "Link source or destination missing";
  CHECK(link_pb.src() != link_pb.dst()) << "Link source same as destination: "
                                        << link_pb.src();
  auto it_one = links_.find(link_pb.src());
  if (it_one != links_.end()) {
    auto it_two = it_one->second.find(link_pb.dst());
    if (it_two != it_one->second.end()) {
      if (link_pb.src_port() == 0 && link_pb.dst_port() == 0) {
        *index = it_two->second.front();
        return true;
      }

      // There are one or many links with the same src and dst addresses.
      const GraphLink* other_with_same_src_port = nullptr;
      const GraphLink* other_with_same_dst_port = nullptr;
      GraphLinkIndex other_with_same_src_port_index;
      for (GraphLinkIndex link_index : it_two->second) {
        const GraphLink* link_ptr = GetLink(link_index);

        if (link_pb.src_port() != 0 &&
            link_ptr->src_port().Raw() == link_pb.src_port()) {
          CHECK(other_with_same_src_port == nullptr);
          other_with_same_src_port = link_ptr;
          other_with_same_src_port_index = link_index;
        }

        if (link_pb.dst_port() != 0 &&
            link_ptr->dst_port().Raw() == link_pb.dst_port()) {
          CHECK(other_with_same_dst_port == nullptr);
          other_with_same_dst_port = link_ptr;
        }
      }

      CHECK(other_with_same_src_port == other_with_same_dst_port)
          << other_with_same_src_port << " != " << other_with_same_dst_port;
      if (other_with_same_src_port != nullptr) {
        *index = other_with_same_src_port_index;
        return true;
      }
    }
  }

  return false;
}

GraphLinkIndex GraphStorage::LinkFromProtobufOrDie(
    const PBGraphLink& link_pb) const {
  GraphLinkIndex out;
  CHECK(LinkFromProtobuf(link_pb, &out));
  return out;
}

GraphLinkIndex GraphStorage::LinkOrDie(const std::string& src,
                                       const std::string& dst) const {
  CHECK(!src.empty() && !dst.empty()) << "Link source or destination missing";
  CHECK(src != dst) << "Link source same as destination: " << src;
  auto it_one = links_.find(src);
  if (it_one != links_.end()) {
    auto it_two = it_one->second.find(dst);
    if (it_two != it_one->second.end()) {
      return it_two->second.front();
    }
  }

  LOG(FATAL) << "No link between " << src << " and " << dst;
  return GraphLinkIndex(0);
}

GraphLinkIndex GraphStorage::LinkFromProtobuf(const PBGraphLink& link_pb) {
  GraphLinkIndex out;
  if (LinkFromProtobuf(link_pb, &out)) {
    return out;
  }

  // Unable to find a link, need to create new one. At this point the protobuf
  // needs to have the ports set.
  CHECK(link_pb.src_port() != 0 && link_pb.dst_port() != 0)
      << "Source or destination port missing for new link from "
      << link_pb.src() << " to " << link_pb.dst();

  auto src_index = NodeFromString(link_pb.src());
  auto dst_index = NodeFromString(link_pb.dst());
  auto link_ptr = std::unique_ptr<GraphLink>(new GraphLink(
      link_pb, src_index, dst_index, GetNode(src_index), GetNode(dst_index)));
  GraphLinkIndex index = link_store_.MoveItem(std::move(link_ptr));
  links_[link_pb.src()][link_pb.dst()].emplace_back(index);
  return index;
}

const GraphLink* GraphStorage::LinkPtrFromProtobufOrDie(
    const PBGraphLink& link_pb) const {
  GraphLinkIndex link_index = LinkFromProtobufOrDie(link_pb);
  return GetLink(link_index);
}

const GraphLink* GraphStorage::LinkPtrFromProtobuf(const PBGraphLink& link_pb) {
  GraphLinkIndex link_index = LinkFromProtobuf(link_pb);
  return GetLink(link_index);
}

bool HasDuplicateLinks(const Links& links) {
  for (size_t i = 0; i < links.size(); ++i) {
    for (size_t j = i + 1; j < links.size(); ++j) {
      if (links[i] == links[j]) {
        return true;
      }
    }
  }

  return false;
}

std::pair<size_t, size_t> LinksDetour(const Links& path_one,
                                      const Links& path_two) {
  CHECK(!path_one.empty());
  CHECK(!path_two.empty());
  size_t detour_start = std::numeric_limits<size_t>::max();
  for (size_t i = 0; i < path_one.size(); ++i) {
    net::GraphLinkIndex link_one = path_one[i];
    net::GraphLinkIndex link_two = path_two[i];
    if (link_one != link_two) {
      detour_start = i;
      break;
    }
  }

  if (detour_start == std::numeric_limits<size_t>::max()) {
    return {std::numeric_limits<size_t>::max(),
            std::numeric_limits<size_t>::max()};
  }

  size_t detour_end = std::numeric_limits<size_t>::max();
  size_t path_one_size = path_one.size();
  size_t path_two_size = path_two.size();
  for (size_t i = 1; i < path_one.size(); ++i) {
    net::GraphLinkIndex link_one = path_one[path_one_size - i];
    net::GraphLinkIndex link_two = path_two[path_two_size - i];
    if (link_one != link_two) {
      detour_end = i;
      break;
    }
  }

  return {detour_start, path_two.size() - detour_end};
}

std::string GraphNodeSetToString(const GraphNodeSet& nodes,
                                 const GraphStorage* graph_storage) {
  std::vector<std::string> node_names;
  for (GraphNodeIndex node_index : nodes) {
    node_names.emplace_back(graph_storage->GetNode(node_index)->id());
  }
  return StrCat("{", Join(node_names, ","), "}");
}

std::string GraphLinkSetToString(const GraphLinkSet& links,
                                 const GraphStorage* graph_storage) {
  std::vector<std::string> link_names;
  for (GraphLinkIndex link_index : links) {
    link_names.emplace_back(
        graph_storage->GetLink(link_index)->ToStringNoPorts());
  }
  return StrCat("{", Join(link_names, ","), "}");
}

LinkSequence::LinkSequence() : delay_(Delay::zero()) {}

LinkSequence::LinkSequence(const Links& links, Delay delay,
                           bool check_for_duplicates)
    : links_(links), delay_(delay) {
  if (check_for_duplicates) {
    CHECK(!HasDuplicateLinks(links)) << "Duplicate link";
  }
}

LinkSequence::LinkSequence(const Links& links, const GraphStorage* storage,
                           bool check_for_duplicates)
    : LinkSequence(links, TotalDelayOfLinks(links, storage),
                   check_for_duplicates) {}

bool LinkSequence::Contains(GraphLinkIndex link) const {
  return std::find(links_.begin(), links_.end(), link) != links_.end();
}

bool LinkSequence::ContainsAny(GraphLinkSet links) const {
  for (GraphLinkIndex link_index : links_) {
    if (links.Contains(link_index)) {
      return true;
    }
  }

  return false;
}

bool LinkSequence::HasDuplicateNodes(const GraphStorage* graph_storage) const {
  if (links_.empty()) {
    return false;
  }

  GraphNodeSet nodes;
  GraphNodeIndex src_of_path = graph_storage->GetLink(links_[0])->src();
  nodes.Insert(src_of_path);
  for (size_t i = 0; i < links_.size(); ++i) {
    GraphNodeIndex dst = graph_storage->GetLink(links_[i])->dst();
    if (nodes.Contains(dst)) {
      return true;
    }
    nodes.Insert(dst);
  }

  return false;
}

std::string LinkSequence::ToString(const GraphStorage* storage) const {
  std::stringstream ss;
  ss << "[";

  for (const auto& edge : links_) {
    const GraphLink* link = storage->GetLink(edge);
    ss << link->ToString();

    if (edge != links_.back()) {
      ss << ", ";
    }
  }

  ss << "]";
  return ss.str();
}

std::string LinkSequence::ToStringNoPorts(const GraphStorage* storage) const {
  std::stringstream ss;
  if (links_.empty()) {
    return "[]";
  }

  ss << "[";
  for (const auto& edge : links_) {
    const GraphLink* link = storage->GetLink(edge);
    ss << link->src_node()->id() << "->";
  }

  const GraphLink* link = storage->GetLink(links_.back());
  ss << link->dst_node()->id();
  ss << "] ";
  ss << StrCat(delay_.count(), "μs");
  return ss.str();
}

std::string LinkSequence::ToStringIdsOnly() const {
  if (links_.empty()) {
    return "[]";
  }

  std::vector<std::string> links_str;
  for (GraphLinkIndex link : links_) {
    links_str.emplace_back(std::to_string(link));
  }

  return StrCat("[", Join(links_str, "->"), "] ", delay_.count(), "μs");
}

GraphNodeIndex LinkSequence::FirstHop(const GraphStorage* storage) const {
  DCHECK(!links_.empty());
  GraphLinkIndex first_link = links_.front();
  return storage->GetLink(first_link)->src();
}

GraphNodeIndex LinkSequence::LastHop(const GraphStorage* storage) const {
  DCHECK(!links_.empty());
  GraphLinkIndex last_link = links_.back();
  return storage->GetLink(last_link)->dst();
}

size_t LinkSequence::InMemBytesEstimate() const {
  return links_.capacity() * sizeof(Links::value_type) + sizeof(*this);
}

Delay LinkSequence::delay() const { return delay_; }

bool operator<(const LinkSequence& lhs, const LinkSequence& rhs) {
  net::Delay lhs_delay = lhs.delay();
  net::Delay rhs_delay = rhs.delay();
  if (lhs_delay != rhs_delay) {
    return lhs_delay < rhs_delay;
  }

  return lhs.links() < rhs.links();
}

bool operator==(const LinkSequence& lhs, const LinkSequence& rhs) {
  return lhs.links_ == rhs.links_;
}

bool operator!=(const LinkSequence& lhs, const LinkSequence& rhs) {
  return lhs.links_ != rhs.links_;
}

std::string GraphPath::ToString() const {
  return link_sequence_.ToString(storage_);
}

std::string GraphPath::ToStringNoPorts() const {
  using namespace std::chrono;
  double delay_ms = duration<double, milliseconds::period>(delay()).count();
  return Substitute("$0 $1ms", link_sequence_.ToStringNoPorts(storage_),
                    delay_ms);
}

GraphNodeIndex GraphPath::FirstHop() const {
  return link_sequence_.FirstHop(storage_);
}

GraphNodeIndex GraphPath::LastHop() const {
  return link_sequence_.LastHop(storage_);
}

void GraphPath::Populate(LinkSequence link_sequence, uint32_t tag) {
  link_sequence_ = link_sequence;
  tag_ = tag;
}

LinkSequence GraphStorage::LinkSequenceFromStringOrDie(
    const std::string& path_string) const {
  CHECK(path_string.length() > 1) << "Path string malformed: " << path_string;
  CHECK(path_string.front() == '[' && path_string.back() == ']')
      << "Path string malformed: " << path_string;

  std::string inner = path_string.substr(1, path_string.size() - 2);
  if (inner.empty()) {
    // Empty path
    return {};
  }

  std::vector<std::string> edge_strings = Split(inner, ", ");
  Links links;

  for (const auto& edge_string : edge_strings) {
    std::vector<std::string> src_and_dst = Split(edge_string, "->");

    CHECK(src_and_dst.size() == 2) << "Path string malformed: " << path_string;
    std::string src = src_and_dst[0];
    std::string dst = src_and_dst[1];
    CHECK(src.size() > 0 && dst.size() > 0) << "Path string malformed: "
                                            << path_string;

    net::PBGraphLink link_pb;
    link_pb.set_src(src);
    link_pb.set_dst(dst);

    links.push_back(LinkFromProtobufOrDie(link_pb));
  }

  return {links, TotalDelayOfLinks(links, this)};
}

const GraphPath* GraphStorage::PathFromStringOrDie(
    const std::string& path_string, uint64_t cookie) {
  LinkSequence link_sequence = LinkSequenceFromStringOrDie(path_string);
  return PathFromLinksOrDie(link_sequence, cookie);
}

const GraphPath* GraphStorage::PathFromLinksOrDie(
    const LinkSequence& link_sequence, uint64_t cookie) {
  if (link_sequence.empty()) {
    return empty_path_.get();
  }

  const GraphPath* return_path;
  std::map<Links, GraphPath>& path_map = cookie_to_paths_[cookie];
  auto iterator_and_added = path_map.emplace(
      std::piecewise_construct, std::forward_as_tuple(link_sequence.links()),
      std::forward_as_tuple(this));
  if (iterator_and_added.second) {
    auto& it = iterator_and_added.first;
    it->second.Populate(link_sequence, ++tag_generator_);
    return_path = &(it->second);
  } else {
    auto& it = iterator_and_added.first;
    return_path = &(it->second);
  }

  return return_path;
}

const GraphPath* GraphStorage::PathFromProtobufOrDie(
    const google::protobuf::RepeatedPtrField<PBGraphLink>& links_pb,
    uint64_t cookie) {
  Links links;
  const std::string* old_dst = nullptr;
  for (const auto& link_pb : links_pb) {
    links.push_back(LinkFromProtobufOrDie(link_pb));

    const std::string& src = link_pb.src();
    const std::string& dst = link_pb.dst();

    CHECK(old_dst == nullptr || *old_dst == src) << "Path not contiguous";
    old_dst = &dst;
  }

  return PathFromLinksOrDie({links, TotalDelayOfLinks(links, this)}, cookie);
}

const GraphPath* GraphStorage::PathFromProtobufOrDie(
    const std::vector<PBGraphLink>& links, uint64_t cookie) {
  google::protobuf::RepeatedPtrField<PBGraphLink> links_pb;
  for (const auto& link : links) {
    *links_pb.Add() = link;
  }

  return PathFromProtobufOrDie(links_pb, cookie);
}

std::string GraphStorage::DumpPaths() const {
  using namespace std::chrono;

  static std::stringstream out;
  for (const auto& cookie_and_path_map : cookie_to_paths_) {
    const std::map<Links, GraphPath>& links_to_path =
        cookie_and_path_map.second;

    for (const auto& links_and_path : links_to_path) {
      const GraphPath& path = links_and_path.second;
      double delay_ms =
          duration<double, milliseconds::period>(path.delay()).count();
      out << Substitute("$0|$1|$2|$3\n", path.ToStringNoPorts(), path.tag(),
                        cookie_and_path_map.first, delay_ms);
    }
  }

  return out.str();
}

const GraphPath* GraphStorage::FindPathByTagOrNull(uint32_t tag) const {
  for (const auto& cookie_and_path_map : cookie_to_paths_) {
    const std::map<Links, GraphPath>& links_to_path =
        cookie_and_path_map.second;

    for (const auto& links_and_path : links_to_path) {
      const GraphPath& path = links_and_path.second;
      if (path.tag() == tag) {
        return &path;
      }
    }
  }

  return nullptr;
}

bool IsInPaths(const std::string& needle,
               const std::vector<LinkSequence>& haystack,
               GraphStorage* storage) {
  const GraphPath* path = storage->PathFromStringOrDie(needle, 0);

  for (const LinkSequence& path_in_haystack : haystack) {
    if (path_in_haystack.links() == path->link_sequence().links()) {
      return true;
    }
  }

  return false;
}

bool IsInPaths(const std::string& needle,
               const std::vector<const GraphPath*>& haystack, uint64_t cookie,
               GraphStorage* storage) {
  const GraphPath* path = storage->PathFromStringOrDie(needle, cookie);

  for (const GraphPath* path_in_haystack : haystack) {
    if (path_in_haystack == path) {
      return true;
    }
  }

  return false;
}

const FiveTuple FiveTuple::kDefaultTuple = {};

std::string FiveTuple::ToString() const {
  std::stringstream ss;
  ss << *this;
  return ss.str();
}

std::ostream& operator<<(std::ostream& output, const FiveTuple& op) {
  output << Substitute("(src: $0, dst: $1, proto: $2, sport: $3, dport: $4)",
                       IPToStringOrDie(op.ip_src()),
                       IPToStringOrDie(op.ip_dst()), op.ip_proto().Raw(),
                       op.src_port().Raw(), op.dst_port().Raw());

  return output;
}

bool operator==(const FiveTuple& a, const FiveTuple& b) {
  return std::tie(a.ip_src_, a.ip_dst_, a.ip_proto_, a.src_port_,
                  a.dst_port_) ==
         std::tie(b.ip_src_, b.ip_dst_, b.ip_proto_, b.src_port_, b.dst_port_);
}

bool operator!=(const FiveTuple& a, const FiveTuple& b) {
  return std::tie(a.ip_src_, a.ip_dst_, a.ip_proto_, a.src_port_,
                  a.dst_port_) !=
         std::tie(b.ip_src_, b.ip_dst_, b.ip_proto_, b.src_port_, b.dst_port_);
}

bool operator<(const FiveTuple& a, const FiveTuple& b) {
  return std::tie(a.ip_src_, a.ip_dst_, a.ip_proto_, a.src_port_, a.dst_port_) <
         std::tie(b.ip_src_, b.ip_dst_, b.ip_proto_, b.src_port_, b.dst_port_);
}

bool operator>(const FiveTuple& a, const FiveTuple& b) {
  return std::tie(a.ip_src_, a.ip_dst_, a.ip_proto_, a.src_port_, a.dst_port_) >
         std::tie(b.ip_src_, b.ip_dst_, b.ip_proto_, b.src_port_, b.dst_port_);
}

std::string IPToStringOrDie(IPAddress ip) {
  char str[INET_ADDRSTRLEN];
  uint32_t address = htonl(ip.Raw());
  const char* return_ptr = inet_ntop(AF_INET, &address, str, INET_ADDRSTRLEN);
  CHECK(return_ptr != nullptr) << "Unable to convert IPv4 to string: "
                               << strerror(errno);
  return std::string(str);
}

IPAddress StringToIPOrDie(const std::string& str) {
  uint32_t address;
  int return_value = inet_pton(AF_INET, str.c_str(), &address);
  CHECK(return_value != 0) << "Invalid IPv4 string: " << str;
  CHECK(return_value != -1) << "Unable to convert string to IPv4: "
                            << strerror(errno);
  CHECK(return_value == 1);
  return IPAddress(ntohl(address));
}

IPAddress MaskAddress(IPAddress ip_address, uint8_t mask_len) {
  CHECK(mask_len <= kMaxIPAddressMaskLen);
  uint8_t slack = kMaxIPAddressMaskLen - mask_len;
  uint64_t mask = ~((1 << slack) - 1);
  return IPAddress(ip_address.Raw() & mask);
}

IPRange::IPRange(IPAddress address, uint8_t mask_len)
    : base_address_(IPAddress::Zero()) {
  Init(address, mask_len);
}

IPRange::IPRange(const std::string& range_str)
    : base_address_(IPAddress::Zero()) {
  std::vector<std::string> pieces = Split(range_str, kDelimiter);
  CHECK(pieces.size() == 2) << "Wrong number of delimited pieces";
  const std::string& address_str = pieces.front();
  const std::string& mask_str = pieces.back();

  uint32_t mask_len;
  CHECK(safe_strtou32(mask_str, &mask_len)) << "Bad mask: " << mask_str;
  Init(StringToIPOrDie(address_str), mask_len);
}

std::string IPRange::ToString() const {
  return StrCat(IPToStringOrDie(base_address_), "/", SimpleItoa(mask_len_));
}

void IPRange::Init(IPAddress address, uint8_t mask_len) {
  mask_len_ = mask_len;
  base_address_ = MaskAddress(address, mask_len);
}

// Returns the index of the cluster a node belongs to.
static size_t IndexOfRegionOrDie(const PBNet& graph, const std::string& node) {
  size_t region_index = std::numeric_limits<size_t>::max();
  for (int i = 0; i < graph.regions_size(); ++i) {
    for (const std::string& cluster_node : graph.regions(i).nodes()) {
      if (cluster_node == node) {
        region_index = i;
      }
    }
  }

  CHECK(region_index != std::numeric_limits<size_t>::max())
      << "Node not in any region: " << node;
  return region_index;
}

size_t GraphStorage::IndexOfRegionOrDie(GraphNodeIndex node) const {
  for (size_t i = 0; i < regions_.size(); ++i) {
    const GraphNodeSet& nodes_in_region = regions_[i];

    if (nodes_in_region.Contains(node)) {
      return i;
    }
  }

  LOG(FATAL) << "Node not in any region: " << node;
  return 0;
}

std::set<std::string> NodesInSameRegionOrDie(const PBNet& graph,
                                             const std::string& node) {
  size_t region_index = IndexOfRegionOrDie(graph, node);

  std::set<std::string> return_set;
  for (const std::string& cluster_node : graph.regions(region_index).nodes()) {
    if (cluster_node != node) {
      return_set.emplace(cluster_node);
    }
  }

  return return_set;
}

GraphNodeSet GraphStorage::NodesInSameRegionOrDie(GraphNodeIndex node) const {
  size_t region_index = IndexOfRegionOrDie(node);
  GraphNodeSet to_return = regions_[region_index];
  to_return.Remove(node);
  return to_return;
}

std::set<std::string> NodesInOtherRegionsOrDie(const PBNet& graph,
                                               const std::string& node) {
  int region_index = IndexOfRegionOrDie(graph, node);

  std::set<std::string> return_set;
  for (int i = 0; i < graph.regions_size(); ++i) {
    if (i != region_index) {
      const auto& nodes_in_cluster = graph.regions(i).nodes();
      return_set.insert(nodes_in_cluster.begin(), nodes_in_cluster.end());
    }
  }

  return return_set;
}

GraphNodeSet GraphStorage::NodesInOtherRegionsOrDie(GraphNodeIndex node) const {
  size_t region_index = IndexOfRegionOrDie(node);
  GraphNodeSet to_return;
  for (size_t i = 0; i < regions_.size(); ++i) {
    if (i != region_index) {
      to_return.InsertAll(regions_[i]);
    }
  }

  return to_return;
}

bool IsNodeInGraph(const PBNet& graph, const std::string& node) {
  for (const auto& link : graph.links()) {
    if (link.src() == node || link.dst() == node) {
      return true;
    }
  }

  return false;
}

bool IsIntraClusterLink(const PBNet& graph, const PBGraphLink& link) {
  const std::string& src = link.src();
  std::set<std::string> in_same_cluster = NodesInSameRegionOrDie(graph, src);
  return ContainsKey(in_same_cluster, link.dst());
}

GraphLink::GraphLink(const net::PBGraphLink& link_pb, GraphNodeIndex src,
                     GraphNodeIndex dst, const GraphNode* src_node,
                     const GraphNode* dst_node)
    : src_(src),
      dst_(dst),
      src_port_(link_pb.src_port()),
      dst_port_(link_pb.dst_port()),
      bandwidth_(Bandwidth::FromBitsPerSecond(link_pb.bandwidth_bps())),
      src_node_(src_node),
      dst_node_(dst_node) {
  using namespace std::chrono;
  CHECK(link_pb.delay_sec() != 0) << "Link has zero delay";
  CHECK(link_pb.bandwidth_bps() != 0) << "Link has zero bandwidth";
  duration<double> duration(link_pb.delay_sec());
  delay_ = duration_cast<Delay>(duration);
}

GraphLink::GraphLink(GraphNodeIndex src, GraphNodeIndex dst,
                     DevicePortNumber src_port, DevicePortNumber dst_port,
                     Bandwidth bw, Delay delay, const GraphNode* src_node,
                     const GraphNode* dst_node)
    : src_(src),
      dst_(dst),
      src_port_(src_port),
      dst_port_(dst_port),
      bandwidth_(bw),
      delay_(delay),
      src_node_(src_node),
      dst_node_(dst_node) {}

std::string GraphLink::ToString() const {
  return Substitute("$0:$1->$2:$3", src_node_->id(), src_port_.Raw(),
                    dst_node_->id(), dst_port_.Raw());
}

std::string GraphLink::ToStringNoPorts() const {
  return Substitute("$0->$1", src_node_->id(), dst_node_->id());
}

PBGraphLink GraphLink::ToProtobuf() const {
  PBGraphLink out_pb;
  out_pb.set_src(src_node_->id());
  out_pb.set_dst(dst_node_->id());
  out_pb.set_src_port(src_port_.Raw());
  out_pb.set_dst_port(dst_port_.Raw());
  out_pb.set_bandwidth_bps(bandwidth_.bps());

  double delay_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(delay_).count();
  out_pb.set_delay_sec(delay_sec);
  return out_pb;
}

PBGraphLink* FindEdgeOrDie(const std::string& src, const std::string& dst,
                           PBNet* net) {
  for (PBGraphLink& link : *net->mutable_links()) {
    if (link.src() == src && link.dst() == dst) {
      return &link;
    }
  }

  LOG(FATAL) << "No edge from " << src << " to " << dst;
  return nullptr;
}

bool IsPartitioned(const PBNet& graph) {
  std::map<std::string, size_t> node_to_index;
  size_t i = 0;
  for (const auto& link : graph.links()) {
    if (!ContainsKey(node_to_index, link.src())) {
      node_to_index[link.src()] = i++;
    }

    if (!ContainsKey(node_to_index, link.dst())) {
      node_to_index[link.dst()] = i++;
    }
  }

  const size_t inf = std::numeric_limits<size_t>::max();
  size_t nodes = node_to_index.size();
  std::vector<std::vector<size_t>> distances(nodes);
  for (size_t i = 0; i < nodes; ++i) {
    distances[i] = std::vector<size_t>(nodes, inf);
  }

  for (const auto& link : graph.links()) {
    size_t src_index = node_to_index[link.src()];
    size_t dst_index = node_to_index[link.dst()];
    distances[src_index][dst_index] = 1;
  }

  for (size_t k = 0; k < nodes; ++k) {
    distances[k][k] = 0;
  }

  for (size_t k = 0; k < nodes; ++k) {
    for (size_t i = 0; i < nodes; ++i) {
      for (size_t j = 0; j < nodes; ++j) {
        size_t via_k = (distances[i][k] == inf || distances[k][j] == inf)
                           ? inf
                           : distances[i][k] + distances[k][j];

        if (distances[i][j] > via_k) {
          distances[i][j] = via_k;
        }
      }
    }
  }

  // If the distances contain any std::numeric_limits<size_t>::max() then the
  // network is partitioned.
  for (size_t i = 0; i < nodes; ++i) {
    for (size_t j = 0; j < nodes; ++j) {
      if (distances[i][j] == inf) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace net
}  // namespace ncode
