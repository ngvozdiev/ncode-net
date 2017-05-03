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

std::chrono::microseconds TotalDelayOfLinks(const Links& links,
                                            const GraphStorage& graph_storage) {
  std::chrono::microseconds total(0);
  for (GraphLinkIndex link_index : links) {
    const GraphLink* link = graph_storage.GetLink(link_index);
    total += link->delay();
  }

  return total;
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

    GraphLinkBase new_link_base(src_cluster_name, dst_cluster_name, port, port,
                                link->bandwidth(), link->delay());
    auto link_ptr = std::unique_ptr<GraphLink>(
        new GraphLink(new_link_base, src_cluster_index, dst_cluster_index,
                      new_storage->GetNode(src_cluster_index),
                      new_storage->GetNode(dst_cluster_index)));
    GraphLinkIndex clustered_link_index =
        new_storage->link_store_.MoveItem(std::move(link_ptr));
    real_to_clustered_links->Add(link_index, clustered_link_index);
    new_storage->links_[src_cluster_name][dst_cluster_name].emplace_back(
        clustered_link_index);
  }

  new_storage->PopulateAdjacencyList();
  return new_storage;
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

GraphStorage::GraphStorage(const GraphBuilder& graph_builder) {
  for (const auto& link_base : graph_builder.links()) {
    const std::string& src_id = link_base.src_id();
    const std::string& dst_id = link_base.dst_id();

    auto src_index = NodeFromString(src_id);
    auto dst_index = NodeFromString(dst_id);
    auto link_ptr = std::unique_ptr<GraphLink>(
        new GraphLink(link_base, src_index, dst_index, GetNode(src_index),
                      GetNode(dst_index)));
    GraphLinkIndex index = link_store_.MoveItem(std::move(link_ptr));
    links_[src_id][dst_id].emplace_back(index);
  }

  PopulateAdjacencyList();
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

bool HasDuplicateNodes(const Links& links, const GraphStorage& graph_storage) {
  if (links.empty()) {
    return false;
  }

  GraphNodeSet nodes;
  GraphNodeIndex src_of_path = graph_storage.GetLink(links[0])->src();
  nodes.Insert(src_of_path);
  for (size_t i = 0; i < links.size(); ++i) {
    GraphNodeIndex dst = graph_storage.GetLink(links[i])->dst();
    if (nodes.Contains(dst)) {
      return true;
    }
    nodes.Insert(dst);
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
                                 const GraphStorage& graph_storage) {
  std::vector<std::string> node_names;
  for (GraphNodeIndex node_index : nodes) {
    node_names.emplace_back(graph_storage.GetNode(node_index)->id());
  }
  return StrCat("{", Join(node_names, ","), "}");
}

std::string GraphLinkSetToString(const GraphLinkSet& links,
                                 const GraphStorage& graph_storage) {
  std::vector<std::string> link_names;
  for (GraphLinkIndex link_index : links) {
    link_names.emplace_back(
        graph_storage.GetLink(link_index)->ToStringNoPorts());
  }
  return StrCat("{", Join(link_names, ","), "}");
}

Walk::Walk() : delay_(Delay::zero()) {}

Walk::Walk(const Links& links, Delay delay) : links_(links), delay_(delay) {}

Walk::Walk(const Links& links, const GraphStorage& storage)
    : Walk(links, TotalDelayOfLinks(links, storage)) {}

bool Walk::Contains(GraphLinkIndex link) const {
  return std::find(links_.begin(), links_.end(), link) != links_.end();
}

bool Walk::ContainsAny(GraphLinkSet links) const {
  for (GraphLinkIndex link_index : links_) {
    if (links.Contains(link_index)) {
      return true;
    }
  }

  return false;
}

bool Walk::IsTrail() const { return !HasDuplicateLinks(links_); }

bool Walk::IsPath(const GraphStorage& graph_storage) const {
  return !HasDuplicateNodes(links_, graph_storage);
}

std::string Walk::ToString(const GraphStorage& storage) const {
  std::stringstream ss;
  ss << "[";

  for (const auto& edge : links_) {
    const GraphLink* link = storage.GetLink(edge);
    ss << link->ToString();

    if (edge != links_.back()) {
      ss << ", ";
    }
  }

  ss << "]";
  return ss.str();
}

std::string Walk::ToStringNoPorts(const GraphStorage& storage) const {
  std::stringstream ss;
  if (links_.empty()) {
    return "[]";
  }

  ss << "[";
  for (const auto& edge : links_) {
    const GraphLink* link = storage.GetLink(edge);
    ss << link->src_node()->id() << "->";
  }

  const GraphLink* link = storage.GetLink(links_.back());
  ss << link->dst_node()->id();
  ss << "] ";
  ss << StrCat(delay_.count(), "μs");
  return ss.str();
}

std::string Walk::ToStringIdsOnly(const GraphStorage& storage) const {
  std::stringstream ss;
  if (links_.empty()) {
    return "[]";
  }

  ss << "[";
  for (const auto& edge : links_) {
    const GraphLink* link = storage.GetLink(edge);
    ss << link->src() << "->";
  }

  const GraphLink* link = storage.GetLink(links_.back());
  ss << link->dst();
  ss << "] ";
  ss << StrCat(delay_.count(), "μs");
  return ss.str();
}

GraphNodeIndex Walk::FirstHop(const GraphStorage& storage) const {
  DCHECK(!links_.empty());
  GraphLinkIndex first_link = links_.front();
  return storage.GetLink(first_link)->src();
}

GraphNodeIndex Walk::LastHop(const GraphStorage& storage) const {
  DCHECK(!links_.empty());
  GraphLinkIndex last_link = links_.back();
  return storage.GetLink(last_link)->dst();
}

size_t Walk::InMemBytesEstimate() const {
  return links_.capacity() * sizeof(Links::value_type) + sizeof(*this);
}

Delay Walk::delay() const { return delay_; }

bool operator<(const Walk& lhs, const Walk& rhs) {
  net::Delay lhs_delay = lhs.delay();
  net::Delay rhs_delay = rhs.delay();
  if (lhs_delay != rhs_delay) {
    return lhs_delay < rhs_delay;
  }

  return lhs.links() < rhs.links();
}

bool operator>(const Walk& lhs, const Walk& rhs) {
  net::Delay lhs_delay = lhs.delay();
  net::Delay rhs_delay = rhs.delay();
  if (lhs_delay != rhs_delay) {
    return lhs_delay > rhs_delay;
  }

  return lhs.links() > rhs.links();
}

bool operator==(const Walk& lhs, const Walk& rhs) {
  return lhs.links_ == rhs.links_;
}

bool operator!=(const Walk& lhs, const Walk& rhs) {
  return lhs.links_ != rhs.links_;
}

std::unique_ptr<Walk> GraphStorage::WalkFromStringOrDie(
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
    links.push_back(LinkOrDie(src, dst));
  }

  return make_unique<Walk>(links, TotalDelayOfLinks(links, *this));
}

bool GraphStorage::IsInWalks(const std::string& needle,
                             const std::vector<Walk>& haystack) const {
  std::unique_ptr<Walk> walk = WalkFromStringOrDie(needle);

  for (const Walk& path_in_haystack : haystack) {
    if (path_in_haystack.links() == walk->links()) {
      return true;
    }
  }

  return false;
}

void GraphStorage::PopulateAdjacencyList() {
  simple_ = true;
  for (GraphLinkIndex link : AllLinks()) {
    const GraphLink* link_ptr = GetLink(link);
    GraphNodeIndex src = link_ptr->src();
    GraphNodeIndex dst = link_ptr->dst();
    for (const auto& link_info : adjacency_list_.GetNeighbors(src)) {
      if (link_info.dst_index == dst) {
        simple_ = false;
      }
    }

    adjacency_list_.AddLink(link, src, dst, link_ptr->delay());
  }
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

std::string GraphLinkBase::ToString() const {
  return Substitute("$0:$1->$2:$3", src_id_, src_port_.Raw(), dst_id_,
                    dst_port_.Raw());
}

std::string GraphLinkBase::ToStringNoPorts() const {
  return Substitute("$0->$1", src_id_, dst_id_);
}

void GraphBuilder::AddLink(const GraphLinkBase& link) {
  CHECK(!link.src_id().empty()) << "missing src id";
  CHECK(!link.dst_id().empty()) << "missing dst id";
  CHECK(link.src_id() != link.dst_id()) << "src id same as dst id";

  if (!auto_port_numbers_) {
    CHECK(link.src_port() != DevicePortNumber::Zero());
    CHECK(link.dst_port() != DevicePortNumber::Zero());
    links_.emplace_back(link);
    return;
  }

  CHECK(link.src_port() == DevicePortNumber::Zero());
  CHECK(link.dst_port() == DevicePortNumber::Zero());

  DevicePortNumber port_num(links_.size() + 1);
  links_.emplace_back(link.src_id(), link.dst_id(), port_num, port_num,
                      link.bandwidth(), link.delay());
}

bool operator==(const GraphLinkBase& a, const GraphLinkBase& b) {
  return std::tie(a.src_id_, a.dst_id_, a.src_port_, a.dst_port_, a.bandwidth_,
                  a.delay_) == std::tie(b.src_id_, b.dst_id_, b.src_port_,
                                        b.dst_port_, b.bandwidth_, b.delay_);
}

}  // namespace net
}  // namespace ncode
