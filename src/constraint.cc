#include "constraint.h"

#include <ncode/ncode_common/logging.h>
#include <ncode/ncode_common/perfect_hash.h>
#include <ncode/ncode_common/substitute.h>
#include <stddef.h>
#include <chrono>
#include <queue>

namespace nc {
namespace net {

Conjunction::Conjunction(const GraphLinkSet& to_avoid,
                         const std::vector<GraphLinkIndex>& to_visit)
    : to_exclude_(to_avoid), to_visit_(to_visit) {}

bool Conjunction::PathComplies(const net::LinkSequence& link_sequence) const {
  size_t visited_index = 0;
  for (GraphLinkIndex link : link_sequence.links()) {
    if (to_exclude_.Contains(link)) {
      return false;
    }

    if (to_visit_.size() > visited_index) {
      if (link == to_visit_[visited_index]) {
        ++visited_index;
      }
    }
  }

  return visited_index == to_visit_.size();
}

// Thin wrapper around a KShortestPaths instance
class KSPathGenerator : public ShortestPathGenerator {
 public:
  net::LinkSequence NextPath() { return k_shortest_paths_->NextPath(); }

 private:
  KSPathGenerator() {}

  GraphLinkSet to_exclude_;
  std::unique_ptr<KShortestPaths> k_shortest_paths_;

  friend class Conjunction;
};

std::unique_ptr<ShortestPathGenerator> Conjunction::PathGenerator(
    const DirectedGraph& graph, GraphNodeIndex src, GraphNodeIndex dst,
    const GraphLinkSet* to_exclude) const {
  KSPathGenerator* generator = new KSPathGenerator();
  GraphSearchAlgorithmConfig config;
  if (to_exclude && !to_exclude->Empty()) {
    config.AddToExcludeLinks(to_exclude);
  }

  if (!to_exclude_.Empty()) {
    generator->to_exclude_ = to_exclude_;
    config.AddToExcludeLinks(&generator->to_exclude_);
  }

  generator->k_shortest_paths_ =
      make_unique<KShortestPaths>(config, to_visit_, src, dst, &graph);
  return std::unique_ptr<ShortestPathGenerator>(generator);
}

std::unique_ptr<Constraint> Conjunction::ExcludeLinks(
    const GraphLinkSet& links) const {
  Conjunction* to_return = new Conjunction();
  to_return->to_exclude_ = to_exclude_;
  to_return->to_exclude_.InsertAll(links);
  to_return->to_visit_ = to_visit_;
  return std::unique_ptr<Constraint>(to_return);
}

std::string Conjunction::ToString(const net::GraphStorage* storage) const {
  std::string to_avoid_str;
  std::string to_visit_str;

  Join(to_exclude_.begin(), to_exclude_.end(),
       ",", [storage](GraphLinkIndex link) {
         return storage->GetLink(link)->ToString();
       }, &to_avoid_str);

  Join(to_visit_.begin(), to_visit_.end(), ",", [storage](GraphLinkIndex link) {
    return storage->GetLink(link)->ToString();
  }, &to_avoid_str);

  return Substitute("(EXCLUDE $0, VISIT $1)", to_avoid_str, to_visit_str);
}

bool Disjunction::PathComplies(const net::LinkSequence& link_sequence) const {
  // The path complies if any of the conjunctions comply.
  for (const auto& conjunction : conjunctions_) {
    if (conjunction->PathComplies(link_sequence)) {
      return true;
    }
  }

  return false;
}

std::unique_ptr<Constraint> Disjunction::ExcludeLinks(
    const GraphLinkSet& links) const {
  std::vector<std::unique_ptr<Conjunction>> new_conjunctions;
  for (const auto& conjunction : conjunctions_) {
    auto new_conjunction = conjunction->ExcludeLinks(links);
    new_conjunctions.emplace_back(std::unique_ptr<Conjunction>(
        static_cast<Conjunction*>(new_conjunction.release())));
  }

  Disjunction* to_return = new Disjunction();
  to_return->conjunctions_ = std::move(new_conjunctions);
  return std::unique_ptr<Constraint>(to_return);
}

// A collection of generators, that are all polled and the least path is
// returned.
class DisjunctionPathGenerator : public ShortestPathGenerator {
 public:
  LinkSequence NextPath() {
    if (queue_.empty()) {
      return {};
    }

    PathGenAndPath top = queue_.top();
    queue_.pop();

    CHECK(!top.candidate.empty());
    LinkSequence to_return = std::move(top.candidate);

    top.candidate = top.generator->NextPath();
    if (!top.candidate.empty()) {
      queue_.push(top);
    }
    return to_return;
  }

 private:
  struct PathGenAndPath {
    PathGenAndPath(ShortestPathGenerator* generator, LinkSequence candidate)
        : generator(generator), candidate(candidate) {}

    ShortestPathGenerator* generator;
    LinkSequence candidate;
  };

  struct Comparator {
    bool operator()(const PathGenAndPath& lhs, const PathGenAndPath& rhs) {
      return lhs.candidate.delay() < rhs.candidate.delay();
    }
  };

  DisjunctionPathGenerator(
      std::vector<std::unique_ptr<ShortestPathGenerator>>* generators)
      : generators_(std::move(*generators)) {
    for (const auto& generator_ptr : generators_) {
      LinkSequence next_path = generator_ptr->NextPath();
      if (next_path.empty()) {
        continue;
      }

      queue_.emplace(generator_ptr.get(), next_path);
    }
  }

  std::priority_queue<PathGenAndPath, std::vector<PathGenAndPath>, Comparator>
      queue_;
  std::vector<std::unique_ptr<ShortestPathGenerator>> generators_;

  friend class Disjunction;
};

std::unique_ptr<ShortestPathGenerator> Disjunction::PathGenerator(
    const DirectedGraph& graph, GraphNodeIndex src, GraphNodeIndex dst,
    const GraphLinkSet* to_exclude) const {
  std::vector<std::unique_ptr<ShortestPathGenerator>> generators;
  for (const auto& conjunction : conjunctions_) {
    generators.emplace_back(
        conjunction->PathGenerator(graph, src, dst, to_exclude));
  }

  return std::unique_ptr<ShortestPathGenerator>(
      new DisjunctionPathGenerator(&generators));
}

std::string Disjunction::ToString(const net::GraphStorage* storage) const {
  std::string out;
  Join(conjunctions_.begin(), conjunctions_.end(),
       " OR ", [storage](const std::unique_ptr<Conjunction>& cj) {
         return cj->ToString(storage);
       }, &out);
  return out;
}

std::unique_ptr<Constraint> DummyConstraint() {
  GraphLinkSet set;
  Links to_visit;
  return make_unique<Conjunction>(set, to_visit);
}

}  // namespace nc
}  // namespace net
