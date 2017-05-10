#ifndef NCODE_NET_GEN_H
#define NCODE_NET_GEN_H

#include <chrono>
#include <cstdint>
#include <random>
#include <vector>

#include "net_common.h"
#include "algorithm.h"

namespace nc {
namespace net {

class PathExpression {
 public:
  PathExpression(const GraphStorage* graph) : graph_(graph) {}

  // Returns a generator capable of generating the K shortest paths that comply
  // with this expression.
  std::unique_ptr<DisjunctKShortestPathsGenerator> Evaluate(
      GraphNodeIndex src, GraphNodeIndex dst) const {}

  // Produces an expression which is the AND of this expression and another one.
  // Unlike other operations, AND needs to happen in the context of a source and
  // a destination.
  std::unique_ptr<PathExpression> And(const PathExpression& other,
                                      GraphNodeIndex src,
                                      GraphNodeIndex dst) const {
    // Only know how to handle two rules with one set of constraints each. When
    // multiple sets of constraints are disjuncted it is hard to figure out
    // which combination we should go for.
    CHECK(constraints_.size() == 1 && other.constraints_.size() == 1);

    // Will get for this and the other expression the best path.
    auto gen_this = Evaluate(src, dst);
    auto gen_other = other.Evaluate(src, dst);

    size_t best_generator_index_this;
    const Walk* best_walk_this =
        gen_this->KthShortestPathOrNull(0, &best_generator_index_this);

    size_t best_generator_index_other;
    const Walk* best_walk_other =
        gen_other->KthShortestPathOrNull(0, &best_generator_index_other);
    if (best_walk_other == nullptr || best_walk_this == nullptr) {
      return {};
    }

    // Now we have the two best paths from each expression. We also have for
    // each expression the index of the OR clause that generated the path. The
    // result of this AND will have an exclusion set that is the combination of
    // the exclusion sets of each expression's clause.
    const KShortestPathsGenerator* best_generator_this =
        gen_this->ksp_generator(best_generator_index_this);
    const KShortestPathsGenerator* best_generator_other =
        gen_other->ksp_generator(best_generator_index_other);

    const ConstraintSet& best_constraint_set_this =
        best_generator_this->constraints();
    const ConstraintSet& best_constraint_set_other =
        best_generator_other->constraints();

    // There are no guarantees that the new exclusion set avoids both paths, but
    // whatever rule we produce needs to be compliant with both exclusion sets.
    ExclusionSet new_exclusion_set;
    new_exclusion_set.AddAll(best_constraint_set_this.exclusion_set());
    new_exclusion_set.AddAll(best_constraint_set_other.exclusion_set());

    // For each of the two best paths need to figure out what the waypoints are.
    // Waypoints are the nodes that make the path compliant---they are one or
    // more nodes from each of the sets that the path needs to visit.
    std::vector<GraphNodeIndex> waypoints_this =
        best_constraint_set_this.Waypoints(best_walk_this->links(), *graph_);
    std::vector<GraphNodeIndex> waypoints_other =
        best_constraint_set_other.Waypoints(best_walk_other->links(), *graph_);

    std::vector<GraphNodeIndex> waypoints_combined =
        CombineWaypoints(src, dst, new_exclusion_set, graph_->AdjacencyList(),
                         {waypoints_this, waypoints_other});

    std::vector<GraphNodeSet> to_visit;
    // Now need to reverse the process---given the waypoints need to convert
    // back to sets.
    for (GraphNodeIndex waypoint : waypoints_combined) {
      const GraphNodeSet* set =
          best_constraint_set_this.FindSetToVisitOrNull(waypoint);
      if (set != nullptr) {
        to_visit.emplace_back(*set);
      }

      set = best_constraint_set_other.FindSetToVisitOrNull(waypoint);
      if (set != nullptr) {
        to_visit.emplace_back(*set);
      }
    }

    auto to_return = make_unique<PathExpression>(graph_);
    to_return->constraints_.emplace_back(new_exclusion_set, to_visit);
    return to_return;
  }

  std::unique_ptr<PathExpression> Or(const PathExpression& other) const {
    auto to_return = make_unique<PathExpression>(graph_);
    std::vector<ConstraintSet>& constraints_to_return = to_return->constraints_;

    constraints_to_return = constraints_;
    constraints_to_return.insert(constraints_to_return.end(),
                                 other.constraints_.begin(),
                                 other.constraints_.end());
    return to_return;
  }

  // Returns a new constraint set that is the result of 'second' added to
  // 'first'. The exclusion sets will be combined. The list of sets of nodes to
  // visit from the second one will be appended to the list of sets of nodes
  // from the first one.
  static ConstraintSet ExtendConstraintSet(const ConstraintSet& first,
                                           const ConstraintSet& second) {
    ConstraintSet return_set = first;
    return_set.Exclude().AddAll(second.exclusion_set());

    for (const auto& set_to_visit : second.to_visit()) {
      return_set.AddToVisitSet(set_to_visit);
    }

    return return_set;
  }

  std::unique_ptr<PathExpression> Then(const PathExpression& other) const {
    std::vector<ConstraintSet> new_constraints;
    for (const ConstraintSet& other_constraint_set : other.constraints_) {
      for (const ConstraintSet& constraint_set : constraints_) {
        new_constraints.emplace_back(
            ExtendConstraintSet(constraint_set, other_constraint_set));
      }
    }

    auto to_return = make_unique<PathExpression>(graph_);
    to_return->constraints_ = std::move(new_constraints);
    return to_return;
  }

  std::unique_ptr<PathExpression> Fallback(const PathExpression& other,
                                           GraphNodeIndex src,
                                           GraphNodeIndex dst) const {
    auto gen = Evaluate(src, dst);
    const Walk* sp = gen->KthShortestPathOrNull(0);
    if (sp != nullptr) {
      auto to_return = make_unique<PathExpression>(graph_);
      to_return->constraints_ = constraints_;
      return to_return;
    }

    auto to_return = make_unique<PathExpression>(graph_);
    to_return->constraints_ = other.constraints_;
    return to_return;
  }

 private:
  // The disjunction of multiple constraint sets.
  std::vector<ConstraintSet> constraints_;

  // The graph.
  const GraphStorage* graph_;
};

}  // namespace net
}  // namespace ncode
#endif
