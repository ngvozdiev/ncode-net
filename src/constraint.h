#ifndef NCODE_NET_CONSTRAINT_H
#define NCODE_NET_CONSTRAINT_H

#include <ncode/ncode_common/common.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "algorithm.h"
#include "net_common.h"

namespace nc {
namespace net {

class ShortestPathGenerator {
 public:
  virtual ~ShortestPathGenerator() {}

  // Returns the next shortest compliant path, or an empty path if there are no
  // more policy compliant paths.
  virtual net::LinkSequence NextPath() = 0;
};

// A constraint that can be used by algorithm instances to check for compliance.
class Constraint {
 public:
  virtual ~Constraint() {}

  // Whether or not a path complies.
  virtual bool PathComplies(const net::LinkSequence& link_sequence) const = 0;

  // Returns an object that can be used to get compliant paths in order of
  // increasing delay.
  virtual std::unique_ptr<ShortestPathGenerator> PathGenerator(
      const DirectedGraph& graph, GraphNodeIndex src, GraphNodeIndex dst,
      const GraphLinkSet* to_exclude) const = 0;

  virtual std::string ToString(const net::GraphStorage* storage) const = 0;

  // Creates a new constraint that is equivalent to this one, but excludes the
  // given link in addition.
  virtual std::unique_ptr<Constraint> ExcludeLinks(
      const GraphLinkSet& links) const = 0;

 protected:
  Constraint() {}

  DISALLOW_COPY_AND_ASSIGN(Constraint);
};

// A conjunctions is a list of links that should be visited in order, and a set
// of links that should not be visited.
class Conjunction : public Constraint {
 public:
  Conjunction(const GraphLinkSet& to_exclude, const Links& to_visit);

  bool PathComplies(const net::LinkSequence& link_sequence) const override;

  std::unique_ptr<ShortestPathGenerator> PathGenerator(
      const DirectedGraph& graph, GraphNodeIndex src, GraphNodeIndex dst,
      const GraphLinkSet* to_exclude) const override;

  std::string ToString(const net::GraphStorage* storage) const override;

  std::unique_ptr<Constraint> ExcludeLinks(
      const GraphLinkSet& links) const override;

 private:
  Conjunction() {}

  static void AddFromPath(const DirectedGraph& graph, const LinkSequence& path,
                          Links* out, GraphNodeSet* nodes);

  GraphLinkSet to_exclude_;
  Links to_visit_;
};

// A disjunction is a series of conjunctions.
class Disjunction : public Constraint {
 public:
  Disjunction() {}

  void AddConjunction(std::unique_ptr<Conjunction> conjunction) {
    conjunctions_.emplace_back(std::move(conjunction));
  }

  bool PathComplies(const net::LinkSequence& link_sequence) const override;

  std::unique_ptr<ShortestPathGenerator> PathGenerator(
      const DirectedGraph& graph, GraphNodeIndex src, GraphNodeIndex dst,
      const GraphLinkSet* to_exclude) const override;

  std::string ToString(const net::GraphStorage* storage) const override;

  std::unique_ptr<Constraint> ExcludeLinks(
      const GraphLinkSet& links) const override;

 private:
  std::vector<std::unique_ptr<Conjunction>> conjunctions_;
};

// A dummy constraint that does no filtering.
std::unique_ptr<Constraint> DummyConstraint();

}  // namespace net
}  // namespace nc

#endif /* NCODE_NET_CONSTRAINT_H */
