// Copyright (c) 2019, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_MANIPULATION_GRAPH_VALIDATION_REPORT_HH
#define HPP_MANIPULATION_GRAPH_VALIDATION_REPORT_HH

#include <hpp/core/validation-report.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/fwd.hh>
#include <hpp/manipulation/graph/graph.hh>
#include <string>
#include <vector>

namespace hpp {
namespace manipulation {
namespace graph {
/// \addtogroup constraint_graph
/// \{

/// Check that graph components are valid.
///
/// A stringified validation report can be obtained via
/// Validation::print or operator<< (std::ostream&, const Validation&).
class HPP_MANIPULATION_DLLAPI Validation {
 public:
  typedef std::vector<std::string> Collision;
  typedef std::vector<Collision> CollisionList;
  typedef std::map<std::string, CollisionList> CollisionMap;
  Validation(const core::ProblemPtr_t& problem) : problem_(problem) {}

  void clear() {
    warnings_.clear();
    errors_.clear();
  }

  bool hasWarnings() const { return !warnings_.empty(); }

  bool hasErrors() const { return !errors_.empty(); }

  virtual std::ostream& print(std::ostream& os) const;

  /// Validate a graph component.
  /// It dynamically casts in order to call the right function among
  /// the validation method below.
  ///
  /// \return true if the component could not be proven infeasible.
  /// \note Even if true is returned, the report can contain warnings.
  bool validate(const GraphComponentPtr_t& comp);

  /// Validate a state
  /// \return true if the state could not be proven infeasible.
  /// \note Even if true is returned, the report can contain warnings.
  bool validateState(const StatePtr_t& state);

  /// Validate an edge
  /// \return true if the edge could not be proven infeasible.
  /// \note Even if true is returned, the report can contain warnings.
  bool validateEdge(const EdgePtr_t& edge);

  /// Validate an graph
  /// \return true if no component of the graph could not be proven infeasible.
  /// \note Even if true is returned, the report can contain warnings.
  bool validateGraph(const GraphPtr_t& graph);

  CollisionList getCollisionsForNode(const std::string& nodeName) {
    return collisions_[nodeName];
  }

 private:
  void addWarning(const GraphComponentPtr_t& c, const std::string& w) {
    warnings_.push_back(Message(c, w));
  }

  void addError(const GraphComponentPtr_t& c, const std::string& w) {
    errors_.push_back(Message(c, w));
  }

  void addCollision(const GraphComponentPtr_t& c, const std::string& obj1,
                    const std::string& obj2) {
    Collision coll = Collision{obj1, obj2};
    collisions_[c->name()].push_back(coll);
  }

  typedef std::pair<GraphComponentPtr_t, std::string> Message;
  std::vector<Message> warnings_, errors_;
  CollisionMap collisions_;

  core::ProblemPtr_t problem_;
};

inline std::ostream& operator<<(std::ostream& os, const Validation& v) {
  return v.print(os);
}

/// \}
}  // namespace graph
}  // namespace manipulation

}  // namespace hpp

#endif  // HPP_MANIPULATION_GRAPH_VALIDATION_REPORT_HH
