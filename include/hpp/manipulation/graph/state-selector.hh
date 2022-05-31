// Copyright (c) 2014, LAAS-CNRS
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

#ifndef HPP_MANIPULATION_GRAPH_STATE_SELECTOR_HH
#define HPP_MANIPULATION_GRAPH_STATE_SELECTOR_HH

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/state.hh"

namespace hpp {
namespace manipulation {
namespace graph {
/// This class is used to get the state of a configuration. States have to
/// be ordered because a configuration can be in several states.
class HPP_MANIPULATION_DLLAPI StateSelector {
 public:
  virtual ~StateSelector(){};

  /// Create a new StateSelector.
  static StateSelectorPtr_t create(const std::string& name);

  const std::string& name() const { return name_; }

  /// Create an empty state
  StatePtr_t createState(const std::string& name, bool waypoint = false,
                         const int w = 0);

  /// Returns the state of a configuration.
  StatePtr_t getState(ConfigurationIn_t config) const;

  /// Returns the state of a roadmap state
  StatePtr_t getState(RoadmapNodePtr_t node) const;

  /// Returns a list of all the states
  States_t getStates() const;

  /// Returns a list of all the states
  States_t getWaypointStates() const;

  /// Select randomly an outgoing edge of the given state.
  virtual EdgePtr_t chooseEdge(RoadmapNodePtr_t from) const;

  /// Print the object in a stream.
  virtual std::ostream& dotPrint(
      std::ostream& os,
      dot::DrawingAttributes da = dot::DrawingAttributes()) const;

  /// Set the parent graph.
  void parentGraph(const GraphWkPtr_t& parent);

  /// Set the parent graph.
  GraphPtr_t parentGraph() const;

 protected:
  /// Initialization of the object.
  void init(const StateSelectorPtr_t& weak);

  /// Constructor
  StateSelector(const std::string& name) : name_(name) {}

  /// Print the object in a stream.
  virtual std::ostream& print(std::ostream& os) const;

  /// List of the states of one end-effector, ordered by priority.
  typedef std::pair<int, StatePtr_t> WeighedState_t;
  typedef std::list<WeighedState_t> WeighedStates_t;
  WeighedStates_t orderedStates_;
  States_t waypoints_;

 private:
  /// Name of the component.
  std::string name_;
  /// A weak pointer to the parent graph.
  GraphWkPtr_t graph_;
  /// Weak pointer to itself.
  StateSelectorPtr_t wkPtr_;

  friend std::ostream& operator<<(std::ostream& os, const StateSelector& ss);
};  // Class StateSelector

inline std::ostream& operator<<(std::ostream& os, const StateSelector& ss) {
  return ss.print(os);
}
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_GRAPH_STATE_SELECTOR_HH
