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

#ifndef HPP_MANIPULATION_GRAPH_GUIDED_STATE_SELECTOR_HH
#define HPP_MANIPULATION_GRAPH_GUIDED_STATE_SELECTOR_HH

#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/graph/state-selector.hh"

namespace hpp {
namespace manipulation {
namespace graph {
class HPP_MANIPULATION_DLLAPI GuidedStateSelector : public StateSelector {
 public:
  /// Create a new GuidedStateSelector.
  static GuidedStateSelectorPtr_t create(const std::string& name,
                                         const core::RoadmapPtr_t& roadmap);

  /// Set the target
  void setStateList(const States_t& stateList);

  /// Select randomly an outgoing edge of the given node.
  virtual EdgePtr_t chooseEdge(RoadmapNodePtr_t from) const;

  /// Print the object in a stream.
  std::ostream& dotPrint(std::ostream& os, dot::DrawingAttributes da =
                                               dot::DrawingAttributes()) const;

 protected:
  /// Initialization of the object.
  void init(const GuidedStateSelectorPtr_t& weak);

  /// Constructor
  GuidedStateSelector(const std::string& name, const core::RoadmapPtr_t roadmap)
      : StateSelector(name), roadmap_(roadmap) {}

  /// Print the object in a stream.
  std::ostream& print(std::ostream& os) const;

 private:
  /// The target
  States_t stateList_;

  /// The roadmap
  core::RoadmapPtr_t roadmap_;

  /// Weak pointer to itself.
  GuidedStateSelectorWkPtr_t wkPtr_;
};  // Class StateSelector
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_GRAPH_GUIDED_STATE_SELECTOR_HH
