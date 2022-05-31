//
// Copyright (c) 2015 CNRS
// Authors: Anna Seppala (seppala@laas.fr)
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

#ifndef HPP_MANIPULATION_CONNECTED_COMPONENT_HH
#define HPP_MANIPULATION_CONNECTED_COMPONENT_HH

#include <hpp/core/connected-component.hh>
#include <hpp/manipulation/config.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/fwd.hh>

namespace hpp {
namespace manipulation {
/// Extension of hpp::core::connected-component. Adds a list of roadmap nodes
/// for every contraint graph state within the connected component. Thus every
/// roadmap node is assigned to a grahp state, which minimises computation time.
class HPP_MANIPULATION_DLLAPI ConnectedComponent
    : public core::ConnectedComponent {
 public:
  /// Map of graph states within the connected component
  typedef std::map<graph::StatePtr_t, RoadmapNodes_t> GraphStates_t;

  ConnectedComponent() {}

  /// return a shared pointer to new instance of
  /// manipulation::ConnectedComponent
  static ConnectedComponentPtr_t create(const RoadmapWkPtr_t& roadmap);

  /// Merge two connected components (extension of
  /// core::ConnectedComponent::merge) \param other manipulation connected
  /// component to merge into this one. \note other will be empty after calling
  /// this method.
  void merge(const core::ConnectedComponentPtr_t& otherCC);

  /// Add roadmap node to connected component
  /// \param roadmap node to be added
  void addNode(const core::NodePtr_t& node);

  const RoadmapNodes_t& getRoadmapNodes(
      const graph::StatePtr_t graphState) const;

 protected:
 private:
  bool check() const;
  GraphStates_t graphStateMap_;
  // a RoadmapWkPtr_t so that memory can be released ?
  RoadmapWkPtr_t roadmap_;
  static RoadmapNodes_t empty_;

  HPP_SERIALIZABLE();
};  // class ConnectedComponent
}  //   namespace manipulation
}  // namespace hpp
#endif  // HPP_MANIPULATION_CONNECTED_COMPONENT_HH
