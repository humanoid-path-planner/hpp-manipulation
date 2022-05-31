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

#include <hpp/manipulation/connected-component.hh>

#include "hpp/manipulation/roadmap-node.hh"
#include "hpp/manipulation/roadmap.hh"

namespace hpp {
namespace manipulation {
RoadmapNodes_t ConnectedComponent::empty_ = RoadmapNodes_t();

bool ConnectedComponent::check() const {
  std::set<core::NodePtr_t> s1;
  for (core::NodeVector_t::const_iterator it = nodes().begin();
       it != nodes().end(); ++it) {
    s1.insert(*it);
  }
  std::set<core::NodePtr_t> s2;
  for (GraphStates_t::const_iterator it = graphStateMap_.begin();
       it != graphStateMap_.end(); ++it) {
    for (RoadmapNodes_t::const_iterator itNodes = it->second.begin();
         itNodes != it->second.end(); ++itNodes) {
      s2.insert(*itNodes);
    }
  }
  if (s1.size() == 0) return false;
  if (s1 == s2) return true;
  return false;
}

ConnectedComponentPtr_t ConnectedComponent::create(
    const RoadmapWkPtr_t& roadmap) {
  ConnectedComponent* ptr = new ConnectedComponent();
  ConnectedComponentPtr_t shPtr(ptr);
  // calls init function in core::ConnectedComponent that saves
  // shPtr into the class variable weak_ (weak pointer). Reimplement?
  ptr->init(shPtr);
  shPtr->roadmap_ = roadmap.lock();
  return shPtr;
}

void ConnectedComponent::merge(const core::ConnectedComponentPtr_t& otherCC) {
  core::ConnectedComponent::merge(otherCC);
  const ConnectedComponentPtr_t other =
      static_pointer_cast<ConnectedComponent>(otherCC);
  /// take all graph states in other->graphStateMap_ and put them in
  /// this->graphStateMap_ if they already exist in this->graphStateMap_, append
  /// roadmap nodes from other graph state to graph state in this.
  for (GraphStates_t::iterator otherIt = other->graphStateMap_.begin();
       otherIt != other->graphStateMap_.end(); otherIt++) {
    // find other graph state in this-graphStateMap_ -> merge their roadmap
    // nodes
    GraphStates_t::iterator mapIt = this->graphStateMap_.find(otherIt->first);
    if (mapIt != this->graphStateMap_.end()) {
      mapIt->second.insert(mapIt->second.end(), otherIt->second.begin(),
                           otherIt->second.end());
    } else {
      this->graphStateMap_.insert(*otherIt);
    }
  }
  other->graphStateMap_.clear();
  assert(check());
}

void ConnectedComponent::addNode(const core::NodePtr_t& node) {
  core::ConnectedComponent::addNode(node);
  // Find right graph state in map and add roadmap node to corresponding vector
  const RoadmapNodePtr_t& n = static_cast<const RoadmapNodePtr_t>(node);
  RoadmapPtr_t roadmap = roadmap_.lock();
  if (!roadmap)
    throw std::logic_error(
        "The roadmap of this ConnectedComponent as been deleted.");
  graphStateMap_[roadmap->getState(n)].push_back(n);
  assert(check());
}

const RoadmapNodes_t& ConnectedComponent::getRoadmapNodes(
    const graph::StatePtr_t graphState) const {
  GraphStates_t::const_iterator mapIt = graphStateMap_.find(graphState);
  if (mapIt != graphStateMap_.end()) return mapIt->second;
  return empty_;
}

}  // namespace manipulation
}  // namespace hpp
