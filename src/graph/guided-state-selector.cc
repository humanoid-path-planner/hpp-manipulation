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

#include "hpp/manipulation/graph/guided-state-selector.hh"

#include <cstdlib>
#include <hpp/core/steering-method.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/util/assertion.hh>

#include "../astar.hh"
#include "hpp/manipulation/roadmap-node.hh"
#include "hpp/manipulation/roadmap.hh"

namespace hpp {
namespace manipulation {
namespace graph {
GuidedStateSelectorPtr_t GuidedStateSelector::create(
    const std::string& name, const core::RoadmapPtr_t& roadmap) {
  GuidedStateSelector* ptr = new GuidedStateSelector(name, roadmap);
  GuidedStateSelectorPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

void GuidedStateSelector::init(const GuidedStateSelectorPtr_t& weak) {
  StateSelector::init(weak);
  wkPtr_ = weak;
}

void GuidedStateSelector::setStateList(const States_t& stateList) {
  stateList_ = stateList;
}

EdgePtr_t GuidedStateSelector::chooseEdge(RoadmapNodePtr_t from) const {
  if (stateList_.empty()) return StateSelector::chooseEdge(from);
  Astar::States_t list;
  bool reverse = false;
  if (from->connectedComponent() ==
      roadmap_->initNode()->connectedComponent()) {
    Astar alg(roadmap_->distance(), wkPtr_.lock(),
              static_cast<RoadmapNodePtr_t>(roadmap_->initNode()));
    list = alg.solution(from);
  } else {
    core::NodeVector_t::const_iterator itg = roadmap_->goalNodes().begin();
    for (; itg != roadmap_->goalNodes().end(); ++itg)
      if ((*itg)->connectedComponent() == from->connectedComponent()) break;
    if (itg == roadmap_->goalNodes().end()) {
      hppDout(error,
              "This configuration can reach neither the initial "
              "configuration nor any of the goal configurations.");
      return EdgePtr_t();
    }
    reverse = true;
    Astar alg(roadmap_->distance(), wkPtr_.lock(), from);
    list = alg.solution(static_cast<RoadmapNodePtr_t>(*itg));
  }
  list.erase(std::unique(list.begin(), list.end()), list.end());
  // Check if the beginning of stateList is list
  if (list.size() <= stateList_.size()) {
    Neighbors_t nn;
    if (reverse) {
      States_t::const_reverse_iterator it1 = stateList_.rbegin();
      Astar::States_t::const_reverse_iterator it2 = list.rbegin();
      Astar::States_t::const_reverse_iterator itEnd2 = list.rend();
      do {
        if (*it1 != *it2) {
          hppDout(error,
                  "The target sequence of nodes does not end with "
                  "the sequence of nodes to reach this configuration.");
          return EdgePtr_t();
        }
        ++it1;
      } while (++it2 != itEnd2);
      StatePtr_t state = getState(from);
      HPP_ASSERT(state == list.front());
      const Neighbors_t& n = state->neighbors();
      /// You stay in the same state
      for (Neighbors_t::const_iterator it = n.begin(); it != n.end(); ++it)
        if (it->second->stateTo() == state) nn.insert(it->second, it->first);
      /// Go from state it1 to state
      // The path will be build from state. So we must find an edge from
      // state to it1, that will be reversely
      for (Neighbors_t::const_iterator it = n.begin(); it != n.end(); ++it)
        if (it->second->stateTo() == *it1) nn.insert(it->second, it->first);
    } else {
      States_t::const_iterator it1 = stateList_.begin();
      Astar::States_t::const_iterator it2 = list.begin();
      Astar::States_t::const_iterator itEnd2 = list.end();
      do {
        if (*it1 != *it2) {
          hppDout(error,
                  "The target sequence of nodes does not start with "
                  "the sequence of nodes to reach this configuration.");
          return EdgePtr_t();
        }
        ++it1;
      } while (++it2 != itEnd2);
      StatePtr_t state = getState(from);
      HPP_ASSERT(state == list.back());
      const Neighbors_t& n = state->neighbors();
      for (Neighbors_t::const_iterator it = n.begin(); it != n.end(); ++it)
        /// You stay in the same state
        /// or go from state to state it1
        if (it->second->stateTo() == state || it->second->stateTo() == *it1)
          nn.insert(it->second, it->first);
    }
    if (nn.size() > 0 && nn.totalWeight() > 0) return nn();
    hppDout(error,
            "This state has no neighbors to get to an admissible states.");
  }
  return EdgePtr_t();
}

std::ostream& GuidedStateSelector::dotPrint(std::ostream& os,
                                            dot::DrawingAttributes) const {
  for (WeighedStates_t::const_iterator it = orderedStates_.begin();
       orderedStates_.end() != it; ++it)
    it->second->dotPrint(os);
  return os;
}

std::ostream& GuidedStateSelector::print(std::ostream& os) const {
  return StateSelector::print(os);
}
}  // namespace graph
}  // namespace manipulation
}  // namespace hpp
