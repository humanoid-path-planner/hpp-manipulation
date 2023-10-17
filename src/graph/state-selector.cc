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

#include "hpp/manipulation/graph/state-selector.hh"

#include <cstdlib>
#include <hpp/core/node.hh>
#include <hpp/pinocchio/configuration.hh>

#include "hpp/manipulation/roadmap-node.hh"

namespace hpp {
namespace manipulation {
namespace graph {
StateSelectorPtr_t StateSelector::create(const std::string& name) {
  StateSelector* ptr = new StateSelector(name);
  StateSelectorPtr_t shPtr(ptr);
  ptr->init(shPtr);
  return shPtr;
}

void StateSelector::init(const StateSelectorPtr_t& weak) { wkPtr_ = weak; }

StatePtr_t StateSelector::createState(const std::string& name, bool waypoint,
                                      const int w) {
  StatePtr_t newState = State::create(name);
  newState->stateSelector(wkPtr_);
  newState->parentGraph(graph_);
  newState->isWaypoint(waypoint);
  if (waypoint)
    waypoints_.push_back(newState);
  else {
    bool found = false;
    for (WeighedStates_t::iterator it = orderedStates_.begin();
         it != orderedStates_.end(); ++it) {
      if (it->first < w) {
        orderedStates_.insert(it, WeighedState_t(w, newState));
        found = true;
        break;
      }
    }
    if (!found) orderedStates_.push_back(WeighedState_t(w, newState));
  }
  return newState;
}

States_t StateSelector::getStates() const {
  States_t ret;
  for (WeighedStates_t::const_iterator it = orderedStates_.begin();
       it != orderedStates_.end(); ++it)
    ret.push_back(it->second);
  return ret;
}

States_t StateSelector::getWaypointStates() const { return waypoints_; }

StatePtr_t StateSelector::getState(ConfigurationIn_t config) const {
  for (WeighedStates_t::const_iterator it = orderedStates_.begin();
       orderedStates_.end() != it; ++it) {
    if (it->second->contains(config)) return it->second;
  }
  std::stringstream oss;
  oss << "A configuration has no node:" << pinocchio::displayConfig(config);
  throw std::logic_error(oss.str());
}

StatePtr_t StateSelector::getState(RoadmapNodePtr_t node) const {
  if (!node->cacheUpToDate())
    node->graphState(getState(node->configuration()));
  return node->graphState();
}

EdgePtr_t StateSelector::chooseEdge(RoadmapNodePtr_t from) const {
  StatePtr_t state = getState(from);
  const Neighbors_t neighborPicker = state->neighbors();
  if (neighborPicker.totalWeight() == 0) {
    return EdgePtr_t();
  }
  return neighborPicker();
}

std::ostream& StateSelector::dotPrint(std::ostream& os,
                                      dot::DrawingAttributes) const {
  for (WeighedStates_t::const_iterator it = orderedStates_.begin();
       orderedStates_.end() != it; ++it)
    it->second->dotPrint(os);
  return os;
}

std::ostream& StateSelector::print(std::ostream& os) const {
  for (WeighedStates_t::const_iterator it = orderedStates_.begin();
       orderedStates_.end() != it; ++it)
    os << it->first << " " << *it->second;
  return os;
}

GraphPtr_t StateSelector::parentGraph() const { return graph_.lock(); }

void StateSelector::parentGraph(const GraphWkPtr_t& parent) {
  graph_ = parent;
  GraphPtr_t g = graph_.lock();
  assert(g);
}

}  // namespace graph
}  // namespace manipulation
}  // namespace hpp
