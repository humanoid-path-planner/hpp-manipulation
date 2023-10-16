//
// Copyright (c) 2015 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel
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

#ifndef HPP_MANIPULATION_ASTAR_HH
#define HPP_MANIPULATION_ASTAR_HH

#include <hpp/core/distance.hh>
#include <hpp/core/edge.hh>
#include <hpp/core/node.hh>
#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/graph/state-selector.hh>
#include <hpp/manipulation/roadmap-node.hh>
#include <limits>
// # include <hpp/core/path-vector.hh>

namespace hpp {
namespace manipulation {
class Astar {
 public:
  typedef std::map<RoadmapNodePtr_t, value_type> CostMap_t;
  struct CostMapCompFunctor {
    CostMap_t& cost_;
    CostMapCompFunctor(CostMap_t& cost) : cost_(cost) {}
    bool operator()(const RoadmapNodePtr_t& n1, const RoadmapNodePtr_t& n2) {
      return cost_[n1] < cost_[n2];
    }
    bool operator()(const RoadmapNodePtr_t& n1, const value_type& val) {
      return cost_[n1] < val;
    }
  };  // struc CostMapCompFunctor

  typedef std::list<graph::StatePtr_t> States_t;
  typedef std::list<RoadmapNodePtr_t> RoadmapNodes_t;
  typedef std::list<core::EdgePtr_t> RoadmapEdges_t;
  typedef std::map<RoadmapNodePtr_t, core::EdgePtr_t> Parent_t;

  Astar(const core::DistancePtr_t distance,
        const graph::StateSelectorPtr_t& stateSelector, RoadmapNodePtr_t from)
      : distance_(distance), selector_(stateSelector), from_(from) {
    open_.push_back(from);
    costFromStart_[from] = 0;
  }

  States_t solution(RoadmapNodePtr_t to) {
    if (parent_.find(to) != parent_.end() || findPath(to)) {
      RoadmapNodePtr_t node = to;
      States_t states;

      states.push_front(selector_->getState(to));
      while (node) {
        Parent_t::const_iterator itNode = parent_.find(node);
        if (itNode != parent_.end()) {
          node = static_cast<RoadmapNodePtr_t>(itNode->second->from());
          states.push_front(selector_->getState(node));
        } else
          node = RoadmapNodePtr_t(0);
      }
      // We may want to clean it a little
      // std::unique (states.begin(), states.end ());

      states.push_front(selector_->getState(from_));
      return states;
    }
    return States_t();
  }

 private:
  bool findPath(RoadmapNodePtr_t to) {
    // Recompute the estimated cost to goal
    for (CostMap_t::iterator it = estimatedCostToGoal_.begin();
         it != estimatedCostToGoal_.end(); ++it) {
      it->second = getCostFromStart(it->first) + heuristic(it->first, to);
    }
    open_.sort(CostMapCompFunctor(estimatedCostToGoal_));

    while (!open_.empty()) {
      RoadmapNodes_t::iterator itv = open_.begin();
      RoadmapNodePtr_t current(*itv);
      if (current == to) {
        return true;
      }
      open_.erase(itv);
      closed_.push_back(current);
      for (RoadmapEdges_t::const_iterator itEdge = current->outEdges().begin();
           itEdge != current->outEdges().end(); ++itEdge) {
        RoadmapNodePtr_t child = static_cast<RoadmapNodePtr_t>((*itEdge)->to());
        if (std::find(closed_.begin(), closed_.end(), child) == closed_.end()) {
          // node is not in closed set
          value_type transitionCost = edgeCost(*itEdge);
          value_type tmpCost = getCostFromStart(current) + transitionCost;
          bool childNotInOpenSet =
              (std::find(open_.begin(), open_.end(), child) == open_.end());
          if ((childNotInOpenSet) || (tmpCost < getCostFromStart(child))) {
            parent_[child] = *itEdge;
            costFromStart_[child] = tmpCost;
            value_type estimatedCost = tmpCost + heuristic(child, to);
            estimatedCostToGoal_[child] = estimatedCost;
            if (childNotInOpenSet) {
              // Find the first element not strictly smaller than child
              RoadmapNodes_t::iterator pos =
                  std::lower_bound(open_.begin(), open_.end(), estimatedCost,
                                   CostMapCompFunctor(estimatedCostToGoal_));
              open_.insert(pos, child);
            }
          }
        }
      }
    }
    return false;
  }

  inline value_type heuristic(RoadmapNodePtr_t node,
                              RoadmapNodePtr_t to) const {
    const Configuration_t& config = node->configuration();
    return (*distance_)(config, to->configuration());
  }

  inline value_type edgeCost(const core::EdgePtr_t& edge) const {
    return edge->path()->length();
  }

  value_type getCostFromStart(RoadmapNodePtr_t to) const {
    CostMap_t::const_iterator it = costFromStart_.find(to);
    if (it == costFromStart_.end())
      return std::numeric_limits<value_type>::max();
    return it->second;
  }

  RoadmapNodes_t closed_;
  RoadmapNodes_t open_;
  std::map<RoadmapNodePtr_t, value_type> costFromStart_;
  std::map<RoadmapNodePtr_t, value_type> estimatedCostToGoal_;
  Parent_t parent_;
  core::DistancePtr_t distance_;
  graph::StateSelectorPtr_t selector_;
  RoadmapNodePtr_t from_;

};  // class Astar
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_ASTAR_HH
