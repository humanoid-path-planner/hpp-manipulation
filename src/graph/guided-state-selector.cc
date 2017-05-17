// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation.
// hpp-manipulation is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation. If not, see <http://www.gnu.org/licenses/>.

#include "hpp/manipulation/graph/guided-state-selector.hh"

#include <hpp/util/assertion.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/core/steering-method.hh>

#include "../astar.hh"
#include "hpp/manipulation/roadmap.hh"
#include "hpp/manipulation/roadmap-node.hh"

#include <cstdlib>

namespace hpp {
  namespace manipulation {
    namespace graph {
      GuidedStateSelectorPtr_t GuidedStateSelector::create(const std::string& name,
          const core::RoadmapPtr_t& roadmap)
      {
        GuidedStateSelector* ptr = new GuidedStateSelector (name, roadmap);
        GuidedStateSelectorPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      void GuidedStateSelector::init (const GuidedStateSelectorPtr_t& weak)
      {
        StateSelector::init (weak);
        wkPtr_ = weak;
      }

      void GuidedStateSelector::setStateList (const States_t& stateList)
      {
        stateList_ = stateList;
      }

      EdgePtr_t GuidedStateSelector::chooseEdge(RoadmapNodePtr_t from) const
      {
        if (stateList_.empty ()) return StateSelector::chooseEdge (from);
        Astar::States_t list;
        bool reverse = false;
        if (from->connectedComponent () == roadmap_->initNode ()->connectedComponent ()) {
          Astar alg (roadmap_->distance (), wkPtr_.lock(), static_cast <RoadmapNodePtr_t> (roadmap_->initNode ()));
          list = alg.solution (from);
        } else {
          core::NodeVector_t::const_iterator itg = roadmap_->goalNodes ().begin ();
          for (; itg != roadmap_->goalNodes ().end (); ++itg)
            if ((*itg)->connectedComponent () == from->connectedComponent ())
              break;
          if (itg == roadmap_->goalNodes ().end ()) {
            hppDout (error, "This configuration can reach neither the initial "
                "configuration nor any of the goal configurations.");
            return EdgePtr_t ();
          }
          reverse = true;
          Astar alg (roadmap_->distance (), wkPtr_.lock(), from);
          list = alg.solution (static_cast <RoadmapNodePtr_t> (*itg));
        }
        list.erase (std::unique (list.begin(), list.end ()), list.end ());
        // Check if the beginning of stateList is list
        if (list.size() <= stateList_.size()) {
          Neighbors_t nn;
          if (reverse) {
            States_t::const_reverse_iterator it1 = stateList_.rbegin ();
            Astar::States_t::const_reverse_iterator it2 = list.rbegin();
            Astar::States_t::const_reverse_iterator itEnd2 = list.rend();
            do {
              if (*it1 != *it2) {
                hppDout (error, "The target sequence of nodes does not end with "
                    "the sequence of nodes to reach this configuration.");
                return EdgePtr_t ();
              }
              ++it1;
            } while (++it2 != itEnd2);
            StatePtr_t state = getState (from);
            HPP_ASSERT (state == list.front ());
            const Neighbors_t& n = state->neighbors();
            /// You stay in the same state
            for (Neighbors_t::const_iterator it = n.begin (); it != n.end (); ++it)
              if (it->second->to () == state)
                nn.insert (it->second, it->first);
            /// Go from state it1 to state
            // The path will be build from state. So we must find an edge from
            // state to it1, that will be reversely 
            for (Neighbors_t::const_iterator it = n.begin (); it != n.end (); ++it)
              if (it->second->to () == *it1)
                nn.insert (it->second, it->first);
          } else {
            States_t::const_iterator it1 = stateList_.begin ();
            Astar::States_t::const_iterator it2 = list.begin();
            Astar::States_t::const_iterator itEnd2 = list.end();
            do {
              if (*it1 != *it2) {
                hppDout (error, "The target sequence of nodes does not start with "
                    "the sequence of nodes to reach this configuration.");
                return EdgePtr_t ();
              }
              ++it1;
            } while (++it2 != itEnd2);
            StatePtr_t state = getState (from);
            HPP_ASSERT (state == list.back ());
            const Neighbors_t& n = state->neighbors();
            for (Neighbors_t::const_iterator it = n.begin (); it != n.end (); ++it)
              /// You stay in the same state
              /// or go from state to state it1 
              if (it->second->to () == state || it->second->to () == *it1)
                nn.insert (it->second, it->first);
          }
          if (nn.size () > 0 && nn.totalWeight() > 0)
            return nn ();
          hppDout (error, "This state has no neighbors to get to an admissible states.");
        }
        return EdgePtr_t ();
      }

      std::ostream& GuidedStateSelector::dotPrint (std::ostream& os, dot::DrawingAttributes) const
      {
        for (WeighedStates_t::const_iterator it = orderedStates_.begin();
            orderedStates_.end() != it; ++it)
          it->second->dotPrint (os);
        return os;
      }

      std::ostream& GuidedStateSelector::print (std::ostream& os) const
      {
        os << "|-- ";
        GraphComponent::print (os) << std::endl;
        for (WeighedStates_t::const_iterator it = orderedStates_.begin();
            orderedStates_.end() != it; ++it)
          os << it->first << " " << *it->second;
        return os;
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp
