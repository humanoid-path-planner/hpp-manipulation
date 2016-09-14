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

#include <hpp/core/node.hh>

#include <hpp/model/configuration.hh>
#include "hpp/manipulation/roadmap-node.hh"
#include "hpp/manipulation/graph/state-selector.hh"

#include <cstdlib>

namespace hpp {
  namespace manipulation {
    namespace graph {
      StateSelectorPtr_t StateSelector::create(const std::string& name)
      {
        StateSelector* ptr = new StateSelector (name);
        StateSelectorPtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      void StateSelector::init (const StateSelectorPtr_t& weak)
      {
        GraphComponent::init (weak);
        wkPtr_ = weak;
      }

      StatePtr_t StateSelector::createState (const std::string& name,
          bool waypoint, const int w)
      {
        StatePtr_t newState = State::create (name);
        newState->stateSelector(wkPtr_);
        newState->parentGraph(graph_);
        newState->isWaypoint (waypoint);
        if (waypoint) waypoints_.push_back(newState);
        else {
          bool found = false;
          for (WeighedStates_t::iterator it = orderedStates_.begin();
              it != orderedStates_.end (); ++it) {
            if (it->first < w) {
              orderedStates_.insert (it, WeighedState_t(w,newState));
              found = true;
              break;
            }
          }
          if (!found) 
            orderedStates_.push_back (WeighedState_t(w,newState));
        }
        return newState;
      }

      States_t StateSelector::getStates () const
      {
        States_t ret;
        for (WeighedStates_t::const_iterator it = orderedStates_.begin();
            it != orderedStates_.end (); ++it)
          ret.push_back (it->second);
        return ret;
      }

      StatePtr_t StateSelector::getState(ConfigurationIn_t config) const
      {
        for (WeighedStates_t::const_iterator it = orderedStates_.begin();
	     orderedStates_.end() != it; ++it) {
          if (it->second->contains(config))
            return it->second;
	}
	std::stringstream oss;
	oss << "A configuration has no state:" << model::displayConfig (config);
	throw std::logic_error (oss.str ());
      }

      StatePtr_t StateSelector::getState(RoadmapNodePtr_t node) const
      {
        StatePtr_t n;
        switch (node->cachingSystem ()) {
          case RoadmapNode::CACHE_UP_TO_DATE:
            n = node->graphState ();
            break;
          case RoadmapNode::CACHE_DISABLED:
          case RoadmapNode::CACHE_NEED_UPDATE:
            n = getState (*(node->configuration ()));
            node->graphState (n);
            break;
          default:
            n = getState (*(node->configuration ()));
            hppDout (error, "Unimplemented caching system.");
            break;
        }
        return n;
      }

      EdgePtr_t StateSelector::chooseEdge(RoadmapNodePtr_t from) const
      {
        StatePtr_t state = getState (from);
        const Neighbors_t neighborPicker = state->neighbors();
        if (neighborPicker.totalWeight () == 0) {
          return EdgePtr_t ();
        }
        return neighborPicker ();
      }

      std::ostream& StateSelector::dotPrint (std::ostream& os, dot::DrawingAttributes) const
      {
        for (WeighedStates_t::const_iterator it = orderedStates_.begin();
            orderedStates_.end() != it; ++it)
          it->second->dotPrint (os);
        return os;
      }

      std::ostream& StateSelector::print (std::ostream& os) const
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
