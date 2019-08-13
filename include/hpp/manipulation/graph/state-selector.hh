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

#ifndef HPP_MANIPULATION_GRAPH_STATE_SELECTOR_HH
# define HPP_MANIPULATION_GRAPH_STATE_SELECTOR_HH

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/state.hh"

namespace hpp {
  namespace manipulation {
    namespace graph {
      /// This class is used to get the state of a configuration. States have to
      /// be ordered because a configuration can be in several states.
      class HPP_MANIPULATION_DLLAPI StateSelector
      {
        public:
          virtual ~StateSelector () {};

          /// Create a new StateSelector.
          static StateSelectorPtr_t create(const std::string& name);

          const std::string& name() const
          {
            return name_;
          }

          /// Create an empty state
          StatePtr_t createState (const std::string& name, bool waypoint = false,
              const int w = 0);

          /// Returns the state of a configuration.
          StatePtr_t getState(ConfigurationIn_t config) const;

          /// Returns the state of a roadmap state
          StatePtr_t getState(RoadmapNodePtr_t node) const;

          /// Returns a list of all the states
          States_t getStates () const;

          /// Returns a list of all the states
          States_t getWaypointStates () const;

          /// Select randomly an outgoing edge of the given state.
          virtual EdgePtr_t chooseEdge(RoadmapNodePtr_t from) const;

          /// Print the object in a stream.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

          /// Set the parent graph.
          void parentGraph(const GraphWkPtr_t& parent);

          /// Set the parent graph.
          GraphPtr_t parentGraph() const;

        protected:
          /// Initialization of the object.
          void init (const StateSelectorPtr_t& weak);

          /// Constructor
          StateSelector (const std::string& name) : name_ (name)
          {}

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

          /// List of the states of one end-effector, ordered by priority.
          typedef std::pair <int, StatePtr_t> WeighedState_t;
          typedef std::list <WeighedState_t> WeighedStates_t;
          WeighedStates_t orderedStates_;
          States_t waypoints_;

        private:
          /// Name of the component.
          std::string name_;
          /// A weak pointer to the parent graph.
          GraphWkPtr_t graph_;
          /// Weak pointer to itself.
          StateSelectorPtr_t wkPtr_;

          friend std::ostream& operator<< (std::ostream& os, const StateSelector& ss);
      }; // Class StateSelector

      inline std::ostream& operator<< (std::ostream& os, const StateSelector& ss)
      {
        return ss.print(os);
      }
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_STATE_SELECTOR_HH
