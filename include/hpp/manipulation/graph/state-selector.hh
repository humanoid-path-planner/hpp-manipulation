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
      class HPP_MANIPULATION_DLLAPI StateSelector : public GraphComponent
      {
        public:
          virtual ~StateSelector () {};

          /// Create a new StateSelector.
          static StateSelectorPtr_t create(const std::string& name);

          /// Create an empty state
          StatePtr_t createState (const std::string& name, bool waypoint = false,
              const int w = 0);

          /// Returns the state of a configuration.
          StatePtr_t getState(ConfigurationIn_t config) const;

          /// Returns the state of a roadmap state
          StatePtr_t getState(RoadmapNodePtr_t node) const;

          /// Returns a list of all the states
          States_t getStates () const;

          /// Select randomly an outgoing edge of the given state.
          virtual EdgePtr_t chooseEdge(RoadmapNodePtr_t from) const;

          /// Should never be called.
          void addNumericalConstraint (const constraints::ImplicitPtr_t& /* function */,
              const segments_t& /* passiveDofs */ = segments_t ())
          {
            HPP_THROW_EXCEPTION (Bad_function_call, "This component does not have constraints.");
          }

          /// Should never be called.
          void addLockedJointConstraint
	    (const constraints::LockedJoint& /* constraint */)
          {
            HPP_THROW_EXCEPTION (Bad_function_call, "This component does not have constraints.");
          }

          /// Print the object in a stream.
          virtual std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

        protected:
          /// Initialization of the object.
          void init (const StateSelectorPtr_t& weak);

          /// Constructor
          StateSelector (const std::string& name) : GraphComponent (name)
          {}

          /// Print the object in a stream.
          virtual std::ostream& print (std::ostream& os) const;

          /// List of the states of one end-effector, ordered by priority.
          typedef std::pair <int, StatePtr_t> WeighedState_t;
          typedef std::list <WeighedState_t> WeighedStates_t;
          WeighedStates_t orderedStates_;
          States_t waypoints_;

          virtual void initialize () { isInit_ = true; };

        private:
          /// Weak pointer to itself.
          StateSelectorPtr_t wkPtr_;
      }; // Class StateSelector
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_STATE_SELECTOR_HH
