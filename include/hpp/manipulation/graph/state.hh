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

#ifndef HPP_MANIPULATION_GRAPH_STATE_HH
# define HPP_MANIPULATION_GRAPH_STATE_HH

# include <boost/function.hpp>

#include <hpp/core/constraint-set.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/constraints/implicit.hh>

#include "hpp/manipulation/config.hh"
#include "hpp/manipulation/deprecated.hh"
#include "hpp/manipulation/fwd.hh"
#include "hpp/manipulation/graph/fwd.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph-component.hh"

namespace hpp {
  namespace manipulation {
    using constraints::Implicit;
    using constraints::ImplicitPtr_t;
    namespace graph {
      /// \addtogroup constraint_graph
      /// \{

      /// State of an end-effector.
      ///
      /// States of the graph of constraints. There is one
      /// graph for each end-effector.
      class HPP_MANIPULATION_DLLAPI State : public GraphComponent
      {
        public:
	typedef boost::function < EdgePtr_t
				  (const std::string&,
				   const GraphWkPtr_t&,
				   const StateWkPtr_t&, const StateWkPtr_t&) >
	EdgeFactory;
          /// Destructor
          ~State ();

          /// Create a new state.
          static StatePtr_t create (const std::string& name);

          /// Create a link from this state to the given state.
          /// \param w if strictly negative, the edge is not included in the neighbor
          ///          list. Otherwise, it is included with Weight_t w
          EdgePtr_t linkTo (const std::string& name, const StatePtr_t& to,
			    const size_type& w = 1,
			    EdgeFactory create = Edge::create);

          /// Check whether the configuration is in this state.
          /// \code
          ///   return configConstraint()->isSatisfied (config);
          /// \endcode
          /// \note You should not use this method to know in which states a
          /// configuration is. This only checks if the configuration satisfies
          /// the constraints. Instead, use the class StateSelector.
          virtual bool contains (ConfigurationIn_t config) const;

          inline bool isWaypoint () const
          {
            return isWaypoint_;
          }

          inline void isWaypoint (bool isWaypoint)
          {
            isWaypoint_ = isWaypoint;
          }

          /// Get the parent StateSelector.
          StateSelectorWkPtr_t stateSelector () const
          {
            return selector_;
          }

          /// Set the StateSelector containing this state.
          void stateSelector (const StateSelectorWkPtr_t& parent)
          {
            selector_ = parent;
          };

          /// Get the neighbors
          const Neighbors_t& neighbors() const
          {
            return neighbors_;
          }

          /// Get the neighbors
          Edges_t neighborEdges() const
          {
            return neighbors_.values();
          }

          /// Get the hidden neighbors
          /// It is a vector of transitions outgoing from this state and that are
          /// included in a waypoint edge.
          const Edges_t& hiddenNeighbors() const
          {
            return hiddenNeighbors_;
          }

          void updateWeight (const EdgePtr_t&edge, const Weight_t& w);

          Weight_t getWeight (const EdgePtr_t&edge);

          /// Constraint to project onto this state.
          ConstraintSetPtr_t configConstraint() const
          {
            throwIfNotInitialized ();
            return configConstraints_;
          }

          /// Add constraints::Implicit to the component.
          virtual void addNumericalConstraintForPath (const ImplicitPtr_t& nm,
              const segments_t& passiveDofs = segments_t ())
          {
            isInit_ = false;
            numericalConstraintsForPath_.push_back (nm);
            passiveDofsForPath_.push_back (passiveDofs);
          }

          /// Add core::DifferentiableFunction to the component.
          virtual void addNumericalConstraintForPath (const DifferentiableFunctionPtr_t& function, const ComparisonTypes_t& ineq)
            HPP_MANIPULATION_DEPRECATED
          {
            isInit_ = false;
            numericalConstraintsForPath_.push_back
              (Implicit::create (function,ineq));
          }

          /// Insert the numerical constraints in a ConfigProjector
          /// \return true is at least one ImplicitPtr_t was inserted.
          bool insertNumericalConstraintsForPath (ConfigProjectorPtr_t& proj) const
          {
            assert (numericalConstraintsForPath_.size () == passiveDofsForPath_.size ());
            IntervalsContainer_t::const_iterator itpdofs = passiveDofsForPath_.begin ();
            for (NumericalConstraints_t::const_iterator it = numericalConstraintsForPath_.begin();
                it != numericalConstraintsForPath_.end(); it++) {
              proj->add (*it, *itpdofs);
              itpdofs++;
            }
            return !numericalConstraintsForPath_.empty ();
          }

          /// Get a reference to the NumericalConstraints_t
          const NumericalConstraints_t& numericalConstraintsForPath () const
          {
            return numericalConstraintsForPath_;
          }

          /// Print the object in a stream.
          std::ostream& dotPrint (std::ostream& os, dot::DrawingAttributes da = dot::DrawingAttributes ()) const;

        protected:
          /// Initialize the object.
          void init (const StateWkPtr_t& self);

          /// Constructor
          State(const std::string& name);

          /// Print the object in a stream.
          std::ostream& print (std::ostream& os) const;

          virtual void populateTooltip (dot::Tooltip& tp) const;

          virtual void initialize ();

        private:
          /// List of possible motions from this state (i.e. the outgoing
          /// vertices).
          Neighbors_t neighbors_;
          Edges_t hiddenNeighbors_;

          /// Set of constraints to be statisfied.
          ConstraintSetPtr_t configConstraints_;

          /// Stores the numerical constraints for path.
          NumericalConstraints_t numericalConstraintsForPath_;
          IntervalsContainer_t passiveDofsForPath_;

          /// A selector that will implement the selection of the next state.
          StateSelectorWkPtr_t selector_;

          /// Weak pointer to itself.
          StateWkPtr_t wkPtr_;

          bool isWaypoint_;
      }; // class State

      /// \}
    } // namespace graph
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_GRAPH_STATE_HH
